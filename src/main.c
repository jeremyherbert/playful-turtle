/* SPDX-License-Identifier: MIT */

#include "stm32f0xx.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_crs.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_spi.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_dma.h"

#include "ringbuf.h"
#include "SEGGER_RTT.h"
#include "tusb.h"

volatile uint8_t led_blink_ms = 500;

/*****************************************************
 * HID/SPI interface
 ****************************************************/

// HID data storage/state
volatile bool hid_pending = false;
uint8_t hid_buf[8];

// SPI configuration
void spi_init() {
    LL_SPI_InitTypeDef SPI_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SPI1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    NVIC_SetPriority(SPI1_IRQn, 0);
    NVIC_EnableIRQ(SPI1_IRQn);

    SPI_InitStruct.TransferDirection = LL_SPI_SIMPLEX_RX;
    SPI_InitStruct.Mode = LL_SPI_MODE_SLAVE;
    SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
    SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
    SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
    SPI_InitStruct.NSS = LL_SPI_NSS_HARD_INPUT;
    SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
    SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
    SPI_InitStruct.CRCPoly = 7;
    LL_SPI_Init(SPI1, &SPI_InitStruct);
    LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
    LL_SPI_DisableNSSPulseMgt(SPI1);

    LL_SPI_EnableIT_RXNE(SPI1);
    LL_SPI_Enable(SPI1);
}

// handler to write SPI data to HID buffer on IRQ
void SPI1_IRQHandler() {
    if (LL_SPI_IsActiveFlag_RXNE(SPI1)) {
        uint16_t rx16 = LL_SPI_ReceiveData16(SPI1);
        uint8_t addr = (rx16 & 0xFF00) >> 8;
        if (addr < 8) {
            hid_buf[addr] = rx16 & 0xFF;
        } else if (addr == 0xFF) {
            hid_pending = true;
        }
    }
}

// HID get report callback
uint16_t tud_hid_get_report_cb(uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
    // dummy implementation, should never be used anyway
    SEGGER_RTT_printf(0, "tud_hid_get_report_cb: report_id: %d, report_type: %d, reqlen: %d\n", report_id, report_type,
                      reqlen);

    for (int i = 0; i < reqlen; i++) {
        buffer[i] = i;
    }

    return reqlen;
}

// HID set report callback
void tud_hid_set_report_cb(uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
    // unimplemented as this is not needed

    SEGGER_RTT_printf(0, "tud_hid_set_report_cb: report_id: %d, report_type: %d, bufsize: %d\n", report_id, report_type,
                      bufsize);
}

// HID process task; sends a HID report if there is a new one pending
void hid_process(void) {
    uint8_t buf[8];

    if (hid_pending) {
        __disable_irq();
        memcpy(buf, &hid_buf[0], 8);
        __enable_irq();

        tud_hid_report(0, buf, 8);
        led_blink_ms = 100;
        SEGGER_RTT_printf(0, "hid report sent\n");
        hid_pending = false;
    }
}

/*****************************************************
 * CDC/UART interface
 ****************************************************/

// CDC data
uint8_t buf_to_host[256];
ringbuf_t ringbuf_to_host;
uint8_t dma_buf[256];
volatile bool dma_tx_running = false;

// UART and DMA configuration
void uart_init(uint32_t baud_rate) {
    ringbuf_init(&ringbuf_to_host, &buf_to_host[0], sizeof(buf_to_host));

    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    // init GPIOs
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // init USART peripheral
    USART_InitStruct.BaudRate = baud_rate;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;

    LL_USART_Init(USART2, &USART_InitStruct);
    LL_USART_DisableIT_CTS(USART2);
    LL_USART_EnableIT_RXNE(USART2);
    NVIC_EnableIRQ(USART2_IRQn);
    LL_USART_ConfigAsyncMode(USART2);
    LL_USART_Enable(USART2);

    // init DMA
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);

    // set DMA addresses
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_4, (uint32_t) dma_buf,
                           LL_USART_DMA_GetRegAddr(USART2, LL_USART_DMA_REG_DATA_TRANSMIT),
                           LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4));
    LL_USART_EnableDMAReq_TX(USART2);

    // enable DMA interrupt handler
    NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0);
    NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);
}

// UART RX handler
void USART2_IRQHandler() {
    if (LL_USART_IsActiveFlag_RXNE(USART2)) {
        uint8_t byte = LL_USART_ReceiveData8(USART2);
        ringbuf_push(&ringbuf_to_host, &byte, 1);
    }
    if (LL_USART_IsActiveFlag_ORE(USART2)) {
        LL_USART_ClearFlag_ORE(USART2); // ignore overrun
    }
}

// DMA finished handler (runs when UART TX has finished)
void DMA1_Channel4_5_IRQHandler() {
    if (LL_DMA_IsActiveFlag_TC4(DMA1)) {
        LL_DMA_ClearFlag_TC4(DMA1);
        dma_tx_running = false;
    }
}

// shuffle data between CDC interface and UART
void cdc_process() {
    if (tud_cdc_connected()) {
        // rx from usb host (tx to UART via DMA)
        if (tud_cdc_available()) {
            if (!dma_tx_running) {
                uint32_t count = tud_cdc_read(dma_buf, 256);

                if (count) {
                    dma_tx_running = true;
                    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
                    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, count);
                    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
                    led_blink_ms = 100;
                }
            }
        }

        // tx to usb host
        uint8_t buf[64];
        LL_USART_DisableIT_RXNE(USART2);
        size_t count = ringbuf_pop(&ringbuf_to_host, buf, 64);
        LL_USART_EnableIT_RXNE(USART2);
        if (count) {
            tud_cdc_write(buf, count);
            led_blink_ms = 100;
        }
        tud_cdc_write_flush();
    } else {
        // empty the buffer if CDC is disconnected
        uint8_t buf[64];
        LL_USART_DisableIT_RXNE(USART2);
        ringbuf_pop(&ringbuf_to_host, buf, 64);
        LL_USART_EnableIT_RXNE(USART2);
    }
}

/*****************************************************
 * Main system code
 ****************************************************/

// core configuration
void system_init() {
    // remap PA11 and PA12 to the USB device peripheral
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    LL_SYSCFG_EnablePinRemap();

    // set flash latency
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
    if (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1) {
        NVIC_SystemReset();
    }
    LL_RCC_HSI_Enable();

    // wait for HSI
    while (LL_RCC_HSI_IsReady() != 1);
    LL_RCC_HSI_SetCalibTrimming(16);
    LL_RCC_HSI48_Enable();

    // wait for HSI48
    while (LL_RCC_HSI48_IsReady() != 1);
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI48);

    // wait for system clock
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI48);

    // set clock sources
    LL_SetSystemCoreClock(48000000);
    LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
    LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_HSI48);

    // configure and enable CRS
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CRS);
    LL_CRS_SetSyncDivider(LL_CRS_SYNC_DIV_1);
    LL_CRS_SetSyncPolarity(LL_CRS_SYNC_POLARITY_RISING);
    LL_CRS_SetSyncSignalSource(LL_CRS_SYNC_SOURCE_USB);
    LL_CRS_SetFreqErrorLimit(34);
    LL_CRS_SetHSI48SmoothTrimming(32);

    // set systick to be 1ms
    SysTick_Config(SystemCoreClock / 1000);

    // update system core clock
    SystemCoreClockUpdate();

    // set priority of PendSV
    NVIC_SetPriority(PendSV_IRQn, 3);

    // enable peripheral and GPIO clocks
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USB);

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
}

void led_gpio_init() {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);
}

int main() {
    system_init();
    led_gpio_init();
    spi_init();
    uart_init(9600); // force 9600 baud, do not change based on USB request
    memset(hid_buf, 0, 8); // initialise HID buffer to zeros

    tusb_init();
    SEGGER_RTT_printf(0, "Started\n");

    while (1) {
        tud_task(); // handle USB packets
        cdc_process(); // handle CDC data
        hid_process(); // transmit HID reports
    }
}

volatile uint8_t tick_count = 0;
void SysTick_Handler(void) {
    bool blinking = false;

    if (led_blink_ms == 0) {
        blinking = false;
    } else {
        led_blink_ms--;
        blinking = true;
    }

    if (blinking) {
        if (tick_count++ >= 50) {
            tick_count = 0;
        }
        if (tick_count < 25) {
            LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);
        } else {
            LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);
        }
    } else {
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);
        tick_count = 0;
    }
}

void NMI_Handler(void) {
    // unused
}

void HardFault_Handler(void) {
    NVIC_SystemReset(); // reset on hard fault
}

void SVC_Handler(void) {
    // unused
}

void PendSV_Handler(void) {
    // unused
}