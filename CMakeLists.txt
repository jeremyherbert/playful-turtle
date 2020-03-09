cmake_minimum_required(VERSION 3.15)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/cmake/arm-gcc-toolchain.cmake)
project(playful-turtle)
#set(CMAKE_VERBOSE_MAKEFILE ON)

enable_language(ASM)

set(CMAKE_C_STANDARD 99)
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

include(cmake/utils.cmake)

set(CPU_FLAGS "-mthumb -mcpu=cortex-m0 -mfloat-abi=soft -fdata-sections -ffunction-sections -ffreestanding --specs=nano.specs -lc -lnosys")

add_definitions(${CPU_FLAGS} ${GCC_FLAGS} -g -Os -DSTM32F042x6 -DUSE_FULL_LL_DRIVER)

set(LINKER_FILE ${CMAKE_CURRENT_LIST_DIR}/STM32F042F6PX_FLASH.ld)
string(APPEND CMAKE_EXE_LINKER_FLAGS " ${CPU_FLAGS} -Wl,-T ${LINKER_FILE}")
string(APPEND CMAKE_EXE_LINKER_FLAGS " -Wl,--gc-sections -Wl,-Map=output.map")

set(RTT_FILES   libs/rtt/SEGGER_RTT.c
                libs/rtt/SEGGER_RTT_printf.c)

set(TINYUSB_FILES   libs/tinyusb/src/class/cdc/cdc_device.c
                    libs/tinyusb/src/class/hid/hid_device.c
                    libs/tinyusb/src/common/tusb_fifo.c
                    libs/tinyusb/src/device/usbd.c
                    libs/tinyusb/src/device/usbd_control.c
                    libs/tinyusb/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
                    libs/tinyusb/src/tusb.c)

set(HAL_SRC_DIR libs/st/STM32F0xx_HAL_Driver/Src)
set(HAL_FILES       ${HAL_SRC_DIR}/stm32f0xx_ll_adc.c
                    ${HAL_SRC_DIR}/stm32f0xx_ll_comp.c
                    ${HAL_SRC_DIR}/stm32f0xx_ll_crc.c
                    ${HAL_SRC_DIR}/stm32f0xx_ll_crs.c
                    ${HAL_SRC_DIR}/stm32f0xx_ll_dac.c
                    ${HAL_SRC_DIR}/stm32f0xx_ll_dma.c
                    ${HAL_SRC_DIR}/stm32f0xx_ll_exti.c
                    ${HAL_SRC_DIR}/stm32f0xx_ll_gpio.c
                    ${HAL_SRC_DIR}/stm32f0xx_ll_i2c.c
                    ${HAL_SRC_DIR}/stm32f0xx_ll_pwr.c
                    ${HAL_SRC_DIR}/stm32f0xx_ll_rcc.c
                    ${HAL_SRC_DIR}/stm32f0xx_ll_rtc.c
                    ${HAL_SRC_DIR}/stm32f0xx_ll_spi.c
                    ${HAL_SRC_DIR}/stm32f0xx_ll_tim.c
                    ${HAL_SRC_DIR}/stm32f0xx_ll_usart.c
                    ${HAL_SRC_DIR}/stm32f0xx_ll_usb.c
                    ${HAL_SRC_DIR}/stm32f0xx_ll_utils.c)

set(MCU_FILES       src/startup_stm32f042f6px.s
                    src/system_stm32f0xx.c)

set(SRC_FILES       src/main.c
                    src/descriptors.c src/stm32f0xx_hal_conf.h)

include_directories(src
                    libs/ringbuf
                    libs/rtt
                    libs/tinyusb/src
                    libs/tinycbor

                    libs/st/CMSIS/Device/ST/STM32F0xx/Include
                    libs/st/CMSIS/Include

                    libs/st/STM32F0xx_HAL_Driver/Inc)

add_executable(playful_turtle ${SRC_FILES} libs/ringbuf/ringbuf.c ${MCU_FILES} ${RTT_FILES} ${TINYUSB_FILES} ${TINYCBOR_FILES} ${HAL_FILES})

firmware_size(playful_turtle)
generate_object(playful_turtle .bin binary)