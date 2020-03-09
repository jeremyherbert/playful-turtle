# playful-turtle

This repository contains a firmware implementation of a composite USB device which exposes the following functionality:

- USB to UART conversion (fixed at a baudrate of 9600)
- USB HID gamepad/joystick interface via SPI

It is designed to be used with the [turtleboard hardware](https://github.com/jeremyherbert/turtleboard) (STM32F042F6P6). 

**Important notes:**

- The turtleboard supports 0V to 3.3V digital signals. Connecting signals outside this range will likely cause permanent damage to the device.
- For simplicity, the USB to UART baudrate is fixed at 9600. The device will ignore requests from the PC to change this.

## Usage

### USB to UART
To use the USB to UART converter, simply connect the device to a free USB port on your computer. Drivers should be automatically installed on all platforms that need them. The device will appear in your operating system as a serial port.

### HID gamepad/joystick

The device supports 16 buttons, 2 joystick axes, 3 rotation axes and 4 direction pad buttons. Much like the USB to UART interface, drivers should be automatically installed on all platforms that need them.

The gamepad data can be sent using the SPI interface on the turtleboard; it behaves as a receive-only SPI slave (so there is no need to connect MISO as it is unused). The interface uses Motorola SPI mode 0, meaning that the clock polarity is active low, and the phase polarity is first/rising edge. The maximum supported clock speed of the interface is 1MHz.

All SPI transactions are two bytes long. The first byte is a register address, and the second byte contains the gamepad data. A list of the possible registers is below.

| Register address | Name        | Format                      | bit 7     | bit 6     | bit 5     | bit 4     | bit 3     | bit 2     | bit 1     | bit 0    |
|------------------|-------------|-----------------------------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|----------|
| 0x00             | BUTTON0     | 1 bit per button, 8 buttons | button 8  | button 7  | button 6  | button 5  | button 4  | button 3  | button 2  | button 1 |
| 0x01             | BUTTON1     | 1 bit per button, 8 buttons | button 16 | button 15 | button 14 | button 13 | button 12 | button 11 | button 10 | button 9 |
| 0x02             | JOYSTICK_X  | 8 bit, two's-comp     |           |           |           |           |           |           |           |          |
| 0x03             | JOYSTICK_Y  | 8 bit, two's-comp     |           |           |           |           |           |           |           |          |
| 0x04             | ROT_X       | 8 bit, two's-comp     |           |           |           |           |           |           |           |          |
| 0x05             | ROT_Y       | 8 bit, two's-comp     |           |           |           |           |           |           |           |          |
| 0x06             | ROT_Z       | 8 bit, two's-comp     |           |           |           |           |           |           |           |          |
| 0x07             | DPAD        | 1 bit per button, 4 buttons | X         | X         | X         | X         | right        | left      | down      | up    |
| 0xFF             | SEND_REPORT |                             | X         | X         | X         | X         | X         | X         | X         | X        |

X = don't care

SEND_REPORT is a special register which, when written to, indicates to the device that it should send new the updated data to the PC as soon as possible. Due to the way that USB works, this update does not happen immediately. The device is configured to send updates every 2ms (500Hz), but this may be much longer depending on the USB host controller and operating system that the device is connected to. 

It is safe to continue to perform SPI transactions whether or not the device has sent the updated data to the PC; if transactions occur on the SPI interface while an update is pending, they will either be ignored or merged with the pending data (depending on the exact timing).


### Connection drawing

![connections](https://i.imgur.com/FbCuaXz.png)
