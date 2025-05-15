# nRF52840DK USB Configuration

This board configuration is designed to enable USB functionality on the Nordic Semiconductor nRF52840 development kit.

## Features

- Enables the USB CDC-ACM interface for serial communication over USB
- Demonstrates USB driver implementation in Tock
- Uses the nRF52840's built-in USB peripheral
- Provides a virtual serial port for communication between the board and host computer

## Usage

This configuration can be used for:

1. Testing USB functionality in the Tock kernel
2. Developing user applications that interact with USB devices
3. Serial communication over USB instead of UART

## Building and Flashing

To build the kernel:

```bash
cd boards/configurations/nrf52840dk/nrf52840dk-test-usb
make
```

To flash the kernel to your device:

```bash
cd boards/configurations/nrf52840dk/nrf52840dk-test-usb
make flash
```

## USB Driver Details

This configuration uses the built-in USB peripheral of the nRF52840 SoC and implements CDC-ACM (Communication Device Class - Abstract Control Model) to create a virtual serial port. When connected to a computer, it will appear as a standard serial device.

The default USB descriptors identify the device as:
- Manufacturer: "Tock"  
- Product: "nRF52840-DK USB CDC"
- Serial: "serial0001"