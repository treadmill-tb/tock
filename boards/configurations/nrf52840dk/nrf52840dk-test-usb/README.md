# nRF52840DK USB Configuration

This configuration board enables the USB driver on the nRF52840DK. It sets up the board with USB CDC (Serial over USB) functionality enabled.

## Building and Flashing

To build and flash this configuration:

```bash
cd boards/configurations/nrf52840dk/nrf52840dk-test-usb
make
make flash
```

## USB Support

This configuration enables the USB CDC (Communication Device Class) ACM (Abstract Control Model) driver, which provides a virtual serial port over USB. When the board is connected to a computer, it will enumerate as a USB CDC device and create a virtual serial port.

## Usage

Once flashed, you can connect to the nRF52840DK via USB. The board will enumerate as a USB serial device that you can connect to using a terminal program:

On Linux/macOS:
```bash
screen /dev/ttyACM0 115200
```

On Windows:
```
Use Device Manager to find the COM port assigned to the device, then connect using PuTTY or another terminal program.
```

## Creating Applications

Applications can communicate over USB using standard console syscalls like `printf`.

## Additional USB Functionality

The USB implementation can be modified in the `main.rs` file to support other USB device classes like HID (Human Interface Device) for keyboard, mouse, or other peripherals.