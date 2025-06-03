# nRF52840DK USB Bulk Configuration

This configuration enables USB bulk endpoint support on the nRF52840DK board.
It includes the `usbc_client` capsule which exposes USB functionality to userspace applications.

## Features

- USB bulk endpoints (IN: 0x81, OUT: 0x02)
- USB VID:PID 0x6667:0xabcd (for compatibility with usb_bulk_echo app)
- CDC-ACM for console output (secondary)
- Full userspace USB access via syscalls

## Usage

This configuration is designed to work with the `usb_bulk_echo` test application
in libtock-c which implements a USB bulk echo functionality.

```bash
make
tockloader flash target/thumbv7em-none-eabi/release/nrf52840dk-usb-bulk.bin
```