# nRF52840DK USB Configuration

This board configuration is designed to enable USB functionality on the Nordic Semiconductor nRF52840 development kit with minimal dependencies.

## Features

- Enables the USB CDC-ACM interface for serial communication over USB
- Demonstrates USB driver implementation in Tock
- Uses the nRF52840's built-in USB peripheral
- Provides a virtual serial port for communication between the board and host computer
- Exposes the USB functionality to userspace applications via the USB syscall driver

## Implementation Details

The key components of the USB implementation are:

1. **USB CDC-ACM Driver Setup**
   ```rust
   // Create a virtual timer for USB CDC-ACM
   let virtual_alarm_cdc = static_init!(
       VirtualMuxAlarm<'static, nrf52::rtc::Rtc>,
       VirtualMuxAlarm::new(mux_alarm)
   );
   virtual_alarm_cdc.setup();

   // Define USB string descriptors
   let strings = static_init!(
       [&str; 3],
       [
           "Tock",                // Manufacturer
           "nRF52840-DK USB CDC", // Product
           "serial0001",          // Serial number
       ]
   );

   // Setup the USB CDC-ACM driver
   let usb_cdc = components::cdc::CdcAcmComponent::new(
       &nrf52840_peripherals.usbd,
       capsules_extra::usb::usbc_client::MAX_CTRL_PACKET_SIZE_NRF52840,
       0x1915, // Nordic Vendor ID
       0x1001, // Product ID (arbitrary)
       strings,
       mux_alarm,
       Some(&usb_reset_bootloader_enter),
   )
   .finalize(components::cdc_acm_component_static!(
       nrf52::usbd::Usbd,
       nrf52::rtc::Rtc
   ));

   // Enable USB CDC-ACM driver
   usb_cdc.enable();
   usb_cdc.attach();
   ```

2. **USB Syscall Driver for Userspace Access**
   ```rust
   // Create USB syscall driver for userspace access
   let usb_driver = static_init!(
       UsbSyscallDriver<'static, capsules_extra::usb::cdc::CdcAcm<...>>,
       UsbSyscallDriver::new(
           usb_cdc, 
           board_kernel.create_grant(
               capsules_extra::usb::usb_user::DRIVER_NUM, 
               &memory_allocation_capability
           )
       )
   );
   ```

3. **Platform Integration**
   ```rust
   // Add to platform struct
   pub struct Platform {
       // ...
       usb_cdc: &'static capsules_extra::usb::cdc::CdcAcm<...>,
       usb_driver: &'static UsbSyscallDriver<'static, capsules_extra::usb::cdc::CdcAcm<...>>,
       // ...
   }

   // Add to syscall driver lookup
   impl SyscallDriverLookup for Platform {
       fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
       where
           F: FnOnce(Option<&dyn kernel::syscall::SyscallDriver>) -> R,
       {
           match driver_num {
               // ...
               capsules_extra::usb::usb_user::DRIVER_NUM => f(Some(self.usb_driver)),
               // ...
           }
       }
   }
   ```

## Usage

This configuration can be used for:

1. Testing USB functionality in the Tock kernel
2. Developing user applications that interact with USB devices
3. Serial communication over USB instead of UART

## USB Driver Details

This configuration uses the built-in USB peripheral of the nRF52840 SoC and implements CDC-ACM (Communication Device Class - Abstract Control Model) to create a virtual serial port. When connected to a computer, it will appear as a standard serial device.

The default USB descriptors identify the device as:
- Manufacturer: "Tock"  
- Product: "nRF52840-DK USB CDC"
- Serial: "serial0001"

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

## Using the USB Driver in Userspace Applications

Applications can use the USB driver by making syscalls to driver number `0x20` (USB driver), which is defined as `capsules_extra::usb::usb_user::DRIVER_NUM`. This allows userspace applications to send and receive data over the USB interface.