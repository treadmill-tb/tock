// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2024.

//! Tock kernel for the Nordic Semiconductor nRF52840 development kit (DK)
//! with minimal USB functionality enabled.

#![no_std]
#![no_main]
#![deny(missing_docs)]

use core::ptr::addr_of;

use capsules_core::virtualizers::virtual_alarm::VirtualMuxAlarm;
use capsules_extra::usb::usb_user::UsbSyscallDriver;
use kernel::component::Component;
use kernel::platform::{KernelResources, SyscallDriverLookup};
use kernel::scheduler::round_robin::RoundRobinSched;
use kernel::{capabilities, create_capability, debug, static_init};
use nrf52840::interrupt_service::Nrf52840DefaultPeripherals;

/// Debug Writer
pub mod io;

// State for loading and holding applications.
// Number of concurrent processes this platform supports.
const NUM_PROCS: usize = 8;

static mut PROCESSES: [Option<&'static dyn kernel::process::Process>; NUM_PROCS] =
    [None; NUM_PROCS];

static mut CHIP: Option<&'static nrf52840::chip::NRF52<Nrf52840DefaultPeripherals>> = None;
static mut PROCESS_PRINTER: Option<&'static capsules_system::process_printer::ProcessPrinterText> =
    None;
static mut CDC_REF_FOR_PANIC: Option<
    &'static capsules_extra::usb::cdc::CdcAcm<
        'static,
        nrf52::usbd::Usbd<'static>,
        VirtualMuxAlarm<'static, nrf52::rtc::Rtc<'static>>,
    >,
> = None;
static mut NRF52_POWER: Option<&'static nrf52840::power::Power> = None;

/// Dummy buffer that causes the linker to reserve enough space for the stack.
#[no_mangle]
#[link_section = ".stack_buffer"]
pub static mut STACK_MEMORY: [u8; 0x1000] = [0; 0x1000];

// Function for the CDC/USB stack to use to reset
fn usb_reset_bootloader_enter() {
    unsafe {
        NRF52_POWER.unwrap().set_gpregret(0x42);
        cortexm4::scb::reset();
    }
}

//------------------------------------------------------------------------------
// SYSCALL DRIVER TYPE DEFINITIONS
//------------------------------------------------------------------------------

type AlarmDriver = components::alarm::AlarmDriverComponentType<nrf52840::rtc::Rtc<'static>>;

/// Supported drivers by the platform
pub struct Platform {
    console: &'static capsules_core::console::Console<'static>,
    // The USB CDC-ACM driver - used for panic output through the USB CDC-ACM interface
    #[allow(dead_code)]
    usb_cdc: &'static capsules_extra::usb::cdc::CdcAcm<
        'static,
        nrf52::usbd::Usbd<'static>,
        VirtualMuxAlarm<'static, nrf52::rtc::Rtc<'static>>,
    >,
    // The USB syscall driver that provides userspace access
    usb_driver: &'static UsbSyscallDriver<'static, capsules_extra::usb::cdc::CdcAcm<
        'static,
        nrf52::usbd::Usbd<'static>,
        VirtualMuxAlarm<'static, nrf52::rtc::Rtc<'static>>
    >>,
    alarm: &'static AlarmDriver,
    scheduler: &'static RoundRobinSched<'static>,
    systick: cortexm4::systick::SysTick,
    ipc: kernel::ipc::IPC<{ NUM_PROCS as u8 }>,
}

impl SyscallDriverLookup for Platform {
    fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
    where
        F: FnOnce(Option<&dyn kernel::syscall::SyscallDriver>) -> R,
    {
        match driver_num {
            capsules_core::console::DRIVER_NUM => f(Some(self.console)),
            capsules_core::alarm::DRIVER_NUM => f(Some(self.alarm)),
            capsules_extra::usb::usb_user::DRIVER_NUM => f(Some(self.usb_driver)),
            kernel::ipc::DRIVER_NUM => f(Some(&self.ipc)),
            _ => f(None),
        }
    }
}

impl KernelResources<nrf52840::chip::NRF52<'static, Nrf52840DefaultPeripherals<'static>>>
    for Platform
{
    type SyscallDriverLookup = Self;
    type SyscallFilter = ();
    type ProcessFault = ();
    type Scheduler = RoundRobinSched<'static>;
    type SchedulerTimer = cortexm4::systick::SysTick;
    type WatchDog = ();
    type ContextSwitchCallback = ();

    fn syscall_driver_lookup(&self) -> &Self::SyscallDriverLookup {
        self
    }
    fn syscall_filter(&self) -> &Self::SyscallFilter {
        &()
    }
    fn process_fault(&self) -> &Self::ProcessFault {
        &()
    }
    fn scheduler(&self) -> &Self::Scheduler {
        self.scheduler
    }
    fn scheduler_timer(&self) -> &Self::SchedulerTimer {
        &self.systick
    }
    fn watchdog(&self) -> &Self::WatchDog {
        &()
    }
    fn context_switch_callback(&self) -> &Self::ContextSwitchCallback {
        &()
    }
}

/// Main function called after RAM initialized.
#[no_mangle]
pub unsafe fn main() {
    //--------------------------------------------------------------------------
    // INITIAL SETUP
    //--------------------------------------------------------------------------

    // Apply errata fixes and enable interrupts.
    nrf52840::init();

    // Create the Nrf52840DefaultPeripherals and initialize
    let ieee802154_ack_buf = static_init!(
        [u8; nrf52840::ieee802154_radio::ACK_BUF_SIZE],
        [0; nrf52840::ieee802154_radio::ACK_BUF_SIZE]
    );
    // Initialize chip peripheral drivers
    let nrf52840_peripherals = static_init!(
        Nrf52840DefaultPeripherals,
        Nrf52840DefaultPeripherals::new(ieee802154_ack_buf)
    );

    nrf52840_peripherals.init();
    let base_peripherals = &nrf52840_peripherals.nrf52;

    //--------------------------------------------------------------------------
    // POWER and BOOTLOADER
    //--------------------------------------------------------------------------

    // Keep reference to power module for bootloader entry
    let power = &nrf52840_peripherals.nrf52.pwr_clk;
    NRF52_POWER = Some(power);

    //--------------------------------------------------------------------------
    // BOARD INITIALIZATION
    //--------------------------------------------------------------------------

    let board_kernel = static_init!(kernel::Kernel, kernel::Kernel::new(&*addr_of!(PROCESSES)));

    // Create an nRF52 chip for memory protection and timers.
    let chip = static_init!(
        nrf52840::chip::NRF52<Nrf52840DefaultPeripherals>,
        nrf52840::chip::NRF52::new(nrf52840_peripherals)
    );
    CHIP = Some(chip);

    // Do nRF52 specific startup
    nrf52_components::startup::NrfStartupComponent::new(
        false,
        nrf52840::gpio::Pin::P0_18, // BUTTON_RST_PIN
        nrf52840::uicr::Regulator0Output::DEFAULT,
        &base_peripherals.nvmc,
    )
    .finalize(());

    //--------------------------------------------------------------------------
    // CAPABILITIES
    //--------------------------------------------------------------------------

    // Create capabilities that the board needs to call certain protected kernel
    // functions.
    let main_loop_capability = create_capability!(capabilities::MainLoopCapability);
    let memory_allocation_capability = create_capability!(capabilities::MemoryAllocationCapability);

    //--------------------------------------------------------------------------
    // TIMER & ALARM SETUP
    //--------------------------------------------------------------------------

    let rtc = &base_peripherals.rtc;
    let _ = rtc.start();
    let mux_alarm = components::alarm::AlarmMuxComponent::new(rtc)
        .finalize(components::alarm_mux_component_static!(nrf52840::rtc::Rtc));
    let alarm = components::alarm::AlarmDriverComponent::new(
        board_kernel,
        capsules_core::alarm::DRIVER_NUM,
        mux_alarm,
    )
    .finalize(components::alarm_component_static!(nrf52840::rtc::Rtc));

    //--------------------------------------------------------------------------
    // UART & CONSOLE & DEBUG
    //--------------------------------------------------------------------------

    // Configure UART pins
    let uart_channel = nrf52_components::UartChannel::Pins(nrf52_components::UartPins::new(
        Some(nrf52840::gpio::Pin::P0_05), // RTS
        nrf52840::gpio::Pin::P0_06,       // TXD
        Some(nrf52840::gpio::Pin::P0_07), // CTS
        nrf52840::gpio::Pin::P0_08,       // RXD
    ));

    // Process printer for debugging
    let process_printer = components::process_printer::ProcessPrinterTextComponent::new()
        .finalize(components::process_printer_text_component_static!());
    PROCESS_PRINTER = Some(process_printer);

    // Setup UART channel
    let uart_channel = nrf52_components::UartChannelComponent::new(
        uart_channel,
        mux_alarm,
        &base_peripherals.uarte0,
    )
    .finalize(nrf52_components::uart_channel_component_static!(
        nrf52840::rtc::Rtc
    ));

    // Virtualize the UART channel
    let uart_mux = components::console::UartMuxComponent::new(uart_channel, 115200)
        .finalize(components::uart_mux_component_static!());

    // Create the console
    let console = components::console::ConsoleComponent::new(
        board_kernel,
        capsules_core::console::DRIVER_NUM,
        uart_mux,
    )
    .finalize(components::console_component_static!());

    // Create the process console for debugging
    let _ = components::process_console::ProcessConsoleComponent::new(
        board_kernel,
        uart_mux,
        mux_alarm,
        process_printer,
        Some(cortexm4::support::reset),
    )
    .finalize(components::process_console_component_static!(
        nrf52840::rtc::Rtc<'static>
    ));

    // Setup debug writer
    components::debug_writer::DebugWriterComponent::new(uart_mux)
        .finalize(components::debug_writer_component_static!());

    //--------------------------------------------------------------------------
    // USB CDC-ACM SETUP - THE IMPORTANT PART
    //--------------------------------------------------------------------------

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
            "Tock",               // Manufacturer
            "nRF52840-DK USB CDC", // Product
            "serial0001",         // Serial number
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

    // Store reference for panic handler
    CDC_REF_FOR_PANIC = Some(usb_cdc);

    // Create USB syscall driver for userspace access
    let usb_driver = static_init!(
        UsbSyscallDriver<'static, capsules_extra::usb::cdc::CdcAcm<
            'static,
            nrf52::usbd::Usbd<'static>,
            VirtualMuxAlarm<'static, nrf52::rtc::Rtc<'static>>
        >>,
        UsbSyscallDriver::new(
            usb_cdc, 
            board_kernel.create_grant(
                capsules_extra::usb::usb_user::DRIVER_NUM, 
                &memory_allocation_capability
            )
        )
    );

    // Enable USB CDC-ACM driver
    usb_cdc.enable();
    usb_cdc.attach();

    //--------------------------------------------------------------------------
    // NRF CLOCK SETUP
    //--------------------------------------------------------------------------

    nrf52_components::NrfClockComponent::new(&base_peripherals.clock).finalize(());

    //--------------------------------------------------------------------------
    // PLATFORM SETUP, SCHEDULER, AND START KERNEL LOOP
    //--------------------------------------------------------------------------

    let scheduler = components::sched::round_robin::RoundRobinComponent::new(&*addr_of!(PROCESSES))
        .finalize(components::round_robin_component_static!(NUM_PROCS));

    // Create the platform object - minimal with just USB, console, and alarm
    let platform = static_init!(
        Platform,
        Platform {
            console,
            usb_cdc,
            usb_driver,
            alarm,
            scheduler,
            systick: cortexm4::systick::SysTick::new_with_calibration(64000000),
            ipc: kernel::ipc::IPC::new(
                board_kernel,
                kernel::ipc::DRIVER_NUM,
                &memory_allocation_capability,
            ),
        }
    );

    debug!("Initialization complete. Entering main loop");

    // Start the main kernel loop
    board_kernel.kernel_loop(
        platform,
        chip,
        None::<&kernel::ipc::IPC<0>>,
        &main_loop_capability,
    );
}