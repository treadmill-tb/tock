// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2024.

use segger::rtt::{RttChannel, RttMemory};

// RTT memory for debug communication
static mut RTT: *mut RttMemory = core::ptr::null_mut();

/// Sets the RTT memory object.
pub unsafe fn set_rtt_memory(memory: &'static mut RttMemory) {
    RTT = memory;
}

/// Implements `Write` trait for a RTT memory object.
struct RttWriter<'a>(&'a mut RttMemory);

impl<'a> RttWriter<'a> {
    pub fn new(memory: &'a mut RttMemory) -> Self {
        Self(memory)
    }
}

impl<'a> core::fmt::Write for RttWriter<'a> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let up = &mut self.0.up_channels[0];
        up.put_str(s);
        Ok(())
    }
}

/// Write debug output to the console.
pub fn debug_write(args: core::fmt::Arguments) {
    use core::fmt::Write;
    let rtt_mem = unsafe { &mut *RTT };
    let mut writer = RttWriter::new(rtt_mem);
    let _ = writer.write_fmt(args);
}

/// Panic implementation.
pub fn panic_fmt(args: core::fmt::Arguments) {
    use core::fmt::Write;
    let rtt_mem = unsafe { &mut *RTT };
    let mut writer = RttWriter::new(rtt_mem);
    let _ = writer.write_str("Kernel panic: ");
    let _ = writer.write_fmt(args);
}