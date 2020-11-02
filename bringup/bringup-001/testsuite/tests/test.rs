#![no_std]
#![no_main]

use cortex_m_rt::entry;
use bringup_001 as _; // memory layout + panic handler

#[entry]
fn main() -> ! {
    assert!(false, "TODO: Write actual tests");

    bringup_001::exit();
}
