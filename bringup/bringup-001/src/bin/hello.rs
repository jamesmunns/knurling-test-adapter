#![no_main]
#![no_std]

use bringup_001 as _; // global logger + panicking-behavior + memory layout

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::info!("Hello, world!");

    bringup_001::exit()
}
