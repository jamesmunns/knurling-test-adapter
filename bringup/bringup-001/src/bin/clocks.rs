#![no_main]
#![no_std]

use bringup_001 as _; // global logger + panicking-behavior + memory layout
use stm32f4xx_hal::{
    stm32::{self, DWT},
    prelude::*,
    spi::{Spi, NoSck, NoMiso},
    serial::{Serial, config::Config},
    timer::Timer,
};
use ws2812_spi::{Ws2812, MODE};
use cortex_m::asm::delay;
use smart_leds::{RGB8, SmartLedsWrite, colors};

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::info!("Hello, world!");

    let board = stm32::Peripherals::take().unwrap();
    let mut core = stm32::CorePeripherals::take().unwrap();
    core.DCB.enable_trace();
    DWT::unlock();
    core.DWT.enable_cycle_counter();

    let rcc = board.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(25.mhz())
        .hclk(96.mhz())
        .sysclk(96.mhz())
        .pclk1(48.mhz())
        .pclk2(96.mhz())
        .require_pll48clk()
        .freeze();

    let gpioa = board.GPIOA.split();
    let gpiob = board.GPIOB.split();

    let spi = Spi::spi5(
        board.SPI5,
        (NoSck, NoMiso, gpiob.pb8.into_alternate_af6()),
        MODE,
        3_000_000.hz(),
        clocks
    );
    let mut led = Ws2812::new(spi);

    let config = Config::default()
        .baudrate(12_000_000.bps());

    let mut timer = Timer::tim1(board.TIM1, 10_000_000.hz(), clocks);

    let (mut tx, mut rx) = Serial::usart1(
        board.USART1,
        (gpioa.pa15.into_alternate_af7(), gpioa.pa10.into_alternate_af7()),
        config,
        clocks
    ).unwrap().split();

    defmt::info!("Clocks good!");

    for _ in 0..1 {
        defmt::info!("Red!");
        let data: [RGB8; 1] = [colors::RED];
        led.write(data.iter().cloned()).unwrap();
        delay(96_000_000 / 3);

        defmt::info!("Green!");
        let data: [RGB8; 1] = [colors::GREEN];
        led.write(data.iter().cloned()).unwrap();
        delay(96_000_000 / 3);

        defmt::info!("Blue!");
        let data: [RGB8; 1] = [colors::BLUE];
        led.write(data.iter().cloned()).unwrap();
        delay(96_000_000 / 3);
    }

    defmt::info!("Black!");
    let data: [RGB8; 1] = [colors::BLACK];
    led.write(data.iter().cloned()).unwrap();
    delay(96_000_000 / 3);

    defmt::info!("UART check!");

    let start = DWT::get_cycle_count();
    for _ in 0..1000 {
        for ch in b"hello" {
            loop {
                match tx.write(*ch) {
                    Ok(_) => break,
                    Err(nb::Error::WouldBlock) => continue,
                    Err(_) => panic!(),
                }
            }
            // delay(10_000_000);
            loop {
                match rx.read() {
                    Ok(rxb) => {
                        assert_eq!(*ch, rxb);
                        break;
                    }
                    Err(nb::Error::WouldBlock) => continue,
                    Err(_) => panic!(),
                }
            }
        }
    }
    let end = DWT::get_cycle_count();

    let elapsed = end.wrapping_sub(start);

    defmt::info!("UART check good!");
    defmt::info!("Took {:u32} cycles", elapsed);

    bringup_001::exit()
}
