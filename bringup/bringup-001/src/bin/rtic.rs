#![no_main]
#![no_std]

use bringup_001 as _; // global logger + panicking-behavior + memory layout
use stm32f4xx_hal::{
    stm32::{SPI5, USART1},
    prelude::*,
    spi::{Spi, NoSck, NoMiso},
    serial::{Serial, config::Config, Tx, Rx},
    gpio::gpiob::PB8,
    gpio::{Alternate, AF6},
};
use ws2812_spi::{Ws2812, MODE};
use cortex_m::asm::delay;
use smart_leds::{SmartLedsWrite, colors, gamma, brightness};

type SmartLed = Ws2812<Spi<SPI5, (NoSck, NoMiso, PB8<Alternate<AF6>>)>>;

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        smartled: SmartLed,
        rs485_tx: Tx<USART1>,
        rs485_rx: Rx<USART1>,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        defmt::info!("Hello, world!");

        let board = ctx.device;

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
        let led = Ws2812::new(spi);

        let config = Config::default()
            .baudrate(12_000_000.bps());

        let (tx, rx) = Serial::usart1(
            board.USART1,
            (gpioa.pa15.into_alternate_af7(), gpioa.pa10.into_alternate_af7()),
            config,
            clocks
        ).unwrap().split();

        defmt::info!("Clocks good!");

        init::LateResources {
            smartled: led,
            rs485_tx: tx,
            rs485_rx: rx,
        }
    }

    #[idle(resources = [smartled])]
    fn idle(ctx: idle::Context) -> ! {
        let led = ctx.resources.smartled;
        let data = [
            colors::RED,
            colors::ORANGE,
            colors::YELLOW,
            colors::GREEN,
            colors::BLUE,
            colors::INDIGO,
            colors::VIOLET,
            colors::BLACK,
        ];

        let mut iter = data.iter().cloned().cycle();

        loop {

            led.write(
                brightness(
                    gamma(
                        iter.clone().take(1)
                    ), 24
                )
            ).unwrap();
            delay(96_000_000 / 3);

            let _ = iter.next();
        }
    }
};
