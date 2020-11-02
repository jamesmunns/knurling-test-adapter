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
    otg_fs::{USB, UsbBus},
};
use ws2812_spi::{Ws2812, MODE};
use cortex_m::asm::delay;
use smart_leds::{RGB8, SmartLedsWrite, colors, gamma, brightness};
use usb_device::{prelude::*, bus::UsbBusAllocator};
use usbd_serial::SerialPort;
use core::sync::atomic::{AtomicU32, Ordering};


static COLOR_CMD: AtomicU32 = AtomicU32::new(0);
type SmartLed = Ws2812<Spi<SPI5, (NoSck, NoMiso, PB8<Alternate<AF6>>)>>;

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        smartled: SmartLed,
        rs485_tx: Tx<USART1>,
        rs485_rx: Rx<USART1>,
        usb_serial: SerialPort<'static, UsbBus<USB>>,
        usb_dev: UsbDevice<'static, UsbBus<USB>>,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        static mut USB_BUS: Option<UsbBusAllocator<UsbBus<USB>>> = None;

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

        let usb = USB {
            usb_global: board.OTG_FS_GLOBAL,
            usb_device: board.OTG_FS_DEVICE,
            usb_pwrclk: board.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into_alternate_af10(),
            pin_dp: gpioa.pa12.into_alternate_af10(),
            hclk: clocks.hclk(),
        };

        *USB_BUS = Some(UsbBus::new(usb, EP_MEMORY));
        let usb_bus = USB_BUS.as_ref().unwrap();

        let mut serial = usbd_serial::SerialPort::new(usb_bus);

        let mut usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();

        init::LateResources {
            smartled: led,
            rs485_tx: tx,
            rs485_rx: rx,
            usb_serial: serial,
            usb_dev,
        }
    }

    #[task(binds = OTG_FS, resources = [usb_serial, usb_dev])]
    fn usb(ctx: usb::Context) {
        let usb = &mut *ctx.resources.usb_dev;
        let serial = &mut *ctx.resources.usb_serial;
        let mut buf = [0u8; 128];

        if usb.poll(&mut [serial]) {
            if let Ok(count) = serial.read(&mut buf) {
                let window = &buf[..count];
                serial.write(window).unwrap();

                for i in window {
                    let color = match i {
                        b'r' => colors::RED,
                        b'o' => colors::ORANGE,
                        b'y' => colors::YELLOW,
                        b'g' => colors::GREEN,
                        b'b' => colors::BLUE,
                        b'i' => colors::INDIGO,
                        b'v' => colors::VIOLET,
                        b'k' => colors::BLACK,
                        b'w' => colors::WHITE,
                        _ => continue,
                    };

                    COLOR_CMD.store(color_to_u32(color), Ordering::Release);
                }
            }
        }
    }

    #[idle(resources = [smartled])]
    fn idle(ctx: idle::Context) -> ! {
        let led = ctx.resources.smartled;

        let mut color = colors::WHITE;

        loop {
            led.write(
                [color].iter().cloned()
            ).unwrap();
            delay(96_000_000 / 2);

            led.write(
                [colors::BLACK].iter().cloned()
            ).unwrap();
            delay(96_000_000 / 2);

            let color_cmd = COLOR_CMD.load(Ordering::Acquire);
            if let Some(col) = u32_to_color(color_cmd) {
                color = col;
            }
        }
    }
};

fn color_to_u32(rgb: RGB8) -> u32 {
    let mut out = [0u8; 4];

    out[0] = rgb.r;
    out[1] = rgb.g;
    out[2] = rgb.b;
    out[3] = 0xFF;

    u32::from_le_bytes(out)
}

fn u32_to_color(input: u32) -> Option<RGB8> {
    let bytes = input.to_le_bytes();

    if bytes[3] == 0xFF {
        Some(RGB8 {
            r: bytes[0],
            g: bytes[1],
            b: bytes[2],
        })
    } else {
        None
    }
}
