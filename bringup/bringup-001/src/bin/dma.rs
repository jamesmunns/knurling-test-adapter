#![no_main]
#![no_std]

use bringup_001 as _; // global logger + panicking-behavior + memory layout
use stm32f4xx_hal::{
    stm32::{self, SPI5, USART1},
    prelude::*,
    spi::{Spi, NoSck, NoMiso},
    serial::{Serial, config::{Config, DmaConfig as SerialDmaConfig}, Tx, Rx},
    gpio::gpiob::PB8,
    gpio::{Alternate, AF6},
    otg_fs::{USB, UsbBus},
    dma,
};
use ws2812_spi::{Ws2812, MODE};
use cortex_m::asm::delay;
use smart_leds::{RGB8, SmartLedsWrite, colors, gamma, brightness};
use usb_device::{prelude::*, bus::UsbBusAllocator};
use usbd_serial::SerialPort;
use core::sync::atomic::{AtomicU32, Ordering};
use bbqueue::{ConstBBBuffer, BBBuffer, consts::*, framed::{FrameConsumer, FrameProducer, FrameGrantW}};

static RS485_RX: BBBuffer<U4096> = BBBuffer ( ConstBBBuffer::new() );
static COLOR_CMD: AtomicU32 = AtomicU32::new(0);
type SmartLed = Ws2812<Spi<SPI5, (NoSck, NoMiso, PB8<Alternate<AF6>>)>>;

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        smartled: SmartLed,
        rs485_tx: Tx<USART1>,
        rs485_rx: Rx<USART1>,
        rs485_prod: FrameProducer<'static, U4096>,
        rs485_cons: FrameConsumer<'static, U4096>,
        usb_serial: SerialPort<'static, UsbBus<USB>>,
        usb_dev: UsbDevice<'static, UsbBus<USB>>,
        rx_xfr: Option<
            dma::Transfer<
                dma::Stream5<stm32::DMA2>,
                dma::Channel4,
                USART1,
                dma::PeripheralToMemory,
                FrameGrantW<'static, U4096>
            >
        >,
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

        let mut config = Config::default()
            .baudrate(12_000_000.bps());
        config.dma = SerialDmaConfig::Rx;

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

        let serial = usbd_serial::SerialPort::new(usb_bus);

        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();

        let (mut prod, cons) = RS485_RX.try_split_framed().unwrap();
        let wgr = prod.grant(128).unwrap();

        let dma_cfg = dma::config::DmaConfig::default()
            .transfer_complete_interrupt(true)
            .memory_increment(true);

        let streams = dma::StreamsTuple::new(board.DMA2);

        let mut txfr = dma::Transfer::init(
            streams.5,
            unsafe { stm32::Peripherals::steal().USART1 },
            wgr,
            None,
            dma_cfg
        );

        txfr.start(|usart| {
            usart.cr1.modify(|_r, w| {
                w.re().set_bit()
            })
        });

        init::LateResources {
            smartled: led,
            rs485_tx: tx,
            rs485_rx: rx,
            usb_serial: serial,
            usb_dev,
            rs485_prod: prod,
            rs485_cons: cons,
            rx_xfr: Some(txfr),
        }
    }

    #[task(binds = DMA2_STREAM5, resources = [rs485_prod, rx_xfr])]
    fn rs485_rx(ctx: rs485_rx::Context) {
        defmt::info!("ding!");
        let prod = ctx.resources.rs485_prod;
        let mut xfr = ctx.resources.rx_xfr.take().unwrap();

        xfr.clear_interrupts();

        let (stream, periph, buf, _dblbuf) = xfr.free();
        buf.commit(128);

        let new_grant = prod.grant(128).unwrap();

        let dma_cfg = dma::config::DmaConfig::default()
            .transfer_complete_interrupt(true)
            .memory_increment(true);

        let mut txfr = dma::Transfer::init(
            stream,
            periph,
            new_grant,
            None,
            dma_cfg
        );

        txfr.start(|usart| {
            usart.cr1.modify(|_r, w| {
                w.re().set_bit()
            })
        });

        *ctx.resources.rx_xfr = Some(txfr);

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

    #[idle(resources = [smartled, rs485_tx, rs485_cons])]
    fn idle(ctx: idle::Context) -> ! {
        let led = ctx.resources.smartled;
        let tx = ctx.resources.rs485_tx;
        let rx = ctx.resources.rs485_cons;

        let mut color = colors::WHITE;

        loop {
            for _ in 0..128 {
                'inner: loop {
                    match tx.write(42) {
                        Ok(_) => break 'inner,
                        Err(nb::Error::WouldBlock) => {
                            delay(96_000_000 / 500_000);
                            continue 'inner
                        },
                        Err(_) => panic!(),
                    }
                }
            }

            // led.write(
            //     [color].iter().cloned()
            // ).unwrap();
            // delay(96_000_000 / 1024);

            while let Some(incoming) = rx.read() {
                defmt::info!("Got {:?} bytes", incoming.len());
                let mut good = 0;
                let mut bad = 0;
                for b in incoming.iter() {
                    if *b == 42 {
                        good += 1;
                    } else {
                        bad += 1;
                    }
                }

                defmt::info!("Good: {:?}, Bad: {:?}", good, bad);

                incoming.release();
            }

            // led.write(
            //     [colors::BLACK].iter().cloned()
            // ).unwrap();
            // delay(96_000_000 / 2);

            // let color_cmd = COLOR_CMD.load(Ordering::Acquire);
            // if let Some(col) = u32_to_color(color_cmd) {
            //     color = col;
            // }
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
