#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::Cell;

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::spi::{self, Phase, Polarity, Spi};
use embassy_rp::{bind_interrupts, pac, peripherals, usb};
use embassy_time::{Duration, Ticker};
use embassy_usb::class::hid;
use embassy_usb::driver::Driver;
use {defmt_rtt as _, panic_probe as _};

type Controller<'d> = hid::HidWriter<'d, embassy_rp::usb::Driver<'d, peripherals::USB>, 6>;
type ControllerData = Cell<[u8; 6]>;

const CONTROLLER_DATA_INIT: ControllerData =
    ControllerData::new([0x00, 0x00, 0x80, 0x80, 0x80, 0x80]);

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<peripherals::USB>;
});

// #[derive(Debug, Clone, Copy)]
// #[repr(packed)]
// struct ControllerData {
//     buttons: u16,
//     rx: u8,
//     ry: u8,
//     x: u8,
//     y: u8,
// }

fn build_hid_controller<'d, D: Driver<'d>>(
    builder: &mut embassy_usb::Builder<'d, D>,
    state: &'d mut hid::State<'d>,
) -> hid::HidWriter<'d, D, 6> {
    let config = hid::Config {
        #[rustfmt::skip]
        report_descriptor: &[
            0x05, 0x01,         // USAGE_PAGE (Generic Desktop)
            0x09, 0x05,         // USAGE (Game Pad)
            0xa1, 0x01,         // COLLECTION (Application)
            0x05, 0x09,         //   USAGE_PAGE (Button)
            0x09, 0x01,         //   USAGE (Button 1)
            0x09, 0x02,         //   USAGE (Button 2)
            0x09, 0x03,         //   USAGE (Button 3)
            0x09, 0x04,         //   USAGE (Button 4)
            0x09, 0x05,         //   USAGE (Button 5)
            0x09, 0x06,         //   USAGE (Button 6)
            0x09, 0x07,         //   USAGE (Button 7)
            0x09, 0x08,         //   USAGE (Button 8)
            0x09, 0x09,         //   USAGE (Button 9)
            0x09, 0x0a,         //   USAGE (Button 10)
            0x09, 0x0b,         //   USAGE (Button 11)
            0x09, 0x0c,         //   USAGE (Button 12)
            0x09, 0x0d,         //   USAGE (Button 13)
            0x09, 0x0e,         //   USAGE (Button 14)
            0x09, 0x0f,         //   USAGE (Button 15)
            0x09, 0x10,         //   USAGE (Button 16)
            0x15, 0x00,         //   LOGICAL_MINIMUM (0)
            0x25, 0x01,         //   LOGICAL_MAXIMUM (1)
            0x75, 0x01,         //   REPORT_SIZE (1)
            0x95, 0x10,         //   REPORT_COUNT (16)
            0x81, 0x02,         //   INPUT (Data,Var,Abs)
            0x05, 0x01,         //   USAGE_PAGE (Generic Desktop)
            0x09, 0x33,         //   USAGE (Rx)
            0x09, 0x34,         //   USAGE (Ry)
            0x09, 0x30,         //   USAGE (X)
            0x09, 0x31,         //   USAGE (Y)
            0x15, 0x00,         //   LOGICAL_MINIMUM (0)
            0x26, 0xff, 0x00,   //   LOGICAL_MAXIMUM (255)
            0x75, 0x08,         //   REPORT_SIZE (8)
            0x95, 0x04,         //   REPORT_COUNT (4)
            0x81, 0x02,         //   INPUT (Data,Var,Abs)
            0xc0,               // END_COLLECTION
        ],
        request_handler: None,
        poll_ms: 1,
        max_packet_size: 8,
    };
    hid::HidWriter::new(builder, state, config)
}

async fn run_controller(mut c: Controller<'_>, data: &ControllerData) {
    loop {
        let mut data = data.get();
        // Fixup SPI bitorder
        for d in &mut data {
            *d = d.reverse_bits();
        }
        // PSX has buttons as active low
        data[0] = !data[0];
        data[1] = !data[1];
        //debug!("data={=[u8]:X}", data);
        unwrap!(c.write(data.as_ref()).await);
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let driver = usb::Driver::new(p.USB, Irqs);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0x0000, 0x0000);
    config.device_release = 0x0000;
    config.manufacturer = Some("Timo Kröger");
    config.product = Some("psx-usb");
    config.max_power = 500;
    config.max_packet_size_0 = 64;

    // Required for Windows support.
    config.composite_with_iads = true;
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut device_descriptor = [0; 256];
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut s0 = hid::State::new();
    let mut s1 = hid::State::new();
    let mut s2 = hid::State::new();
    let mut s3 = hid::State::new();

    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        &mut device_descriptor,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut control_buf,
    );

    let c0 = build_hid_controller(&mut builder, &mut s0);
    let c1 = build_hid_controller(&mut builder, &mut s1);
    let c2 = build_hid_controller(&mut builder, &mut s2);
    let c3 = build_hid_controller(&mut builder, &mut s3);

    let mut usb = builder.build();

    let data = &[CONTROLLER_DATA_INIT; 4];

    // USB
    let fut = usb.run();

    // Controllers
    let fut = join(fut, async move { run_controller(c0, &data[0]).await });
    let fut = join(fut, async move { run_controller(c1, &data[1]).await });
    let fut = join(fut, async move { run_controller(c2, &data[2]).await });
    let fut = join(fut, async move { run_controller(c3, &data[3]).await });

    // SPI
    let fut = join(fut, async move {
        let mut led = Output::new(p.PIN_25, Level::Low);

        let mut config = spi::Config::default();
        config.frequency = 10000;
        config.phase = Phase::CaptureOnSecondTransition;
        config.polarity = Polarity::IdleHigh;
        let mut spi = Spi::new(
            p.SPI0, p.PIN_2, p.PIN_3, p.PIN_4, p.DMA_CH0, p.DMA_CH1, config,
        );
        // Hacky way to configure a pull up on MISO PIN4
        pac::PADS_BANK0.gpio(4).modify(|w| {
            w.set_pue(true);
            w.set_pde(false);
        });
        let mut cs = Output::new(p.PIN_5, Level::High);

        let cmd_part1 = [0x80, 0x42, 0x80];
        let cmd_part2 = [
            0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 1
            0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 2
            0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 3
            0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 4
        ];

        let mut ticker = Ticker::every(Duration::from_millis(100));
        loop {
            ticker.next().await;

            cs.set_low();
            let mut rsp_part1 = [0_u8; 3];
            unwrap!(spi.transfer(&mut rsp_part1, &cmd_part1).await);

            //debug!("rsp_part1 = {=[u8]:X}", rsp_part1);
            if rsp_part1[1] == 0x01 && rsp_part1[2] == 0x5A {
                let mut rsp_part2 = [0_u8; 32];
                unwrap!(spi.transfer(&mut rsp_part2, &cmd_part2).await);
                //debug!("rsp_part2 = {=[u8]:X}", rsp_part2);

                data[0].set(rsp_part2[2..8].try_into().unwrap());
                data[1].set(rsp_part2[10..16].try_into().unwrap());
                data[2].set(rsp_part2[18..24].try_into().unwrap());
                data[3].set(rsp_part2[26..32].try_into().unwrap());
                led.toggle();
            }
            cs.set_high();
        }
    });

    fut.await;
}

#[defmt::panic_handler]
fn defmt_panic() -> ! {
    cortex_m::asm::udf();
}
