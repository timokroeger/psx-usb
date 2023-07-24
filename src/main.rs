#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::Cell;
use core::pin::pin;

use defmt::*;
use embassy_futures::join::join;
use embassy_rp::gpio::{Level, Output, Pull};
use embassy_rp::pio::{self, Direction, IrqFlags, Pio, ShiftConfig, ShiftDirection, StateMachine};
use embassy_rp::relocate::RelocatedProgram;
use embassy_rp::{bind_interrupts, peripherals, usb, Peripheral};
use embassy_time::{with_timeout, Duration, Ticker};
use embassy_usb::class::hid;
use embassy_usb::driver::{Driver, EndpointIn};
use fixed::traits::ToFixed;
use fixed_macro::types::U56F8;
use {defmt_rtt as _, panic_probe as _};

type Controller<'d> = hid::HidWriter<'d, embassy_rp::usb::Driver<'d, peripherals::USB>, 6>;
type ControllerData = Cell<[u8; 6]>;

const CONTROLLER_DATA_INIT: ControllerData =
    ControllerData::new([0x00, 0x00, 0x80, 0x80, 0x80, 0x80]);

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<peripherals::USB>;
    PIO0_IRQ_0 => pio::InterruptHandler<peripherals::PIO0>;
});

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

async fn run_controller(mut c: Controller<'_>, data: &ControllerData) -> ! {
    loop {
        let mut data = data.get();
        // PSX has buttons as active low
        data[0] = !data[0];
        data[1] = !data[1];
        //debug!("data={=[u8]:X}", data);
        unwrap!(c.write(data.as_ref()).await);
    }
}

async fn poll_psx(
    sm0: &mut StateMachine<'_, peripherals::PIO0, 0>,
    irq_flags: &IrqFlags<'_, peripherals::PIO0>,
    rx_dma: &mut peripherals::DMA_CH0,
    tx_dma: &mut peripherals::DMA_CH1,
) -> Option<[u8; 32]> {
    sm0.clear_fifos();
    irq_flags.set(0); // ACK flag
    sm0.set_enable(true);

    let (rx, tx) = sm0.rx_tx();

    let command = [
        0x01_u8, // controller (not memory card)
        0x42,    // poll
        0x01,    // multitap
        0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 1
        0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 2
        0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 3
        0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 4
    ];
    let tx_fut = tx.dma_push(tx_dma.into_ref(), &command);

    let mut response = [0x0_u8; 35];
    let rx_fut = rx.dma_pull(rx_dma.into_ref(), &mut response);

    let _ = with_timeout(Duration::from_micros(1800), join(tx_fut, rx_fut)).await;

    sm0.set_enable(false);

    //debug!("{=[u8]:X}", response);
    if response[1] == 0x80 && response[2] == 0x5A {
        Some(unwrap!(response[3..].try_into()))
    } else {
        None
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut p = embassy_rp::init(Default::default());

    // Configure PIO state machine for the custom PSX SPI protocol
    let Pio {
        mut common,
        irq_flags,
        mut sm0,
        mut sm1,
        ..
    } = Pio::new(p.PIO0, Irqs);

    // Setup up the pins for PIO.
    let sck_clk = common.make_pio_pin(p.PIN_2);
    let txd_mosi = common.make_pio_pin(p.PIN_3);
    let mut rxd_miso = common.make_pio_pin(p.PIN_4);
    rxd_miso.set_pull(Pull::Up);
    let mut dsr_ack = common.make_pio_pin(p.PIN_6);
    dsr_ack.set_pull(Pull::Up);

    // SM0 acts as a master SPI peripheral which waits for IRQ0 to be set before
    // clocking out the next byte.
    let psx_spi = pio_proc::pio_file!("src/psx.pio", select_program("spi"));
    let psx_spi = RelocatedProgram::new(&psx_spi.program);
    let psx_spi_len = psx_spi.code().count();
    let mut config = pio::Config::default();
    config.use_program(&common.load_program(&psx_spi), &[&sck_clk]);
    config.set_in_pins(&[&rxd_miso]);
    config.set_out_pins(&[&txd_mosi]);
    // 1MHz PIO clock gives SPI frequency of 250kHz
    config.clock_divider = (U56F8!(125_000_000) / U56F8!(1_000_000)).to_fixed();
    // LSB first
    config.shift_in = ShiftConfig {
        threshold: 32,
        direction: ShiftDirection::Right,
        auto_fill: true,
    };
    config.shift_out = ShiftConfig {
        threshold: 8,
        direction: ShiftDirection::Right,
        auto_fill: false,
    };
    sm0.set_config(&config);
    sm0.set_pin_dirs(Direction::Out, &[&sck_clk, &txd_mosi]);
    // The pio_file() proc macro fails to compile the instruction
    // `mov x, !null` to set the register X to 0xFFFFFFFF.
    // Execute to compiled version of this instruction as a workaround.
    unsafe { sm0.exec_instr(0xa02b) };

    // SM1 continuously monitors the ACK line and sets IRQ1 if there is an acknowledge.
    let psx_ack = pio_proc::pio_file!("src/psx.pio", select_program("ack"));
    let psx_ack = RelocatedProgram::new_with_origin(&psx_ack.program, psx_spi_len as u8);
    let mut config = pio::Config::default();
    config.use_program(&common.load_program(&psx_ack), &[]);
    config.set_in_pins(&[&dsr_ack]);
    sm1.set_config(&config);
    sm1.set_enable(true);

    // Use a GPIO as output to control the chip select line.
    let mut dtr_cs = Output::new(p.PIN_5, Level::High);

    // Configure the USB stack
    let driver = usb::Driver::new(p.USB, Irqs);

    // The state must be defined before the builder for correct drop order.
    // Otherwise the compiler fails to see that the state lives long enough.
    // let mut s0 = hid::State::new();
    // let mut s1 = hid::State::new();
    // let mut s2 = hid::State::new();
    // let mut s3 = hid::State::new();

    let mut config = embassy_usb::Config::new(0x045E, 0x028E);
    config.device_release = 0x0114;
    config.manufacturer = Some("©Microsoft Corporation");
    config.product = Some("Controller");
    config.serial_number = Some("04B229A");
    config.max_power = 500;
    config.max_packet_size_0 = 64;

    let mut device_descriptor = [0; 20];
    let mut config_descriptor = [0; 112];
    let mut bos_descriptor = [0; 12];
    let mut control_buf = [0; 64];

    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        &mut device_descriptor,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut control_buf,
    );
    // let c0 = build_hid_controller(&mut builder, &mut s0);
    // let c1 = build_hid_controller(&mut builder, &mut s1);
    // let c2 = build_hid_controller(&mut builder, &mut s2);
    // let c3 = build_hid_controller(&mut builder, &mut s3);

    const CLASS_VENDOR: u8 = 0xFF;
    const SUBCLASS_XINPUT: u8 = 0x5D;
    const PROTOCOL_WIRED: u8 = 0x01;
    //const PROTOCOL_WIRELESS: u8 = 0x81;
    let mut function = builder.function(CLASS_VENDOR, SUBCLASS_XINPUT, PROTOCOL_WIRED);
    let mut interface = function.interface();
    let mut alt = interface.alt_setting(CLASS_VENDOR, SUBCLASS_XINPUT, PROTOCOL_WIRED, None);

    // Unknown descriptor
    // https://www.partsnotincluded.com/understanding-the-xbox-360-wired-controllers-usb-data/
    alt.descriptor(
        0x21,
        &[
            0x00, 0x01, 0x01, 0x25, // unknown
            0x81, // IN endpoint
            0x14, // IN data size
            0x00, 0x00, 0x00, 0x00, 0x13, // unknown
            0x01, // OUT endpoint
            0x08, // OUT data size
            0x00, 0x00, // unknown
        ],
    );

    let mut ep_in = alt.endpoint_interrupt_in(32, 4);
    let _ep_out = alt.endpoint_interrupt_out(32, 8);

    drop(function);

    let mut usb = builder.build();

    // USB
    let usb = pin!(async { usb.run().await });

    // Controller data shawed between tasks
    let controller_data = &[CONTROLLER_DATA_INIT; 4];

    // Controllers
    // let c0 = pin!(async move { run_controller(c0, &controller_data[0]).await });
    // let c1 = pin!(async move { run_controller(c1, &controller_data[1]).await });
    // let c2 = pin!(async move { run_controller(c2, &controller_data[2]).await });
    // let c3 = pin!(async move { run_controller(c3, &controller_data[3]).await });

    let c0 = pin!(async move {
        loop {
            let psx_data = controller_data[0].get();
            let mut xinput_data = [0_u8; 20];

            xinput_data[0] = 0; // Message type
            xinput_data[1] = 20; // Length

            let map_bit = |psx_byte: u8, from_bit, to_bit| {
                if psx_byte & (1_u8 << from_bit) == 0_u8 {
                    1_u8 << to_bit
                } else {
                    0
                }
            };

            xinput_data[2] |= map_bit(psx_data[0], 4, 0); // dpad up
            xinput_data[2] |= map_bit(psx_data[0], 6, 1); // dpad down
            xinput_data[2] |= map_bit(psx_data[0], 7, 2); // dpad left
            xinput_data[2] |= map_bit(psx_data[0], 5, 3); // dpad right
            xinput_data[2] |= map_bit(psx_data[0], 3, 4); // start
            xinput_data[2] |= map_bit(psx_data[0], 0, 5); // select
            xinput_data[2] |= map_bit(psx_data[0], 1, 6); // L3
            xinput_data[2] |= map_bit(psx_data[0], 2, 7); // R3

            xinput_data[3] |= map_bit(psx_data[1], 2, 0); // L1
            xinput_data[3] |= map_bit(psx_data[1], 3, 1); // R1
            xinput_data[3] |= map_bit(psx_data[1], 6, 4); // A
            xinput_data[3] |= map_bit(psx_data[1], 5, 5); // B
            xinput_data[3] |= map_bit(psx_data[1], 7, 6); // X
            xinput_data[3] |= map_bit(psx_data[1], 4, 7); // Y

            xinput_data[4] = map_bit(psx_data[1], 0, 0) * 0xFF; // L2
            xinput_data[5] = map_bit(psx_data[1], 1, 0) * 0xFF; // R2

            let scale_axis = |psx_axis: u8, invert: bool| {
                let mut xinput_axis = ((i32::from(psx_axis) - 0x80) * 256)
                    .clamp(i16::MIN.into(), i16::MAX.into())
                    as i16;
                if invert {
                    xinput_axis = xinput_axis.saturating_neg();
                }
                xinput_axis.to_le_bytes()
            };
            [xinput_data[6], xinput_data[7]] = scale_axis(psx_data[4], false);
            [xinput_data[8], xinput_data[9]] = scale_axis(psx_data[5], true);
            [xinput_data[10], xinput_data[11]] = scale_axis(psx_data[2], false);
            [xinput_data[12], xinput_data[13]] = scale_axis(psx_data[3], true);

            //debug!("data={=[u8]:X}", data);
            unwrap!(ep_in.write(xinput_data.as_ref()).await);
        }
    });

    // PSX protocol: SPI with LSB first and special ack signal
    let psx_spi = pin!(async move {
        let mut led = Output::new(p.PIN_25, Level::Low);

        let mut ticker = Ticker::every(Duration::from_millis(2));
        let mut fail_count = 0;
        loop {
            ticker.next().await;
            led.set_low();

            dtr_cs.set_low();
            let poll_data = poll_psx(&mut sm0, &irq_flags, &mut p.DMA_CH0, &mut p.DMA_CH1).await;
            dtr_cs.set_high();

            if let Some(poll_data) = poll_data {
                led.set_high();

                controller_data[0].set(poll_data[2..8].try_into().unwrap());
                controller_data[1].set(poll_data[10..16].try_into().unwrap());
                controller_data[2].set(poll_data[18..24].try_into().unwrap());
                controller_data[3].set(poll_data[26..32].try_into().unwrap());
            } else {
                fail_count += 1;
                warn!("PSX SPI fail_count={=usize}", fail_count);
            }
        }
    });

    lilos::exec::run_tasks(
        &mut [usb, c0, /*c1, c2, c3,*/ psx_spi],
        lilos::exec::ALL_TASKS,
    )
}

#[defmt::panic_handler]
fn defmt_panic() -> ! {
    cortex_m::asm::udf();
}
