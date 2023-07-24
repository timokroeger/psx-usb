#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::Cell;
use core::pin::pin;

use defmt::{assert, assert_eq, *};
use embassy_futures::join::join;
use embassy_rp::gpio::{Level, Output, Pull};
use embassy_rp::pio::{self, Direction, IrqFlags, Pio, ShiftConfig, ShiftDirection, StateMachine};
use embassy_rp::relocate::RelocatedProgram;
use embassy_rp::{bind_interrupts, peripherals, usb, Peripheral};
use embassy_time::{with_timeout, Duration, Ticker};
use embassy_usb::driver::{Driver, Endpoint, EndpointIn};
use fixed::traits::ToFixed;
use fixed_macro::types::U56F8;
use {defmt_rtt as _, panic_probe as _};

type PsxData = Cell<[u8; 6]>;
const PSX_DATA_INIT: PsxData = PsxData::new([0xFF, 0xFF, 0x80, 0x80, 0x80, 0x80]);

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<peripherals::USB>;
    PIO0_IRQ_0 => pio::InterruptHandler<peripherals::PIO0>;
});

struct XInput<'d, D: Driver<'d>> {
    ep_in: D::EndpointIn,
    ep_out: D::EndpointOut,
}

impl<'d, D: Driver<'d>> XInput<'d, D> {
    fn new(builder: &mut embassy_usb::Builder<'d, D>, ep: u8) -> Self {
        const CLASS_VENDOR: u8 = 0xFF;
        const SUBCLASS_XINPUT: u8 = 0x5D;
        const PROTOCOL_WIRED: u8 = 0x01;
        //const PROTOCOL_WIRELESS: u8 = 0x81;
        let mut function = builder.function(CLASS_VENDOR, SUBCLASS_XINPUT, PROTOCOL_WIRED);
        let mut interface = function.interface();
        let mut alt = interface.alt_setting(CLASS_VENDOR, SUBCLASS_XINPUT, PROTOCOL_WIRED, None);

        let ep_in = 0x80 + ep;
        let ep_out = ep;

        // Unknown descriptor
        // https://www.partsnotincluded.com/understanding-the-xbox-360-wired-controllers-usb-data/
        alt.descriptor(
            0x21,
            &[
                0x00, 0x01, 0x01, 0x25,  // unknown
                ep_in, // IN endpoint
                0x14,  // IN data size
                0x00, 0x00, 0x00, 0x00, 0x13,   // unknown
                ep_out, // OUT endpoint
                0x08,   // OUT data size
                0x00, 0x00, // unknown
            ],
        );

        let ep_in = alt.endpoint_interrupt_in(32, 4);
        assert_eq!(ep_in.info().addr.index(), ep as usize);
        let ep_out = alt.endpoint_interrupt_out(32, 8);
        assert_eq!(ep_out.info().addr.index(), ep as usize);

        XInput { ep_in, ep_out }
    }
}

fn psx_to_xinput(psx_data: &[u8; 6]) -> [u8; 20] {
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
        let mut xinput_axis =
            ((i32::from(psx_axis) - 0x80) * 256).clamp(i16::MIN.into(), i16::MAX.into()) as i16;
        if invert {
            xinput_axis = xinput_axis.saturating_neg();
        }
        xinput_axis.to_le_bytes()
    };
    [xinput_data[6], xinput_data[7]] = scale_axis(psx_data[4], false);
    [xinput_data[8], xinput_data[9]] = scale_axis(psx_data[5], true);
    [xinput_data[10], xinput_data[11]] = scale_axis(psx_data[2], false);
    [xinput_data[12], xinput_data[13]] = scale_axis(psx_data[3], true);

    xinput_data
}

async fn run_controller<'d>(
    mut xinput: XInput<'d, usb::Driver<'d, peripherals::USB>>,
    psx_data: &PsxData,
) -> ! {
    loop {
        let psx_data = psx_data.get();
        let xinput_data = psx_to_xinput(&psx_data);

        //debug!("data={=[u8]:X}", xinput_data);
        unwrap!(xinput.ep_in.write(&xinput_data).await);
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

    //let mut config = embassy_usb::Config::new(0x045E, 0x0719);
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
    let mut c0 = XInput::new(&mut builder, 1);

    let mut usb = builder.build();

    // USB
    let usb = pin!(async { usb.run().await });

    // Controller data shawed between tasks
    let psx_data = &[PSX_DATA_INIT; 4];

    // Controllers
    let c0 = pin!(async move { run_controller(c0, &psx_data[0]).await });

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

                psx_data[0].set(poll_data[2..8].try_into().unwrap());
                psx_data[1].set(poll_data[10..16].try_into().unwrap());
                psx_data[2].set(poll_data[18..24].try_into().unwrap());
                psx_data[3].set(poll_data[26..32].try_into().unwrap());
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
