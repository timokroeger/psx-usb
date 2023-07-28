#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod xinput;

use core::pin::pin;

use ::pio::{Instruction, InstructionOperands, MovDestination, MovOperation, MovSource};
use defmt::*;
use embassy_futures::join::join;
use embassy_rp::gpio::{Level, Output, Pull};
use embassy_rp::pio::{self, Direction, IrqFlags, Pio, ShiftConfig, ShiftDirection, StateMachine};
use embassy_rp::{bind_interrupts, peripherals, usb, Peripheral};
use embassy_time::{with_timeout, Duration, Ticker};
use fixed::traits::ToFixed;
use fixed_macro::types::U56F8;
use xinput::{SerialNumberHandler, State, XInput};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<peripherals::USB>;
    PIO0_IRQ_0 => pio::InterruptHandler<peripherals::PIO0>;
});

const CONTROLLER_STATE_INIT: State = State::new();
static CONTROLLER_STATE: [State; 4] = [CONTROLLER_STATE_INIT; 4];

fn psx_to_xinput(psx_data: &[u8; 6]) -> [u8; 12] {
    let mut xinput_data = [0_u8; 12];

    let map_bit = |psx_byte: u8, from_bit, to_bit| {
        if psx_byte & (1_u8 << from_bit) == 0_u8 {
            1_u8 << to_bit
        } else {
            0
        }
    };

    xinput_data[0] |= map_bit(psx_data[0], 4, 0); // dpad up
    xinput_data[0] |= map_bit(psx_data[0], 6, 1); // dpad down
    xinput_data[0] |= map_bit(psx_data[0], 7, 2); // dpad left
    xinput_data[0] |= map_bit(psx_data[0], 5, 3); // dpad right
    xinput_data[0] |= map_bit(psx_data[0], 3, 4); // start
    xinput_data[0] |= map_bit(psx_data[0], 0, 5); // select
    xinput_data[0] |= map_bit(psx_data[0], 1, 6); // L3
    xinput_data[0] |= map_bit(psx_data[0], 2, 7); // R3

    xinput_data[1] |= map_bit(psx_data[1], 2, 0); // L1
    xinput_data[1] |= map_bit(psx_data[1], 3, 1); // R1
    xinput_data[1] |= map_bit(psx_data[1], 6, 4); // A
    xinput_data[1] |= map_bit(psx_data[1], 5, 5); // B
    xinput_data[1] |= map_bit(psx_data[1], 7, 6); // X
    xinput_data[1] |= map_bit(psx_data[1], 4, 7); // Y

    xinput_data[2] = map_bit(psx_data[1], 0, 0) * 0xFF; // L2
    xinput_data[3] = map_bit(psx_data[1], 1, 0) * 0xFF; // R2

    let scale_axis = |psx_axis: u8, invert: bool| {
        let mut xinput_axis =
            ((i32::from(psx_axis) - 0x80) * 256).clamp(i16::MIN.into(), i16::MAX.into()) as i16;
        if invert {
            xinput_axis = xinput_axis.saturating_neg();
        }
        xinput_axis.to_le_bytes()
    };
    [xinput_data[4], xinput_data[5]] = scale_axis(psx_data[4], false);
    [xinput_data[6], xinput_data[7]] = scale_axis(psx_data[5], true);
    [xinput_data[8], xinput_data[9]] = scale_axis(psx_data[2], false);
    [xinput_data[10], xinput_data[11]] = scale_axis(psx_data[3], true);

    xinput_data
}

async fn poll_psx(
    sm0: &mut StateMachine<'_, peripherals::PIO0, 0>,
    irq_flags: &IrqFlags<'_, peripherals::PIO0>,
    rx_dma: &mut peripherals::DMA_CH0,
    tx_dma: &mut peripherals::DMA_CH1,
) -> Option<[u8; 32]> {
    let mut rumble_data = [(0_u8, 0_u8); 4];
    for (rumble_data, state) in rumble_data.iter_mut().zip(&CONTROLLER_STATE) {
        *rumble_data = state.rumble();
    }

    let mut command = [0_u8; 35];
    let mut response = [0_u8; 35];

    // header
    command[0] = 0x01; // controller (not memory card)
    command[1] = 0x42; // poll
    command[2] = 0x01; // multitap

    // controller data
    for i in 0..4 {
        let offset = 3 + i * 8;
        let (strong, weak) = CONTROLLER_STATE[i].rumble();
        command[offset + 0] = 0x42; // poll
        command[offset + 4] = weak; // right small motor
        command[offset + 5] = strong; // left big motor
    }

    let (rx, tx) = sm0.rx_tx();
    let tx_fut = tx.dma_push(tx_dma.into_ref(), &command);
    let rx_fut = rx.dma_pull(rx_dma.into_ref(), &mut response);

    // Start the transaction by setting the ACK irq flag.
    irq_flags.set(0);
    let _ = with_timeout(Duration::from_micros(1800), join(tx_fut, rx_fut)).await;
    sm0.clear_fifos();

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
    let mut config = pio::Config::default();
    config.use_program(&common.load_program(&psx_spi.program), &[&sck_clk]);
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
    // Execute the compiled version of this instruction from here as a workaround.
    // Also has the nice side effect that it sets the clock line to idle high.
    unsafe {
        sm0.exec_instr(
            Instruction {
                operands: InstructionOperands::MOV {
                    destination: MovDestination::X,
                    op: MovOperation::Invert,
                    source: MovSource::NULL,
                },
                delay: 0,
                side_set: Some(1),
            }
            .encode(psx_spi.program.side_set),
        )
    };
    sm0.set_enable(true);

    // SM1 continuously monitors the ACK line and sets IRQ1 if there is an acknowledge.
    let psx_ack = pio_proc::pio_file!("src/psx.pio", select_program("ack"));
    let mut config = pio::Config::default();
    config.use_program(&common.load_program(&psx_ack.program), &[]);
    config.set_in_pins(&[&dsr_ack]);
    sm1.set_config(&config);
    sm1.set_enable(true);

    // Use a GPIO as output to control the chip select line.
    let mut dtr_cs = Output::new(p.PIN_5, Level::High);

    // Configure the USB stack
    let driver = usb::Driver::new(p.USB, Irqs);

    let mut config = embassy_usb::Config::new(0x045E, 0x0719);
    config.device_class = 0xFF;
    config.device_sub_class = 0xFF;
    config.device_protocol = 0xFF;
    config.device_release = 0x0100;
    config.manufacturer = Some("©Microsoft");
    config.product = Some("Xbox 360 Wireless Receiver for Windows");
    config.serial_number = Some("E0CB7AD0");
    config.max_power = 260;
    config.max_packet_size_0 = 64;

    let mut device_descriptor = [0; 20];
    let mut config_descriptor = [0; 324];
    let mut bos_descriptor = [0; 12];
    // Must be bigger than than n * 2 + 2 where n is the number of characters
    // in the longest string descriptor.
    let mut control_buf = [0; 128];

    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        &mut device_descriptor,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut control_buf,
    );

    // The first 4 bytes should match the USB serial number descriptor.
    let mut serial_number_handler = SerialNumberHandler([0xe0, 0xcb, 0x7a, 0xd0, 0x0a, 0x89, 0xb7]);
    builder.handler(&mut serial_number_handler);

    let mut c0 = XInput::new_wireless(&mut builder, &CONTROLLER_STATE[0]);
    let mut c1 = XInput::new_wireless(&mut builder, &CONTROLLER_STATE[1]);
    let mut c2 = XInput::new_wireless(&mut builder, &CONTROLLER_STATE[2]);
    let mut c3 = XInput::new_wireless(&mut builder, &CONTROLLER_STATE[3]);

    let mut usb = builder.build();

    // USB
    let usb = pin!(async { usb.run().await });

    // Controllers
    let c0 = pin!(async move { c0.run().await });
    let c1 = pin!(async move { c1.run().await });
    let c2 = pin!(async move { c2.run().await });
    let c3 = pin!(async move { c3.run().await });

    let mut prev_psx_data = [[0xFF_u8, 0xFF, 0x80, 0x80, 0x80, 0x80]; 4];

    // PSX protocol: SPI with LSB first and special ack signal
    let psx_spi = pin!(async {
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

                for i in 0..CONTROLLER_STATE.len() {
                    let idx_from = 2 + 8 * i;
                    let idx_to = idx_from + 6;

                    let prev_psx_data = &mut prev_psx_data[i];
                    let psx_data: [u8; 6] = unwrap!(poll_data[idx_from..idx_to].try_into());
                    if *prev_psx_data != psx_data {
                        let state = &CONTROLLER_STATE[i];
                        state.set_available(true);
                        state.send_xinput(psx_to_xinput(&psx_data));
                        *prev_psx_data = psx_data;
                    }
                }
            } else {
                fail_count += 1;
                warn!("PSX SPI fail_count={=usize}", fail_count);
            }
        }
    });

    lilos::exec::run_tasks(&mut [usb, c0, c1, c2, c3, psx_spi], lilos::exec::ALL_TASKS)
}

#[defmt::panic_handler]
fn defmt_panic() -> ! {
    cortex_m::asm::udf();
}
