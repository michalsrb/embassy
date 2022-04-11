#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use cortex_m::peripheral::{nvic, NVIC};
use defmt::panic;
use defmt::*;
use defmt_rtt as _; // global logger
use embassy::executor::Spawner;
use embassy::time::{Duration, Timer};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::interrupt;
use embassy_stm32::pac::Interrupt;
use embassy_stm32::rcc::low_level::RccPeripheral;
use embassy_stm32::rcc::*;
use embassy_stm32::time::Hertz;
use embassy_stm32::usb::{Driver, Instance};
use embassy_stm32::{pac, peripherals};
use embassy_stm32::{Config, Peripherals};
use embassy_usb::driver::EndpointError;
use embassy_usb::UsbDeviceBuilder;
use embassy_usb_serial::{CdcAcmClass, State};
use futures::future::join;
use panic_probe as _;

fn config() -> Config {
    let mut config = Config::default();
    config.rcc.mux = ClockSrc::HSE(Hertz(16_000_000));

    config.rcc.mux = ClockSrc::PLL(
        PLLSource::HSI16,
        PLLClkDiv::Div2,
        PLLSrcDiv::Div1,
        PLLMul::Mul8,
        Some(PLLClkDiv::Div2),
    );
    config.rcc.hsi48 = true;

    config
}

#[embassy::main(config = "config()")]
async fn main(_spawner: Spawner, p: Peripherals) {
    info!("Hello World!");

    let regs = pac::USB;

    // Create the driver, from the HAL.
    let irq = interrupt::take!(USB_FS);
    let driver = Driver::new(p.USB, irq, p.PA12, p.PA11);

    // Create embassy-usb Config
    let config = embassy_usb::Config::new(0xc0de, 0xcafe);

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut device_descriptor = [0; 256];
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 7];

    let mut state = State::new();

    let mut builder = UsbDeviceBuilder::new(
        driver,
        config,
        &mut device_descriptor,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut control_buf,
    );

    // Create classes on the builder.
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the builder.
    let mut usb = builder.build().await;

    // Run the USB device.
    let usb_fut = usb.run();

    // Do stuff with the class!
    let echo_fut = async {
        loop {
            class.wait_connection().await;
            info!("Connected");
            let _ = echo(&mut class).await;
            info!("Disconnected");
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, echo_fut).await;
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn echo<'d, T: Instance + 'd>(
    class: &mut CdcAcmClass<'d, Driver<'d, T>>,
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        info!("data: {:x}", data);
        class.write_packet(data).await?;
    }
}
