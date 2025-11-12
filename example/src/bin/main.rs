#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    mutex::Mutex,
};
use embassy_embedded_hal::shared_bus::asynch::i2c::{self, I2cDevice};
use embassy_time::{Delay, Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::InputConfig;
use esp_hal::gpio::{AnyPin, Input};
use esp_hal::i2c::master;
use esp_hal::i2c::master::I2c;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{Async, Config};
use log::{error, info};
use static_cell::StaticCell;

extern crate alloc;

use aht21;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.0.1

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let p = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);

    let timg0 = TimerGroup::new(p.TIMG0);
    let sw_interrupt = esp_hal::interrupt::software::SoftwareInterruptControl::new(p.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    static I2C_BUS: StaticCell<Mutex<NoopRawMutex, esp_hal::i2c::master::I2c<Async>>> =
        StaticCell::new();
    let i2c = esp_hal::i2c::master::I2c::new(
        p.I2C0,
        Config::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(p.GPIO22)
    .with_scl(p.GPIO23)
    .into_async();

    let i2c_bus = &*I2C_BUS.init(Mutex::new(i2c));

    let aht21_device: i2c::I2cDevice<'static, NoopRawMutex, master::I2c<'static, Async>> =
        I2cDevice::new(i2c_bus);

    spawner.spawn(reading_task(aht21_device)).unwrap();
}

#[embassy_executor::task]
async fn reading_task(
    mut aht21: Aht21<I2cDevice<'static, NoopRawMutex, I2c<'static, Async>>, Delay>,
) {
    aht21.startup_aht21();
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    loop {
        error!("PANIC: {info}");
    }
}
