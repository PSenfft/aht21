#![no_std]

pub mod error;
use crate::error::Error;

#[cfg(not(feature = "async"))]
use embedded_hal::{delay::DelayNs, i2c::I2c};

#[cfg(feature = "async")]
use embedded_hal_async::{delay::DelayNs as AsyncDelayNs, i2c::I2c as AsyncI2c};
use libm::powf;
use log::{debug, info};

pub const DEVICE_ADDRESS: u8 = 0x38;
pub const STATUS_REGISTER: u8 = 0x71;
pub const DEVICE_INIT_DONE: u8 = 0x18;
pub const MEASURE_COMMAND: u8 = 0xAC;
pub const MEASURE_COMMAND_DATA_0: u8 = 0x33;
pub const MEASURE_COMMAND_DATA_1: u8 = 0x00;

pub const STARTUP_DELAY: u8 = 100; // ms
pub const AFTER_STARTUP_DELAY: u8 = 10; //ms
pub const MEASURE_DELAY: u8 = 80; //ms

pub struct Aht21<I2C, D> {
    /// I²C interface
    i2c: I2C,

    /// I²C device address
    address: u8,
    delayer: D,
}

pub struct Measurement {
    humidity: f32,
    temperature: f32,
}

#[cfg(not(feature = "async"))]
impl<I2C, D, E> Aht21<I2C, D>
where
    I2C: I2c<Error = E>,
    D: DelayNs,
{
    /// create new AHT21 driver
    pub fn new(i2c: I2C, delayer: D) -> Self {
        debug!("new called");
        Self {
            i2c,
            address: DEVICE_ADDRESS,
            delayer,
        }
    }

    /// give back the I2C interface
    pub fn release(self) -> I2C {
        self.i2c
    }
}

#[cfg(feature = "async")]
impl<I2C, D, E> Aht21<I2C, D>
where
    I2C: I2c<Error = E>,
    D: DelayNs,
{
    /// create new AHT21 driver
    pub fn new(i2c: I2C, delayer: D) -> Self {
        debug!("new called");
        Self {
            i2c,
            address: DEVICE_ADDRESS,
            delayer,
        }
    }

    /// give back the I2C interface
    pub fn release(self) -> I2C {
        self.i2c
    }
}

#[maybe_async_cfg::maybe(
    sync(
        cfg(not(feature = "async")),
        self = "Aht21",
        idents(AsyncI2c(sync = "I2c"), AsyncDelayNs(sync = "DelayNs"))
    ),
    async(feature = "async", keep_self)
)]

impl<I2C, D, E> Aht21<I2C, D>
where
    I2C: AsyncI2c<Error = E>,
    D: AsyncDelayNs,
{
    // command_buf is an u8 array that starts with command byte followed by command data byte(s)
    async fn write_command<const N: usize>(
        &mut self,
        command_buf: [u8; N],
    ) -> Result<(), Error<E>> {
        // debug!("write_command : {:#?}", command_buf);
        self.i2c
            .write(self.address, &command_buf)
            .await
            .map_err(Error::I2c)
    }

    async fn read_register(
        &mut self,
        register_address: u8,
        buffer: &mut [u8],
    ) -> Result<(), Error<E>> {
        let mut command_buffer = [0u8; 1];
        command_buffer[0] = register_address;
        // let mut result_buffer = [0u8; N];
        self.i2c
            .write_read(self.address, &command_buffer, buffer)
            .await
            .map_err(Error::I2c)?;
        Ok(())
    }

    async fn write_and_read_register<const N: usize>(
        &mut self,
        command_buf: [u8; N],
        result_buffer: &mut [u8],
    ) -> Result<(), Error<E>> {
        // let mut result_buffer = [0u8; N];
        self.i2c
            .write_read(self.address, &command_buf, result_buffer)
            .await
            .map_err(Error::I2c)?;
        Ok(())
    }

    pub async fn startup(&mut self) {
        self.delayer.delay_ms(STARTUP_DELAY.into()).await;
        let mut result_buf = [0; 5];
        self.write_and_read_register([DEVICE_ADDRESS, STATUS_REGISTER], &mut result_buf);
        info!("{:?}", result_buf);
        self.delayer.delay_ms(AFTER_STARTUP_DELAY as u32);
        todo!("proof if status register result == DEVICE INIT iS DONE");
    }

    pub async fn measure(&mut self) {
        let mut result_buf = [0; 5];
        self.write_command([
            DEVICE_ADDRESS,
            MEASURE_COMMAND,
            MEASURE_COMMAND_DATA_0,
            MEASURE_COMMAND_DATA_1,
        ]);
        self.delayer.delay_ms(MEASURE_DELAY.into()).await;
        self.read_register(DEVICE_ADDRESS, &mut result_buf);
        info!("{:?}", result_buf);
    }
}

//20 Bit register value to RH[%]
fn convet_humidity(humidity: u32) -> f32 {
    let f_humidity: f32 = humidity as f32;
    let result = (f_humidity / powf(2.0, 20.0)) * 100.0;
    result
}

fn convert_to_celcius(temperature: u32) -> f32 {
    let f_temperature: f32 = temperature as f32;
    let result = (f_temperature / powf(2.0, 20.0)) * 200.0 - 50.0;
    result
}

fn convert_to_fahrenheit(temperature: u32) -> f32 {
    let result = (convert_to_celcius(temperature) * 9.0 / 5.0) + 32.0;
    result
}

#[cfg(test)]
mod tests {

    use super::*;
    use libm::truncf;

    fn truncate_to_2(x: f32) -> f32 {
        let factor = 100.0;
        truncf(x * factor) / factor
    }
    #[test]
    fn convet_humidity_test() {
        let max_humidity: u32 = 0b11111111111111111111;
        let result = convet_humidity(max_humidity);
        assert_eq!(truncate_to_2(result), 99.99);
    }

    #[test]
    fn convert_to_celcius_max_test() {
        let max_temp: u32 = 0b11111111111111111111;
        let result = convert_to_celcius(max_temp);
        assert_eq!(truncate_to_2(result), 149.99);
    }

    #[test]
    fn convert_to_celcius_min_test() {
        let max_temp: u32 = 0b0;
        let result = convert_to_celcius(max_temp);
        assert_eq!(truncate_to_2(result), -50.0);
    }

    #[test]
    fn convert_to_fahrenheit_test() {
        let max_temp: u32 = 0b11111111111111111111;
        let result = convert_to_fahrenheit(max_temp);
        assert_eq!(truncate_to_2(result), 301.99);
    }
}
