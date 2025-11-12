#[derive(Clone, Copy, Debug)]
pub enum Error<E>{
    /// Error during I2C write/read operation.
    I2c(E),

    /// Measuremen is not completed AHT is not ready
    Busy(E),
}