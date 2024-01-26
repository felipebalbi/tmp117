//! This is a platform agnostic Rust driver for the TMP117 temperature sensor
//! based on the [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
//!
//! This driver allows you to:
//! - Enable/disable the device by entering/exiting Shutdown Mode
//! - Initiate Soft-reset
//! - Read the temperature
//! - Set the minimum alert temperature
//! - Set the maximum alert temperature
//! - Configure conversion mode between Continuous, One-shot, and Therm and
//!   Alert
//! - Set the ALERT pin polarity
//!
//! For further details of the device architecture and operation, please refer
//! to the official Datasheet.
//!
//! Datasheet:
//!   - [TMP117](https://www.ti.com/lit/ds/symlink/tmp117.pdf)
//!
//! # Usage examples
//!
//! To use this driver, import this crate and an `embedded_hal` implementation
//! of your choice, then instantiate the `tmp117` device.
//!
//! ### Read temperature in Celsius
//! ### Provide an alternative I2C address
//! ### Alert interrupts
//! ### Set Alert pin polarity
//! ### Set conversion mode
//! ### Enter/exit Shutdown mode

#![doc(html_root_url = "https://docvs.rs/tmp117/latest")]
#![deny(missing_docs, unsafe_code)]
#![no_std]

use bilge::prelude::*;
use embedded_hal::i2c::I2c;

/// I2C device address.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Address(pub(crate) u8);

/// ADD0 Pin logic level representation.
#[derive(Debug)]
pub enum Add0Connection {
    /// ADD0 tied to GND (default).
    Ground,
    /// ADD0 tied to V+.
    Vplus,
    /// ADD0 tied to SDA.
    Sda,
    /// ADD0 tied to SCL.
    Scl,
}

/// Conversion Mode.
#[bitsize(2)]
#[derive(Debug, FromBits)]
pub enum ConversionMode {
    /// Continuous conversion (default).
    #[fallback]
    Continuous,
    /// Shutdown.
    Shutdown,
    /// One-shot conversion.
    OneShot,
}

/// Conversion cycle.
/// TODO!

/// Conversion Averaging support.
#[bitsize(2)]
#[derive(Debug, FromBits)]
pub enum Average {
    /// No averaging
    NoAverage,
    /// 8 averaged conversions.
    Average8,
    /// 32 averaged conversions.
    Average32,
    /// 64 averaged conversions.
    Average64,
}

/// ALERT Mode Select.
#[bitsize(1)]
#[derive(Debug, FromBits)]
pub enum AlertMode {
    /// Alert mode (default).
    Alert,
    /// Therm mode.
    Therm,
}

/// ALERT pin polarity.
#[bitsize(1)]
#[derive(Debug, FromBits)]
pub enum Polarity {
    /// Active low (default).
    ActiveLow,
    /// Active High.
    ActiveHigh,
}

/// ALERT pin select.
#[bitsize(1)]
#[derive(Debug, FromBits)]
pub enum AlertSelect {
    /// ALERT pin reflects the status of the alert flags (default).
    Alert,
    /// ALERT pin reflects the status of the data ready flag.
    DataReady,
}

/// Default address.
impl Default for Address {
    fn default() -> Self {
        Address::from(Add0Connection::Ground)
    }
}

/// Compute device address from state of ADD0 pin connection.
impl From<Add0Connection> for Address {
    fn from(connection: Add0Connection) -> Self {
        match connection {
            Add0Connection::Ground => Address(0b100_1000),
            Add0Connection::Vplus => Address(0b100_1001),
            Add0Connection::Sda => Address(0b100_1010),
            Add0Connection::Scl => Address(0b100_1011),
        }
    }
}

#[bitsize(16)]
#[derive(DebugBits, FromBits)]
struct TempResult {
    t: u16,
}

#[bitsize(16)]
#[derive(DebugBits, FromBits, Copy, Clone)]
struct Configuration {
    _reserved: bool,
    soft_reset: bool,
    dr_alert: AlertSelect,
    high_alert: bool,
    pol: Polarity,
    t_na: AlertSelect,
    avg: Average,
    conv: u3,
    mode: ConversionMode,
    eeprom_busy: bool,
    data_ready: bool,
    low_alert: bool,
}

impl Default for Configuration {
    fn default() -> Self {
        Self::from(0x0220)
    }
}

#[bitsize(16)]
#[derive(DebugBits, FromBits)]
struct DeviceId {
    did: u12,
    rev: u4,
}

/// Register representation
struct Register;

/// Register implementation
impl Register {
    const TEMP_RESULT: u8 = 0x00;
    const CONFIGURATION: u8 = 0x01;
    const THIGH_LIMIT: u8 = 0x02;
    const TLOW_LIMIT: u8 = 0x03;
    const EEPROM_UL: u8 = 0x04;
    const EEPROM1: u8 = 0x05;
    const EEPROM2: u8 = 0x06;
    const TEMP_OFFSET: u8 = 0x07;
    const EEPROM3: u8 = 0x08;
    const DEVICE_ID: u8 = 0x0f;
}

/// TMP117 device driver.
#[derive(Debug, Default)]
pub struct Tmp117<I2C> {
    /// The concrete I2C device implementation.
    i2c: I2C,
    /// The I2C address.
    address: Address,
    /// Configuration register contents
    config: Configuration,
}

impl<I2C: I2c> Tmp117<I2C> {
    const CELSIUS_PER_BIT: f32 = 7.8125 / 1000.0;
    const TMP117: u16 = 0x0117;
}

impl<I2C: I2c> Tmp117<I2C> {
    /// Create a new instance of TMP117 device with default address.
    pub fn new(i2c: I2C) -> Self {
        let address = Address::default();
        let config = Configuration::default();
        Self::build_instance(i2c, address, config)
    }

    /// Create a new instance of TMP117 device with given state of ADD0 pin.
    pub fn new_with_add0_connection(i2c: I2C, connection: Add0Connection) -> Self {
        let address = Address::from(connection);
        let config = Configuration::default();
        Self::build_instance(i2c, address, config)
    }

    /// Create a new instance of TMP117 device with given address.
    pub fn new_with_address(i2c: I2C, address: Address) -> Self {
        let config = Configuration::default();
        Self::build_instance(i2c, address, config)
    }

    /// Destroy driver instance, return I2C bus instance.
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// Enable the sensor (default state).
    pub fn enable(&mut self) -> Result<(), I2C::Error> {
        self.config.set_mode(ConversionMode::Continuous);
        let contents = u16::to_be_bytes(u16::from(self.config));
        self.write_register(Register::CONFIGURATION, contents)
    }

    /// Disable the sensor.
    pub fn disable(&mut self) -> Result<(), I2C::Error> {
        self.config.set_mode(ConversionMode::Shutdown);
        let contents = u16::to_be_bytes(u16::from(self.config));
        self.write_register(Register::CONFIGURATION, contents)
    }

    /// Reset the sensor
    pub fn reset(&mut self) -> Result<(), I2C::Error> {
        self.config.set_soft_reset(true);
        let contents = u16::to_be_bytes(u16::from(self.config));
        self.config.set_soft_reset(false);
        self.write_register(Register::CONFIGURATION, contents)
    }

    /// Set Temperature High Limit
    pub fn set_thigh_limit(&mut self, limit: f32) -> Result<(), I2C::Error> {
        let temp = (limit / Self::CELSIUS_PER_BIT) as u16;
        let contents = u16::to_be_bytes(temp);
        self.write_register(Register::THIGH_LIMIT, contents)
    }

    /// Set Temperature Low Limit
    pub fn set_tlow_limit(&mut self, limit: f32) -> Result<(), I2C::Error> {
        let temp = (limit / Self::CELSIUS_PER_BIT) as u16;
        let contents = u16::to_be_bytes(temp);
        self.write_register(Register::TLOW_LIMIT, contents)
    }

    fn write_register(&mut self, register: u8, contents: [u8; 2]) -> Result<(), I2C::Error> {
        let mut data = [0; 3];

        data[0] = register;
        data[1] = contents[0];
        data[2] = contents[1];

        self.i2c.write(self.address.0, &data)
    }
}

impl<I2C: I2c> Tmp117<I2C> {
    /// Identify a TMP117 device.
    pub fn identify(i2c: I2C) -> Option<Self> {
        let address = Address::default();
        let config = Configuration::default();
        Self::validate(i2c, address, config)
    }

    /// Identify a TMP117 device with given state of ADD0 pin.
    pub fn identify_with_add0_connection(i2c: I2C, connection: Add0Connection) -> Option<Self> {
        let address = Address::from(connection);
        let config = Configuration::default();
        Self::validate(i2c, address, config)
    }

    /// Identify TMP117 device with given address.
    pub fn identify_with_address(i2c: I2C, address: Address) -> Option<Self> {
        let config = Configuration::default();
        Self::validate(i2c, address, config)
    }

    /// Read Device ID Register
    pub fn device_id(&mut self) -> Result<u16, I2C::Error> {
        let bytes = self.read_register(Register::DEVICE_ID)?;
        let id = DeviceId::from(bytes);
        Ok(id.into())
    }

    /// Read temperature in Celsius
    pub fn read_temperature(&mut self) -> Result<f32, I2C::Error> {
        let bytes = self.read_register(Register::TEMP_RESULT)?;
        let temperature = (bytes as i16 as f32) * Self::CELSIUS_PER_BIT;
        Ok(temperature)
    }

    /// Read temperature limits
    pub fn read_temperature_limits(&mut self) -> Result<(f32, f32), I2C::Error> {
        let mut bytes = self.read_register(Register::THIGH_LIMIT)?;
        let high = (bytes as i16 as f32) * Self::CELSIUS_PER_BIT;
        bytes = self.read_register(Register::TLOW_LIMIT)?;
        let low = (bytes as i16 as f32) * Self::CELSIUS_PER_BIT;

        Ok((high, low))
    }

    fn validate(i2c: I2C, address: Address, config: Configuration) -> Option<Self> {
        let mut tmp = Self::build_instance(i2c, address, config);
        if let Ok(id) = tmp.device_id() {
            if id != Self::TMP117 {
                None
            } else {
                Some(tmp)
            }
        } else {
            None
        }
    }

    fn read_register(&mut self, register: u8) -> Result<u16, I2C::Error> {
        let mut bytes = [0; 2];
        self.i2c
            .write_read(self.address.0, &[register], &mut bytes)?;
        Ok(u16::from_be_bytes(bytes))
    }
}

impl<I2C> Tmp117<I2C> {
    fn build_instance(i2c: I2C, address: Address, config: Configuration) -> Self {
        Self {
            i2c,
            address,
            config,
        }
    }
}
