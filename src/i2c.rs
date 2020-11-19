//! I2C
use hal::blocking::i2c::{Read, Write, WriteRead};

use crate::gpio::{gpioa::*, gpiob::*};
use crate::gpio::{AltFunction, OpenDrain, Output};
use crate::rcc::Rcc;
use crate::stm32;
use crate::stm32::{I2C1, I2C2};
use crate::time::Hertz;
use core::cmp;
use core::marker::PhantomData;
use cortex_m::asm;
use rtt_target::rprintln;

pub struct Config {
    speed: Option<Hertz>,
    timing: Option<u32>,
    analog_filter: bool,
    digital_filter: u8,
}

impl Config {
    pub fn new<T>(speed: T) -> Self
    where
        T: Into<Hertz>,
    {
        Config {
            speed: Some(speed.into()),
            timing: None,
            analog_filter: true,
            digital_filter: 0,
        }
    }

    pub fn with_timing(timing: u32) -> Self {
        Config {
            timing: Some(timing),
            speed: None,
            analog_filter: true,
            digital_filter: 0,
        }
    }

    pub fn disable_analog_filter(mut self) -> Self {
        self.analog_filter = false;
        self
    }

    pub fn enable_digital_filter(mut self, cycles: u8) -> Self {
        assert!(cycles <= 16);
        self.digital_filter = cycles;
        self
    }

    fn timing_bits(&self, i2c_clk: Hertz) -> u32 {
        if let Some(bits) = self.timing {
            return bits;
        }
        let speed = self.speed.unwrap();
        let (psc, scll, sclh, sdadel, scldel) = if speed.0 <= 100_000 {
            let psc = 3;
            let scll = cmp::max(((i2c_clk.0 >> (psc + 1)) / speed.0) - 1, 255);
            let sclh = scll - 4;
            let sdadel = 2;
            let scldel = 4;
            (psc, scll, sclh, sdadel, scldel)
        } else {
            let psc = 1;
            let scll = cmp::max(((i2c_clk.0 >> (psc + 1)) / speed.0) - 1, 255);
            let sclh = scll - 6;
            let sdadel = 1;
            let scldel = 3;
            (psc, scll, sclh, sdadel, scldel)
        };
        psc << 28 | scldel << 20 | sdadel << 16 | sclh << 8 | scll
    }
}

/// I2C abstraction
pub struct I2c<I2C, SDA, SCL, Mode> {
    i2c: I2C,
    sda: SDA,
    scl: SCL,
    _mode: PhantomData<Mode>,
}

// I2C SDA pin
pub trait SDAPin<I2C> {
    fn setup(&self);
}

// I2C SCL pin
pub trait SCLPin<I2C> {
    fn setup(&self);
}

// I2C error
#[derive(Debug)]
pub enum Error {
    Overrun,
    Nack,
    PECError,
    BusError,
    ArbitrationLost,
    Timeout,
}

// I2C bus modes

/// The type of an I2C peripheral that has been created
/// but not yet assigned as a master or slave.
pub struct BusNone;

// The type of an I2C peripheral that has been assigned
// as a bus master.
pub struct BusMaster;

// The type of an I2C peripheral that has been assigned
// as a bus slave.
pub struct BusSlave;

pub trait I2cExt<I2C> {
    fn i2c_master<SDA, SCL>(
        self,
        sda: SDA,
        scl: SCL,
        config: Config,
        rcc: &mut Rcc,
    ) -> I2c<I2C, SDA, SCL, BusMaster>
    where
        SDA: SDAPin<I2C>,
        SCL: SCLPin<I2C>;

    /// Creates a new I2C object in slave mode.
    fn i2c_slave<SDA, SCL>(
        self,
        sda: SDA,
        scl: SCL,
        config: Config,
        addr: u8,
        rcc: &mut Rcc,
    ) -> I2c<I2C, SDA, SCL, BusSlave>
    where
        SDA: SDAPin<I2C>,
        SCL: SCLPin<I2C>;
}

// Sequence to flush the TXDR register. This resets the TXIS and TXE flags
macro_rules! flush_txdr {
    ($i2c:expr) => {
        // If a pending TXIS flag is set, write dummy data to TXDR
        if $i2c.isr.read().txis().bit_is_set() {
            $i2c.txdr.write(|w| unsafe { w.txdata().bits(0) });
        }

        // If TXDR is not flagged as empty, write 1 to flush it
        if $i2c.isr.read().txe().bit_is_clear() {
            $i2c.isr.write(|w| w.txe().set_bit());
        }
    };
}

// Sequence to flush the RXDR register.
macro_rules! flush_rxdr {
    ($i2c:expr) => {
        // RXNE=1 meaning that the previous received data byte has not yet been read
        // the SCL line is stretched low until I2C_RXDR is read
        if $i2c.isr.read().rxne().bit_is_set() {
            let _ = $i2c.rxdr.read();
        }
    };
}

macro_rules! busy_wait {
    ($i2c:expr, $flag:ident, $variant:ident) => {
        loop {
            let isr = $i2c.isr.read();

            if isr.$flag().$variant() {
                break;
            } else if isr.berr().bit_is_set() {
                $i2c.icr.write(|w| w.berrcf().set_bit());
                return Err(Error::BusError);
            } else if isr.arlo().bit_is_set() {
                $i2c.icr.write(|w| w.arlocf().set_bit());
                return Err(Error::ArbitrationLost);
            } else if isr.ovr().bit_is_set() {
                $i2c.icr.write(|w| w.ovrcf().set_bit());
                return Err(Error::Overrun);
            } else if isr.timeout().bit_is_set() {
                $i2c.icr.write(|w| w.timoutcf().set_bit());
                return Err(Error::Timeout);
            } else if isr.nackf().bit_is_set() {
                $i2c.icr.write(|w| w.stopcf().set_bit().nackcf().set_bit());
                flush_txdr!($i2c);
                return Err(Error::Nack);
            } else {
                // try again
            }
        }
    };
}

macro_rules! i2c {
    ($I2CX:ident, $i2cx:ident, $i2cxen:ident, $i2crst:ident,
        sda: [ $($PSDA:ty,)+ ],
        scl: [ $($PSCL:ty,)+ ],
    ) => {
        $(
            impl SDAPin<$I2CX> for $PSDA {
                fn setup(&self) {
                    self.set_alt_mode(AltFunction::AF6)
                }
            }
        )+

        $(
            impl SCLPin<$I2CX> for $PSCL {
                fn setup(&self) {
                    self.set_alt_mode(AltFunction::AF6)
                }
            }
        )+

        impl I2cExt<$I2CX> for $I2CX {
            fn i2c_master<SDA, SCL>(
                self,
                sda: SDA,
                scl: SCL,
                config: Config,
                rcc: &mut Rcc,
            ) -> I2c<$I2CX, SDA, SCL, BusMaster>
            where
                SDA: SDAPin<$I2CX>,
                SCL: SCLPin<$I2CX>,
            {
                I2c::$i2cx(self, sda, scl, config, rcc).into_master()
            }


            /// Creates a new I2C object in slave mode.
            fn i2c_slave<SDA, SCL>(
                self,
                sda: SDA,
                scl: SCL,
                config: Config,
                addr: u8,
                rcc: &mut Rcc,
            ) -> I2c<$I2CX, SDA, SCL, BusSlave>
            where
                SDA: SDAPin<$I2CX>,
                SCL: SCLPin<$I2CX>,
            {
                I2c::$i2cx(self, sda, scl, config, rcc).into_slave(addr)
            }
        }

        impl<SDA, SCL> I2c<$I2CX, SDA, SCL, BusNone> {
            pub fn $i2cx(i2c: $I2CX, sda: SDA, scl: SCL, config: Config, rcc: &mut Rcc) -> Self
            where
                SDA: SDAPin<$I2CX>,
                SCL: SCLPin<$I2CX>,
            {

                sda.setup();
                scl.setup();

                // Enable clock for I2C
                rcc.rb.apbenr1.modify(|_, w| w.$i2cxen().set_bit());

                // Reset I2C
                rcc.rb.apbrstr1.modify(|_, w| w.$i2crst().set_bit());
                rcc.rb.apbrstr1.modify(|_, w| w.$i2crst().clear_bit());

                // Make sure the I2C unit is disabled so we can configure it
                i2c.cr1.modify(|_, w| w.pe().clear_bit());

                // Setup protocol timings
                let i2c_clk = rcc.clocks.apb_clk;
                let timing_bits = config.timing_bits(i2c_clk);
                i2c.timingr.write(|w| unsafe { w.bits(timing_bits) });

                // Setup timeout values (SMBus compatible)
                // Note: This leaves the timeouts disabled, but we need the I2C
                // peripheral's clock frequency to set these up.

                // each bit is 4 I2C clock cycles.
                // Equation: 4 * (1/i2c_clk_hz) * (timeout_bits + 1) = timeout_sec
                // 50us for bus idle detection is specified by SMBus 2.0
                let timeouta_us = 50;
                let timeouta_bits = (i2c_clk.0 * timeouta_us / 1000 / 1000 / 4) - 1;
                // 25ms for SCL held low timeout is specified by SMBus 2.0
                let timeoutb_ms = 25;
                let timeoutb_bits = (i2c_clk.0 * timeoutb_ms / 1000 / 4) - 1;

                i2c.timeoutr.modify(|_, w| unsafe {
                    w
                        // TIMEOUTA is used to detect both
                        // SCL and SDA high timeout (bus idle condition)
                        .timeouta().bits(timeouta_bits as u16)
                        // TIMEOUTB is used to detect SCL being held low
                        // (clock stretched) for longer than is permitted
                        .timeoutb().bits(timeoutb_bits as u16)
                        // Leave all timeouts disabled
                        .tidle().clear_bit()
                        .timouten().clear_bit()
                        .texten().clear_bit()
                });

                // Enable the I2C processing
                i2c.cr1.modify(|_, w| unsafe {
                    w.pe()
                        .set_bit()
                        .dnf()
                        .bits(config.digital_filter)
                        .anfoff()
                        .bit(!config.analog_filter)
                });

                while !i2c.cr1.read().pe().bit_is_set() {
                    asm::nop();
                } // wait for enabled

                I2c { i2c, sda, scl, _mode: PhantomData }
            }
        }

        impl<SDA, SCL, Mode> I2c<$I2CX, SDA, SCL, Mode> {
            pub fn into_master(self) -> I2c<$I2CX, SDA, SCL, BusMaster> {

                let (i2c, sda, scl) = self.release();

                // Reset the bus, bringing it up in master mode
                // (sets the TXE (Transmit data register empty) interrupt bit)
                i2c.cr1.modify(|_, w| w.pe().clear_bit());
                while i2c.cr1.read().pe().bit_is_set() {
                    asm::nop();
                } // wait for disabled

                // Disable slave settings: clock stretch, slave byte control
                i2c.cr1.modify(|_, w| {
                    w.nostretch().set_bit().sbc().clear_bit()
                });

                // Clear own address register
                i2c.oar1.modify(|_, w| {
                    w.oa1en()
                        .clear_bit()
                        .oa1mode()
                        .clear_bit() // ensure we're in 7-bit mode
                });

                // Disable interrupts
                i2c.cr1.modify(|_, w| {
                    w.addrie()
                        .clear_bit()
                        .errie()
                        .clear_bit()
                        .stopie()
                        .clear_bit()
                        .nackie()
                        .clear_bit()
                        .tcie()
                        .clear_bit()
                });

                // Re-enable bus
                i2c.cr1.modify(|_, w| w.pe().set_bit());

                let mut periph = I2c { i2c, sda, scl, _mode: PhantomData };

                // Disable timeout for clock stretch / lockup detection
                // if it was previously set
                periph.disable_scl_timeout();

                return periph;
            }

            pub fn into_slave(self, addr: u8) -> I2c<$I2CX, SDA, SCL, BusSlave> {
                let (i2c, sda, scl) = self.release();

                // Reset the bus, bringing it up in slave mode
                // (sets the TXE (Transmit data register empty) interrupt bit)
                i2c.cr1.modify(|_, w| w.pe().clear_bit());
                while i2c.cr1.read().pe().bit_is_set() {
                    asm::nop();
                } // wait for disabled

                // Set up slave mode: set own address
                // In 7-bit addressing mode for I2C, the low bit of the address
                // must not be set and the upper 7 bits of the u8 are used.
                assert!(addr & 0x01 == 0);
                i2c.oar1.modify(|_, w| unsafe {
                    w.oa1en()
                        .clear_bit()
                        .oa1mode()
                        .clear_bit() // ensure we're in 7-bit mode
                        .oa1_7_1()
                        .bits(addr)
                        .oa1en()
                        .set_bit()
                });

                // Enable ACK generation of slave addresses
                i2c.cr2.modify(|_, w| {
                    w.nack().clear_bit()
                });

                // Enable clock stretch with slave byte control
                i2c.cr1.modify(|_, w| {
                    w
                        .gcen().set_bit()
                        .nostretch().clear_bit()
                        .sbc().set_bit()
                        .pecen().clear_bit() // not using PEC
                });

                // Re-enable bus and interrupts
                i2c.cr1.modify(|_, w| {
                    w
                        .pe()
                        .set_bit()
                        .addrie()
                        .set_bit()
                        .errie()
                        .set_bit()
                });

                let mut periph = I2c { i2c, sda, scl, _mode: PhantomData };

                // Enable timeout for clock stretch / lockup detection
                periph.enable_scl_timeout();

                return periph;
            }

            pub fn release(self) -> ($I2CX, SDA, SCL) {
                (self.i2c, self.sda, self.scl)
            }

            pub fn enable_scl_timeout(&mut self) {
                self.i2c.timeoutr.modify(|_, w| {
                    w
                        // Detect SCL being held low (clock stretched) for
                        // longer than is permitted in a single transaction
                        .texten().set_bit()
                        // Bus lockup detection (SCL held low continuously)
                        .timouten().set_bit()
                });

                // Clear timeout status
                self.i2c.icr.write(|w| w.timoutcf().set_bit());
            }

            pub fn disable_scl_timeout(&mut self) {
                self.i2c.timeoutr.modify(|_, w| {
                    w.texten().clear_bit().timouten().clear_bit()
                });

                // Clear timeout status
                self.i2c.icr.write(|w| w.timoutcf().set_bit());
            }
        }

        impl<SDA, SCL> WriteRead for I2c<$I2CX, SDA, SCL, BusMaster> {
            type Error = Error;

            fn write_read(
                &mut self,
                addr: u8,
                bytes: &[u8],
                buffer: &mut [u8],
            ) -> Result<(), Self::Error> {
                // TODO support transfers of more than 255 bytes
                assert!(bytes.len() < 256 && bytes.len() > 0);
                assert!(buffer.len() < 256 && buffer.len() > 0);

                // Wait for any previous address sequence to end automatically.
                // This could be up to 50% of a bus cycle (ie. up to 0.5/freq)
                while self.i2c.cr2.read().start().bit_is_set() {};

                // Set START and prepare to send `bytes`.
                // The START bit can be set even if the bus is BUSY or
                // I2C is in slave mode.
                self.i2c.cr2.write(|w| unsafe {
                    w
                        // Start transfer
                        .start().set_bit()
                        // Set number of bytes to transfer
                        .nbytes().bits(bytes.len() as u8)
                        // Set address to transfer to/from
                        .sadd().bits((addr << 1) as u16)
                        // 7-bit addressing mode
                        .add10().clear_bit()
                        // Set transfer direction to write
                        .rd_wrn().clear_bit()
                        // Software end mode
                        .autoend().clear_bit()
                });

                for byte in bytes {
                    // Wait until we are allowed to send data
                    // (START has been ACKed or last byte went through)
                    busy_wait!(self.i2c, txis, bit_is_set);

                    // Put byte on the wire
                    self.i2c.txdr.write(|w| unsafe { w.txdata().bits(*byte) });
                }

                // Wait until the write finishes before beginning to read.
                busy_wait!(self.i2c, tc, bit_is_set);

                // reSTART and prepare to receive bytes into `buffer`
                self.i2c.cr2.write(|w| unsafe {
                    w
                        // Start transfer
                        .start().set_bit()
                        // Set number of bytes to transfer
                        .nbytes().bits(buffer.len() as u8)
                        // Set address to transfer to/from
                        .sadd().bits((addr << 1) as u16)
                        // 7-bit addressing mode
                        .add10().clear_bit()
                        // Set transfer direction to read
                        .rd_wrn().set_bit()
                        // Automatic end mode
                        .autoend().set_bit()
                });

                for byte in buffer {
                    // Wait until we have received something
                    busy_wait!(self.i2c, rxne, bit_is_set);

                    *byte = self.i2c.rxdr.read().rxdata().bits();
                }

                // automatic STOP

                Ok(())
            }
        }

        impl<SDA, SCL> Write for I2c<$I2CX, SDA, SCL, BusMaster> {
            type Error = Error;

            fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
                assert!(bytes.len() < 256 && bytes.len() > 0);

                self.i2c.cr2.modify(|_, w| unsafe {
                    w
                        // Start transfer
                        .start().set_bit()
                        // Set number of bytes to transfer
                        .nbytes().bits(bytes.len() as u8)
                        // Set address to transfer to/from
                        .sadd().bits((addr << 1) as u16)
                        // Set transfer direction to write
                        .rd_wrn().clear_bit()
                        // Automatic end mode
                        .autoend().set_bit()
                });

                for byte in bytes {
                    // Wait until we are allowed to send data
                    // (START has been ACKed or last byte when through)
                    busy_wait!(self.i2c, txis, bit_is_set);

                    // Put byte on the wire
                    self.i2c.txdr.write(|w| unsafe { w.txdata().bits(*byte) });
                }

                // automatic STOP

                Ok(())
            }
        }

        impl<SDA, SCL> Read for I2c<$I2CX, SDA, SCL, BusMaster> {
            type Error = Error;

            fn read(&mut self, addr: u8, bytes: &mut [u8]) -> Result<(), Self::Error> {
                // TODO support transfers of more than 255 bytes
                assert!(bytes.len() < 256 && bytes.len() > 0);

                // Wait for any previous address sequence to end automatically.
                // This could be up to 50% of a bus cycle (ie. up to 0.5/freq)
                while self.i2c.cr2.read().start().bit_is_set() {};

                // Set START and prepare to receive bytes into `buffer`.
                // The START bit can be set even if the bus
                // is BUSY or I2C is in slave mode.
                self.i2c.cr2.modify(|_, w| unsafe {
                    w
                        // Start transfer
                        .start().set_bit()
                        // Set number of bytes to transfer
                        .nbytes().bits(bytes.len() as u8)
                        // Set address to transfer to/from
                        .sadd().bits((addr << 1) as u16)
                        // Set transfer direction to read
                        .rd_wrn().set_bit()
                        // automatic end mode
                        .autoend().set_bit()
                });

                for byte in bytes {
                    // Wait until we have received something
                    busy_wait!(self.i2c, rxne, bit_is_set);

                    *byte = self.i2c.rxdr.read().rxdata().bits();
                }

                // automatic STOP

                Ok(())
            }
        }

        impl<SDA, SCL> Write for I2c<$I2CX, SDA, SCL, BusSlave> {
            type Error = Error;

            /// Writes the specified bytes on the I2C bus (transmitting slave
            /// to master receiver).
            ///
            /// This function is intended to be called from an interrupt handler
            /// or polling listener that waits for the I2C peripheral to indicate
            /// an address match. It will program the registers to transfer data
            /// and then clear the address match bit itself: it is important
            /// that the interrupt handler does not try to clear this bit before calling
            /// this function, as there is some additional setup required while the
            /// bus clock is being stretched.
            fn write(&mut self, _addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
                assert!(bytes.len() < 256 && bytes.len() > 0);

                self.i2c.cr2.modify(|_, w| unsafe {
                    w
                        // Start transfer - in slave mode this bit must be clear
                        .start().clear_bit()
                        // Set number of bytes to transfer
                        .nbytes().bits(bytes.len() as u8)
                        // do not use automatic end mode as a slave
                        .autoend().clear_bit()
                        // do not use reload mode when transmitting
                        .reload().clear_bit()
                        // enable address acknowledge
                        .nack().clear_bit()
                });

                // Clear address acknowledge bit to signal peripheral to begin transfer
                self.i2c.icr.write(|w| w.addrcf().set_bit());

                for byte in bytes {
                    // Wait until we are allowed to send data
                    busy_wait!(self.i2c, txis, bit_is_set);

                    // Put byte on the wire
                    self.i2c.txdr.write(|w| unsafe { w.txdata().bits(*byte) });
                }

                // After a successful transaction, master will signal NACK,
                // then START or STOP depending on if another transaction is pending.
                // The I2C peripheral hardware will handle the START/STOP processing,
                // so it remains up to us to check for NACK.
                loop {
                    let isr = self.i2c.isr.read();
                    if isr.nackf().bit_is_set() {
                        // Expected at end of transaction
                        self.i2c.icr.write(|w| w.nackcf().set_bit());
                    } else if isr.stopf().bit_is_set() {
                        // Expected case: a stop condition is set, we're done
                        self.i2c.icr.write(|w| w.stopcf().set_bit().nackcf().set_bit());
                        busy_wait!(self.i2c, stopf, bit_is_clear);
                        flush_txdr!(self.i2c); // Ensure flushed
                        self.i2c.cr2.write(|w| w.nack().set_bit());
                        break;
                    } else if isr.timeout().bit_is_set() {
                        self.i2c.icr.write(|w| w.timoutcf().set_bit());
                        return Err(Error::Timeout);
                    } else if isr.berr().bit_is_set() {
                        self.i2c.icr.write(|w| w.berrcf().set_bit());
                        return Err(Error::BusError);
                    } else if isr.arlo().bit_is_set() {
                        self.i2c.icr.write(|w| w.arlocf().set_bit());
                        return Err(Error::ArbitrationLost);
                    } else {
                        // try again
                        asm::nop()
                    }
                }

                // Clear errors at end of known successful transaction
                self.i2c.icr.write(|w| {
                    w
                        .arlocf().set_bit()
                        .berrcf().set_bit()
                        .nackcf().set_bit()
                        .ovrcf().set_bit()
                        .timoutcf().set_bit()
                });

                Ok(())
            }
        }

        /// Reads bytes from the I2C bus into the provided buffer (receiving slave
        /// from master transmitter).
        ///
        /// This function is intended to be called from an interrupt handler
        /// or polling listener that waits for the I2C peripheral to indicate
        /// an address match. It will program the registers to transfer data
        /// and then clear the address match bit itself: it is important
        /// that the interrupt handler does not try to clear this bit before calling
        /// this function, as there is some additional setup required while the
        /// bus clock is being stretched.
        impl<SDA, SCL> Read for I2c<$I2CX, SDA, SCL, BusSlave> {
            type Error = Error;

            fn read(&mut self, _addr: u8, bytes: &mut [u8]) -> Result<(), Self::Error> {
                // TODO support transfers of more than 255 bytes
                assert!(bytes.len() < 256 && bytes.len() > 0);

                // Wait for any previous address sequence to end automatically.
                // This could be up to 50% of a bus cycle (ie. up to 0.5/freq)
                while self.i2c.cr2.read().start().bit_is_set() {};

                // Prepare to receive bytes into `buffer`.
                self.i2c.cr2.modify(|_, w| unsafe {
                    w
                        // Start transfer - in slave mode this bit must not be set
                        .start().clear_bit()
                        // Set number of bytes to transfer
                        .nbytes().bits(bytes.len() as u8)
                        // do not use automatic end mode as a slave
                        .autoend().clear_bit()
                        // do use reload mode when receiving, for byte acking
                        .reload().set_bit()
                        // enable address acknowledge
                        .nack().clear_bit()
                });

                // Clear address acknowledge bit to signal peripheral to begin transfer
                self.i2c.icr.write(|w| w.addrcf().set_bit());

                for byte in bytes {
                    // Wait until we have received something
                    busy_wait!(self.i2c, rxne, bit_is_set);

                    *byte = self.i2c.rxdr.read().rxdata().bits();
                }

                // User is responsible for
                // inspecting bytes and NACKing (and waiting for NACK clear by hw) if required
                // Followed by waiting for stop condition ending transfer.

                Ok(())
            }
        }

        // Additional utility functions only used when I2C bus is in slave mode
        impl<SDA, SCL> I2c<$I2CX, SDA, SCL, BusSlave> {

            pub fn handle_slave_error(&mut self) {
                // Prepare to NACK the current xfer
                self.i2c.cr2.modify(|_, w| w.nack().set_bit());

                // Clear the address match,
                // otherwise no HAL read or write occurs to clear this
                if self.i2c.isr.read().addr().bit_is_set() {
                    self.i2c.icr.write(|w| w.addrcf().set_bit());
                }

                // Flush the transmit data register
                flush_txdr!(self.i2c);

                // Flush the receive data register
                flush_rxdr!(self.i2c);
            }

            pub fn status(&mut self)
            -> Result<stm32g0::R<u32, stm32g0::Reg<u32, stm32::i2c1::_ISR>>, Error> {
                let isr = self.i2c.isr.read();

                // Not an error condition:
                // All transactions begin with a START and are terminated by a STOP
                if isr.stopf().bit_is_set() {
                    // Clear interrupt
                    self.i2c.icr.write(|w| w.stopcf().set_bit());
                }

                // Detect error states, otherwise return the contents
                // of the status register
                if isr.berr().bit_is_set() {
                    self.i2c.icr.write(|w| w.berrcf().set_bit());
                    Err(Error::BusError)
                } else if isr.arlo().bit_is_set() {
                    self.i2c.icr.write(|w| w.arlocf().set_bit());
                    Err(Error::ArbitrationLost)
                } else if isr.nackf().bit_is_set() {
                    self.i2c.icr.write(|w| w.nackcf().set_bit());
                    Err(Error::Nack)
                } else if isr.ovr().bit_is_set() {
                    self.i2c.icr.write(|w| w.ovrcf().set_bit());
                    Err(Error::Overrun)
                } else {
                    Ok(isr)
                }
            }

            // Sends an ACK signal at the end of a successful read transfer
            // and finishes the transfer.
            //
            // This functionality is separate from the basic Read trait
            // implementation so that in I2C modes (e.g, SMBUS and other
            // protocols) that want to be able to ACK/NACK the data received
            // can now use the return value from i2c.read() and decide what
            // to do about it.
            pub fn end_read_transaction(&mut self) -> Result<(), Error> {

                // Nonzero nbytes value to send configured ACK or NACK
                self.i2c.cr2.modify(|_, w| unsafe {
                    w.nbytes().bits(1)
                });

                // Wait for NACK to happen if it needs to
                while self.i2c.cr2.read().nack().bit_is_set() {}

                // Looking for STOP to signal end of transfer
                loop {
                    let isr = self.i2c.isr.read();
                    if isr.stopf().bit_is_set() {
                        // Expected case: a stop condition is set, we're done.
                        // Clear the stop bit
                        self.i2c.icr.write(|w| w.stopcf().set_bit());
                        return Ok(());
                    } else if isr.rxne().bit_is_set() {
                        // Error, transmitter is sending too much data
                        rprintln!("Error waiting for stop: rxne");
                        return Err(Error::Nack);
                    } else if isr.timeout().bit_is_set() {
                        self.i2c.icr.write(|w| w.timoutcf().set_bit());
                        rprintln!("Error waiting for stop: Timeout");
                        return Err(Error::Timeout);
                    } else if isr.berr().bit_is_set() {
                        self.i2c.icr.write(|w| w.berrcf().set_bit());
                        rprintln!("Error waiting for stop: BusError");
                        return Err(Error::BusError);
                    } else if isr.arlo().bit_is_set() {
                        self.i2c.icr.write(|w| w.arlocf().set_bit());
                        rprintln!("Error waiting for stop: ArbitrationLost");
                        return Err(Error::ArbitrationLost);
                    } else {
                        // try again
                        asm::nop()
                    }
                }
            }
        }
    };
}

i2c!(
    I2C1,
    i2c1,
    i2c1en,
    i2c1rst,
    sda: [
        PA10<Output<OpenDrain>>,
        PB7<Output<OpenDrain>>,
        PB9<Output<OpenDrain>>,
    ],
    scl: [
        PA9<Output<OpenDrain>>,
        PB6<Output<OpenDrain>>,
        PB8<Output<OpenDrain>>,
    ],
);

i2c!(
    I2C2,
    i2c2,
    i2c2en,
    i2c2rst,
    sda: [
        PA12<Output<OpenDrain>>,
        PB11<Output<OpenDrain>>,
        PB14<Output<OpenDrain>>,
    ],
    scl: [
        PA11<Output<OpenDrain>>,
        PB10<Output<OpenDrain>>,
        PB13<Output<OpenDrain>>,
    ],
);
