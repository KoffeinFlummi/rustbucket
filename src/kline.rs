//! Physical layer implementation for the K line

use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::sleep;
use std::time::{Duration, SystemTime};

use gpio_cdev::{Chip, LineHandle, LineRequestFlags};
use log::{debug, info, warn};
use serial::core::SerialDevice;
use serial::core::SerialPortSettings;

use crate::error::*;
use crate::misc::*;

/// Baud rate to use for initialization
const INIT_BAUD_RATE: u64 = 5;

/// Delay before writing byte to K line
const WRITE_DELAY_MICROS: u64 = 5000;

/// General physical layer implementation for various K line protocols.
pub struct KLine {
    /// UART port used for communication after initialization
    pub port: serial::unix::TTYPort,
    /// Given or determined baud rate
    pub baud_rate: u64,
}

impl KLine {
    /**
     * Initialize the K line on the UART1 bus by addressing the given ECU.
     *
     * If the baud rate is not given, it will be determined automatically from
     * the sync byte. Because of the extremely low baud rate used for the
     * initialization, the initialization will have to be done in GPIO mode.
     *
     * Fast init as used by KWP2000 is not implemented at the moment, and
     * neither is the initialization on the L line in parallel, as is
     * used optionally by ISO 9141, and KWP2000.
     */
    pub fn init(init_address: u8, baud_rate: Option<u64>) -> Result<Self, Error> {
        // Initialize communication manually, in GPIO mode
        let (tx, rx) = Self::initialize_gpio()?;

        // Guarantee that the K line is high for a while before we pull it down
        sleep(Duration::from_millis(300));

        // Write 0x01 using 7O1 UART, at 5 baud
        Self::write_byte_software(&tx, init_address, 7, true, INIT_BAUD_RATE)?;

        // Manually read sync byte (0x55) to figure out main baud rate
        busy_wait_until(&rx, 0, 500_000 as u64)?;
        let reference = SystemTime::now();
        busy_wait_until(&rx, 1, 500_000 as u64)?;

        for _i in 0..4 {
            busy_wait_until(&rx, 0, 500_000 as u64)?;
            busy_wait_until(&rx, 1, 500_000 as u64)?;
        }

        let measured = 1_000_000 / (reference.elapsed().unwrap().as_micros() / 9);
        let baud = baud_rate.unwrap_or(Self::nearest_baud_rate(measured as u64));

        if baud_rate.is_none() {
            info!("Measured baud rate: {}", measured);
            info!("Using nearest known baud rate: {}", baud);
        }

        // Switch to proper UART (8N1) for remainder of communication
        Ok(Self {
            port: Self::initialize_uart(baud)?,
            baud_rate: baud,
        })
    }

    /**
     * Continuously transmit or receive data via the UART1 bus using the given
     * baud rate. This is useful for low-level hardware testing, or hooking up
     * an oscilloscope.
     */
    pub fn test_hardware(tx: bool, baud_rate: u64) -> Result<(), Error> {
        let mut port = Self::initialize_uart(baud_rate)?;

        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();

        ctrlc::set_handler(move || {
            r.store(false, Ordering::SeqCst);
        })
        .unwrap();

        let mut buffer = vec![0];
        loop {
            if !running.load(Ordering::SeqCst) {
                break;
            }

            if tx {
                port.write(&[0x55])?;
                sleep(Duration::from_micros(2000));
            } else {
                if let Ok(_) = port.read_exact(&mut buffer) {
                    debug!("{:?}", buffer);
                } else {
                    warn!("failed");
                }
            }
        }

        Ok(())
    }

    /**
     * Determine the known baud rate closest to the one that was measured.
     */
    fn nearest_baud_rate(measured: u64) -> u64 {
        let mut known: Vec<u64> = vec![9600, 10400];
        known.sort_by_key(|b| ((*b as i64) - (measured as i64)).abs());
        known[0]
    }

    /**
     * Set the UART1 tx/rx pins' pin multiplexer state to GPIO, and initialize
     * the LineHandles.
     */
    pub fn initialize_gpio() -> Result<(LineHandle, LineHandle), Error> {
        set_pin_mode(9, 24, PinMode::Gpio)?;
        set_pin_mode(9, 26, PinMode::Gpio)?;

        let mut chip = Chip::new("/dev/gpiochip0")?;
        let tx = chip
            .get_line(15)?
            .request(LineRequestFlags::OUTPUT, 1, "k-tx")?;
        let rx = chip
            .get_line(14)?
            .request(LineRequestFlags::INPUT, 0, "k-rx")?;

        Ok((tx, rx))
    }

    /**
     * Set the UART1 tx/rx pins' pin multiplexer state to UART, and initialize
     * the UART bus with the given baud rate.
     */
    pub fn initialize_uart(baud_rate: u64) -> Result<serial::unix::TTYPort, Error> {
        set_pin_mode(9, 24, PinMode::Uart)?;
        set_pin_mode(9, 26, PinMode::Uart)?;

        let mut port = serial::open("/dev/ttyO1")?;
        let mut settings = port.read_settings()?;
        settings.set_baud_rate(serial::BaudOther(baud_rate as usize))?;
        settings.set_char_size(serial::Bits8);
        settings.set_parity(serial::ParityNone);
        settings.set_stop_bits(serial::Stop1);
        settings.set_flow_control(serial::FlowNone);
        port.write_settings(&settings)?;
        port.set_timeout(Duration::from_millis(1000))?;

        Ok(port)
    }

    /**
     * Write byte to serial bus using software UART in GPIO mode (needed for
     * extremely low baud rate used during init). Always use start bit and 1
     * stop bit, odd  parity bit can be used (only during initialization),
     * char size is 7 for initialization, 8 afterwards.
     */
    fn write_byte_software(
        tx: &LineHandle,
        value: u8,
        char_size: u8,
        parity: bool,
        baud: u64,
    ) -> Result<(), Error> {
        let start = SystemTime::now();

        let bits: Vec<u8> = (0..char_size).map(|i| (value & (1 << i)) >> i).collect();
        let delay: u64 = 1_000_000 / baud;

        tx.set_value(0)?;
        busy_wait(start, delay - 50);

        for (i, b) in bits.iter().enumerate() {
            tx.set_value(*b)?;
            busy_wait(start, delay * (i + 2) as u64);
        }

        if parity {
            tx.set_value(1 - (bits.iter().sum::<u8>() % 2))?;
            busy_wait(start, delay * (bits.len() + 2) as u64);
        }

        tx.set_value(1)?;
        busy_wait(start, delay * ((bits.len() as u64) + 2 + (parity as u64)));

        Ok(())
    }

    /**
     * Read a single byte, optionally send a complement byte.
     */
    pub fn read_byte(&mut self, complement: bool) -> Result<u8, Error> {
        let mut buffer: Vec<u8> = vec![0];
        self.port.read_exact(&mut buffer)?;

        if complement {
            self.write_byte(0xff - buffer[0], false)?;
        }

        Ok(buffer[0])
    }

    /**
     * Write a single byte, optionally expect a complement byte.
     */
    pub fn write_byte(&mut self, value: u8, complement: bool) -> Result<(), Error> {
        // We need to add a small delay before writing, otherwise communication
        // might not work. Presumably the ECU is not fast enough.
        busy_wait(SystemTime::now(), WRITE_DELAY_MICROS);

        let mut buffer: Vec<u8> = vec![0];

        let start = SystemTime::now();
        self.port.write(&[value])?;

        // read back value
        self.port.read_exact(&mut buffer)?;
        busy_wait(start, 10 * 1_000_000 / self.baud_rate);

        if complement {
            self.port.read_exact(&mut buffer)?;
            if buffer[0] != (0xff - value) {
                warn!("Invalid complement received.");
            }
        }

        Ok(())
    }
}
