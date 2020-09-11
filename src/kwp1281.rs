//! Protocol implementation for KWP1281

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::sleep;
use std::time::Duration;

use log::{debug, info};

use crate::dtc::*;
use crate::error::*;
use crate::kline::*;
use crate::misc::*;

/// Address to talk to for initialization (0x01 = ECU)
const INIT_ADDRESS: u8 = 0x01;

/// Enum of KWP1281 block types.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum Kwp1281BlockType {
    ClearDtcs,
    Quit,
    GetDtcs,
    Ack,
    ReadData,
    DataReply,
    Ascii,
    Other(u8),
}

impl From<u8> for Kwp1281BlockType {
    fn from(block_type: u8) -> Kwp1281BlockType {
        match block_type {
            0x05 => Self::ClearDtcs,
            0x06 => Self::Quit,
            0x07 => Self::GetDtcs,
            0x09 => Self::Ack,
            0x29 => Self::ReadData,
            0xe7 => Self::DataReply,
            0xf6 => Self::Ascii,
            x => Self::Other(x),
        }
    }
}

impl Into<u8> for Kwp1281BlockType {
    fn into(self) -> u8 {
        match self {
            Self::ClearDtcs => 0x05,
            Self::Quit => 0x06,
            Self::GetDtcs => 0x07,
            Self::Ack => 0x09,
            Self::ReadData => 0x29,
            Self::DataReply => 0xe7,
            Self::Ascii => 0xf6,
            Self::Other(x) => x,
        }
    }
}

impl std::fmt::LowerHex for Kwp1281BlockType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let val: u8 = (*self).into();
        std::fmt::LowerHex::fmt(&val, f)
    }
}

/// KWP1281 block type
#[derive(Clone, Debug)]
struct Kwp1281Block {
    pub block_type: Kwp1281BlockType,
    pub data: Vec<u8>,
}

/// Protocol for talking to the vehicle's K line via KWP1281.
pub struct Kwp1281 {
    kline: KLine,
    block_counter: u8,
    /// Identifying data returned by the ECU after initialization
    pub ecu_data: Vec<u8>,
}

impl Kwp1281 {
    /**
     * Initialize the KWP1281 protocol via the UART1 bus. Because of the
     * extremely low baud rate used for initialization (5), the initialization
     * will have to be done manually in GPIO mode, while we can use the UART
     * driver for the actual communication.
     *
     * The initialization sends a target address. This method always sends 0x01
     * at the moment, which indicates wanting to talk to the ECU. Other
     * addresses are possible, for example for talking to airbag or ABS
     * controllers.
     *
     * It will be attempted to deduce the baud rate automatically using a sync
     * byte if no specific baud rate is given.
     */
    pub fn init(baud_rate: Option<u64>) -> Result<Self, Error> {
        let kline = KLine::init(baud_rate, INIT_ADDRESS)?;

        let mut kwp = Self {
            kline,
            block_counter: 0,
            ecu_data: Vec::new(),
        };

        // Because of the UART setup time, we might miss the first byte, 0x01
        // and only receive the second one, 0x8a. Because of this we have to
        // complement this one manually.
        let mut byte = kwp.kline.read_byte(false)?;
        if byte == 0x01 {
            byte = kwp.kline.read_byte(false)?;
        }

        if byte != 0x8a {
            return Err(Error::new("Unexpected key byte."));
        }

        kwp.kline.write_byte(0xff - byte, false)?;

        // Initialization is over, ECU will now send some data about itself.
        // We will have to wait for that to finish while ACKing blocks.
        let mut ecu_done_yapping = false;
        for _i in 0..10 {
            let block = kwp.read_block()?;
            if block.block_type == Kwp1281BlockType::Ack {
                ecu_done_yapping = true;
                break;
            }

            kwp.ecu_data.extend(&block.data);

            kwp.write_ack()?;
        }

        if !ecu_done_yapping {
            return Err(Error::new("Timeout waiting for ECU to finish"));
        }

        // ECU is done. We can now send our own requests.

        Ok(kwp)
    }

    /**
     * Run a crude car simulator using the given baud rate. This simulator can
     * be used for testing the logic level conversion hardware using two BBBs.
     */
    pub fn run_simulator(baud_rate: u64) -> Result<(), Error> {
        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();

        ctrlc::set_handler(move || {
            r.store(false, Ordering::SeqCst);
        })
        .unwrap();

        while running.load(Ordering::SeqCst) {
            info!("Waiting for connections...");

            {
                let (_tx, rx) = KLine::initialize_gpio()?;

                while running.load(Ordering::SeqCst) {
                    if let Err(_) = busy_wait_until(&rx, 0, 500_000 as u64) {
                        continue;
                    }

                    break;
                }

                if !running.load(Ordering::SeqCst) {
                    break;
                }

                // manually read init address
                let mut bits: Vec<u8> = vec![0; 7];
                for i in 0..7 {
                    sleep(Duration::from_millis(200));
                    bits[i] = rx.get_value()?;
                }

                // wait for last bit, parity bit, and stop bit to finish
                sleep(Duration::from_millis(200 * 3));

                info!("Connection received: {:?}", bits);
            }

            let port = KLine::initialize_uart(baud_rate)?;
            let mut kline = KLine { port, baud_rate };

            kline.write_byte(0x55, false)?;
            kline.write_byte(0x01, false)?;
            kline.write_byte(0x8a, true)?;

            let mut kwp = Kwp1281 {
                kline,
                block_counter: 0,
                ecu_data: Vec::new(),
            };

            // write some ascii blocks
            for _i in 0..4 {
                kwp.write_block(Kwp1281Block {
                    block_type: Kwp1281BlockType::Ascii,
                    data: String::from("foobar").into_bytes(),
                })?;
                let ack = kwp.read_block()?;
                assert_eq!(ack.block_type, Kwp1281BlockType::Ack);
            }

            kwp.write_ack()?;

            while running.load(Ordering::SeqCst) {
                let block = kwp.read_block()?;

                match block.block_type {
                    Kwp1281BlockType::GetDtcs => {
                        kwp.write_block(Kwp1281Block {
                            block_type: Kwp1281BlockType::Other(0xfc),
                            data: vec![0x46, 0x3a, 0x23, 0x02, 0x0a, 0x1e],
                        })?;
                    }
                    _ => {
                        kwp.write_ack()?;
                    }
                }
            }
        }

        Ok(())
    }

    /**
     * Write an ACK block to the K line.
     */
    fn write_ack(&mut self) -> Result<(), Error> {
        let block = Kwp1281Block {
            block_type: Kwp1281BlockType::Ack,
            data: Vec::new(),
        };

        self.write_block(block)
    }

    /**
     * Write a given block to the K line. Does not wait for acknowledgement.
     */
    fn write_block(&mut self, block: Kwp1281Block) -> Result<(), Error> {
        debug!("SEND {:02x} {:02x?}", block.block_type, &block.data);

        self.block_counter = self.block_counter.wrapping_add(1);
        let length = block.data.len() + 3;

        self.kline.write_byte(length as u8, true)?;
        self.kline.write_byte(self.block_counter, true)?;
        self.kline.write_byte(block.block_type.into(), true)?;

        for b in block.data {
            self.kline.write_byte(b, true)?;
        }

        self.kline.write_byte(0x03, false)?;

        Ok(())
    }

    /**
     * Read a block from the K line. Does not send acknowledgement.
     */
    fn read_block(&mut self) -> Result<Kwp1281Block, Error> {
        let length = self.kline.read_byte(true)? - 3;

        let mut data: Vec<u8> = Vec::with_capacity(length as usize);

        self.block_counter = self.kline.read_byte(true)?;
        let block_type = self.kline.read_byte(true)?;

        for _i in 0..length {
            data.push(self.kline.read_byte(true)?);
        }

        self.kline.read_byte(false)?;

        debug!("RECV {:02x} {:02x?}", block_type, data);

        Ok(Kwp1281Block {
            block_type: block_type.into(),
            data,
        })
    }
}

impl Diagnose for Kwp1281 {
    fn read_dtcs(&mut self, _pending: bool) -> Result<Vec<DiagnosticTroubleCode>, Error> {
        let mut dtcs = Vec::new();

        self.write_block(Kwp1281Block {
            block_type: Kwp1281BlockType::GetDtcs,
            data: Vec::new(),
        })?;

        for i in 0..10 {
            let block = self.read_block()?;
            if block.block_type == Kwp1281BlockType::Ack {
                return Ok(dtcs);
            }

            if i == 0 && block.data == &[0xff, 0xff, 0x88] {
                return Ok(dtcs);
            }

            for chunk in block.data.chunks(3) {
                let code = ((chunk[0] as u16) << 8) + (chunk[1] as u16);
                let status = chunk[2];
                dtcs.push(DiagnosticTroubleCode::Oem(code, status));
            }

            self.write_ack()?;
        }

        return Err(Error::new("Timeout waiting for DTCs to finish."));
    }

    fn clear_dtcs(&mut self) -> Result<(), Error> {
        self.write_block(Kwp1281Block {
            block_type: Kwp1281BlockType::ClearDtcs,
            data: Vec::new(),
        })?;

        let response = self.read_block()?;

        if response.block_type == Kwp1281BlockType::Ack {
            Ok(())
        } else {
            Err(Error::new("Unexpected response to ClearDtcs command."))
        }
    }

    fn read_data(&mut self, pid: u8, freeze_frame: bool) -> Result<Vec<u8>, Error> {
        if freeze_frame {
            return Err(Error::new("Freeze frames are not supported by KWP1281."));
        }

        self.write_block(Kwp1281Block {
            block_type: Kwp1281BlockType::ReadData,
            data: vec![pid],
        })?;

        let response = self.read_block()?;

        if response.block_type == Kwp1281BlockType::DataReply {
            Ok(response.data)
        } else {
            Err(Error::new("Unexpected response to ReadData command."))
        }
    }

    fn read_data_formatted(&mut self, pid: u8, freeze_frame: bool) -> Result<String, Error> {
        let data = self.read_data(pid, freeze_frame)?;

        // Special case: all the data is one string
        if data[0] == 0x3f {
            return Ok(String::from_utf8_lossy(&data[1..]).to_string());
        }

        // Otherwise, all data contains 4 groups of 1 format identifier,
        // and 2 data bytes. Some may be empty (0x25, 0x00, 0x00).
        let mut output = String::from("");
        for chunk in data.chunks(3) {
            if chunk == [0x25, 0x00, 0x00] {
                continue;
            }

            let format = chunk[0];
            let a = chunk[1];
            let b = chunk[2];

            output.push_str(&match format {
                0x01 => format!("{:6.1} rpm ", a as f32 * b as f32 * 0.2),
                0x02 => format!("{:7.3} % ", a as f32 * b as f32 * 0.002),
                0x03 => format!("{:7.3} deg ", a as f32 * b as f32 * 0.002),
                0x05 => format!("{:5.1} C ", a as f32 * (b as f32 - 100.0) * 0.1),
                0x06 | 0x15 => format!("{:6.3} V ", a as f32 * b as f32 * 0.001),
                0x07 => format!("{:6.2} km/h ", a as f32 * b as f32 * 0.01),
                0x0f => format!("{:7.2} ms ", a as f32 * b as f32 * 0.01),
                0x12 => format!("{:7.2} mbar ", a as f32 * b as f32 * 0.04),
                0x14 => format!("{:8.3} % ", a as f32 * (b as f32 - 128.0) / 128.0),
                0x19 => format!("{:6.3} g/s ", (a as f32 / 128.0) + (b as f32 * 1.1421)),
                0x21 => format!(
                    "{:7.3} % ",
                    if a == 0 {
                        b as f32 * 100.0
                    } else {
                        (b as f32 * 100.0) / a as f32
                    }
                ),
                0x24 => format!("{:6} km ", (((a as u32) << 8) + b as u32) * 10),
                0x2f => format!("{:4} ms ", (b as i32 - 128) * a as i32),
                0x31 => format!("{:7.2} mg/h ", (b as f32 / 4.0) * a as f32 * 0.1),
                0x34 => format!("{:7.2} Nm ", b as f32 * 0.002 * a as f32 - a as f32),
                0x36 => format!("{:5} ", ((a as u16) << 8) + b as u16),
                0x42 => format!("{:6.3} V ", a as f32 * b as f32 / 511.12),
                _ => format!("{:02x?} ", chunk),
            })
        }

        if output == "" {
            output = "No data".to_string()
        }

        Ok(output.trim_end().to_string())
    }
}
