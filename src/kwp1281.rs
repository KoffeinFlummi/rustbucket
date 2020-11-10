//! Protocol implementation for KWP1281

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::sleep;
use std::time::Duration;

use log::{debug, info};

use crate::diagnose::*;
use crate::error::*;
use crate::kline::*;
use crate::misc::*;

/// Enum of KWP1281 block types (also referred to as block titles online).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum Kwp1281BlockType {
    ClearDtcs,
    Quit,
    GetDtcs,
    Ack,
    ReadData,
    DataReply,
    Ascii,
    ReadAdaptation,
    TestAdaptation,
    WriteAdaptation,
    AdaptationReply,
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
            0x21 => Self::ReadAdaptation,
            0x22 => Self::TestAdaptation,
            0x2a => Self::WriteAdaptation,
            0xe6 => Self::AdaptationReply,
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
            Self::ReadAdaptation => 0x21,
            Self::TestAdaptation => 0x22,
            Self::WriteAdaptation => 0x2a,
            Self::AdaptationReply => 0xe6,
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
    pub fn init(target_address: u8, baud_rate: Option<u64>) -> Result<Self, Error> {
        let kline = KLine::init(target_address, baud_rate)?;

        let mut kwp = Self {
            kline,
            block_counter: 0,
            ecu_data: Vec::new(),
        };

        // Because of the UART setup time, we might miss the first byte, 0x01
        // and only receive the second one, 0x8a. Because of this we have to
        // complement this one manually.
        let kb1 = kwp.kline.read_byte(false)?;
        let kb2 = kwp.kline.read_byte(false)?;
        debug!("Key Bytes: {:02x?} {:02x?}", kb1, kb2);

        if kb1 != 0x01 || kb2 != 0x8a {
            return Err(Error::new("Unexpected key byte."));
        }

        kwp.kline.write_byte(0xff - kb2, false)?;

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

    /**
     * Read adaptation value.
     */
    pub fn read_adaptation(&mut self, pid: u8) -> Result<Vec<u8>, Error> {
        self.write_block(Kwp1281Block {
            block_type: Kwp1281BlockType::ReadAdaptation,
            data: vec![pid],
        })?;

        let response = self.read_block()?;

        if response.block_type == Kwp1281BlockType::AdaptationReply {
            Ok(response.data[1..].to_vec())
        } else {
            Err(Error::new("Unexpected response to ReadAdaptation command."))
        }
    }

    /**
     * Write adaptation value.
     */
    pub fn write_adaptation(
        &mut self,
        pid: u8,
        value: &[u8; 2],
        test: bool,
    ) -> Result<Vec<u8>, Error> {
        let block_type = if test {
            Kwp1281BlockType::TestAdaptation
        } else {
            Kwp1281BlockType::WriteAdaptation
        };

        let mut data: Vec<u8> = vec![pid];
        data.extend(value);

        if !test {
            // three bytes of workshop code.
            // TODO: make this a command line option
            data.extend(vec![0x00, 0x00, 0x00]);
        }

        self.write_block(Kwp1281Block { block_type, data })?;

        let response = self.read_block()?;

        if response.block_type == Kwp1281BlockType::AdaptationReply {
            Ok(response.data)
        } else {
            Err(Error::new(
                "Unexpected response to WriteAdaptation command.",
            ))
        }
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

    fn read_data(&mut self, pid: u8, freeze_frame: bool) -> Result<DiagnosticData, Error> {
        if freeze_frame {
            return Err(Error::new("Freeze frames are not supported by KWP1281."));
        }

        self.write_block(Kwp1281Block {
            block_type: Kwp1281BlockType::ReadData,
            data: vec![pid],
        })?;

        let response = self.read_block()?;

        if response.block_type == Kwp1281BlockType::DataReply {
            Ok(DiagnosticData::from_kwp1281_data(pid, response.data))
        } else {
            Err(Error::new("Unexpected response to ReadData command."))
        }
    }
}
