//! Protocol implementation for KWP2000 / ISO 14230

use std::num::Wrapping;
use std::time::SystemTime;

use log::debug;

use crate::diagnose::*;
use crate::error::*;
use crate::kline::*;
use crate::misc::*;

/// Delay between ECU response and next tester request (P4)
const BLOCK_DELAY_MICROS: u64 = 60_000;

/// Protocol for talking to the vehicle's K line via KWP2000.
pub struct Kwp2000 {
    kline: KLine,
    physical_address: u8,
    block_delay: u64,
}

impl Kwp2000 {
    /**
     * Initialize the KWP2000 protocol via the UART1 bus.
     *
     * It will be attempted to deduce the baud rate automatically using a sync
     * byte if no specific baud rate is given.
     *
     * Not implemented at the moment is fast init, and neither is
     * initialization of the L line.
     */
    pub fn init(target_address: u8, baud_rate: Option<u64>, physical_address: Option<u8>) -> Result<Self, Error> {
        // TODO: fast init?
        // TODO: write address to L line as well

        // The physical address used for addressing KWP2000 blocks differ from
        // the 5-baud init addresses. See appendix B of ISO 14230-2.
        let physical_address = physical_address.unwrap_or(match target_address {
            0x01 => 0x10, // ECU
            0x02 => 0x18, // transmission
            0x03 => 0x28, // brakes
            _ => {
                return Err(Error::new("Could not guess phsyical address, please set one manually."));
            }
        });

        let kline = KLine::init(target_address, baud_rate)?;
        let mut kwp = Self {
            kline,
            physical_address,
            block_delay: BLOCK_DELAY_MICROS,
        };

        let kb1 = kwp.kline.read_byte(false)?;
        let kb2 = kwp.kline.read_byte(false)?;
        debug!("Key Bytes: {:02x?} {:02x?}", kb1, kb2);

        if kb2 != 0x8f {
            return Err(Error::new("Unexpected protocol identifier."));
        }

        // TODO: check/parse KB2

        kwp.kline.write_byte(0xff - kb2, false)?;

        // inverted address
        let byte = kwp.kline.read_byte(false)?;
        debug!("Address Complement: {:02x?}", byte);
        if (byte & 0x7f) != (0x7f - target_address) {
            return Err(Error::new("Unexpected address complement."));
        }

        // TODO: read timing parameters?

        // Start diagnostic session. The 0x89 parameter might be manufacturer
        // specific and will have to be modified.
        kwp.write_block(0x80, physical_address, &[0x10, 0x89])?;
        kwp.read_block()?; // TODO: check this

        Ok(kwp)
    }

    /**
     * Write a data block to the K line via KWP2000.
     *
     * Data length is added to format byte, which should be 0x80, or 0xc0 for
     * diagnostic requests. Target address should be 0x33 for diagnostic
     * requests. Source address is always 0xf1 as per ISO 15031-5.
     */
    fn write_block(&mut self, format: u8, target: u8, data: &[u8]) -> Result<(), Error> {
        busy_wait(SystemTime::now(), self.block_delay);

        let mut msg: Vec<u8> = vec![format + data.len() as u8, target, 0xf1];
        msg.extend(data);

        let crc: Wrapping<u8> = msg.iter().map(|x| Wrapping(*x)).sum();
        msg.push(crc.0);

        debug!("SEND {:02x?}", &msg);

        for byte in msg {
            self.kline.write_byte(byte, false)?;
        }

        Ok(())
    }

    /**
     * Read a data block from the K line via KWP2000, returning just the data
     * bytes, without format and source/target addresses.
     */
    fn read_block(&mut self) -> Result<Vec<u8>, Error> {
        let header = self.kline.read_byte(false)?;

        let mut crc: Wrapping<u8> = Wrapping(header);
        let length = header as usize & 0x3f;

        let mut msg = Vec::with_capacity(length + 4);
        msg.push(header);

        for i in 0..length + 3 {
            let byte = self.kline.read_byte(false)?;
            msg.push(byte);

            if i < length + 2 {
                crc += Wrapping(byte);
            }
        }

        debug!("RECV {:02x?}", &msg);

        if crc.0 != msg[msg.len() - 1] {
            return Err(Error::new("CRC Error"));
        }

        Ok(msg[3..(length + 3)].into())
    }
}

impl Diagnose for Kwp2000 {
    fn read_dtcs(&mut self, _pending: bool) -> Result<Vec<DiagnosticTroubleCode>, Error> {
        self.write_block(0x80, self.physical_address, &[0x18, 0x02, 0xff, 0x00])?;

        let data = self.read_block()?;

        if data[0] != 0x58 {
            return Err(Error::new("Unexpected response to readDiagnosticTroubleCodesByStatus command."));
        }

        let mut dtcs = Vec::with_capacity(data[1] as usize);
        for chunk in data[2..].chunks(3) {
            let code = ((chunk[0] as u16) << 8) + (chunk[1] as u16);
            let status = chunk[2];
            dtcs.push(DiagnosticTroubleCode::Oem(code, status));
        }

        return Ok(dtcs);
    }

    fn clear_dtcs(&mut self) -> Result<(), Error> {
        self.write_block(0x80, self.physical_address, &[0x14, 0xff, 0x00])?;

        let mut data = self.read_block()?;

        // For some reason, the Mk60 ESP controller I tested with first returns
        // an error, and then a positive reply immediately afterwards.
        // http://nefariousmotorsports.com/forum/index.php?topic=3946.0title=
        if data[0] == 0x7f {
            // If ECU returns another block, use that. If read times out, use
            // the original data for error code.
            if let Ok(d) = self.read_block() {
                data = d;
            }
        }

        if data[0] == 0x54 {
            Ok(())
        } else {
            Err(Error::new("Unexpected response to clearDiagnosticInformation command."))
        }
    }

    fn read_data(&mut self, _pid: u8, _freeze_frame: bool) -> Result<DiagnosticData, Error> {
        todo!();
    }
}
