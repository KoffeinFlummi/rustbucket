//! Protocol implementation for KWP2000

use std::num::Wrapping;

use log::debug;

use crate::error::*;
use crate::kline::*;
use crate::obd2::*;

/// Address to talk to for initialization (0x01 = ECU)
const INIT_ADDRESS: u8 = 0x01;

/// Protocol for talking to the vehicle's K line via KWP2000.
pub struct Kwp2000 {
    kline: KLine,
}

impl Kwp2000 {
    pub fn init(baud_rate: Option<u64>) -> Result<Self, Error> {
        // TODO: fast init
        // TODO: write address to L line as well

        let kline = KLine::init(baud_rate, INIT_ADDRESS)?;

        let mut kwp = Self { kline };

        let kb1 = kwp.kline.read_byte(false)?;
        let kb2 = kwp.kline.read_byte(false)?;

        // The key bytes are actually sent as 7O1, so we'll have to remove the
        // parity bit manually
        let protocol_id = (((kb2 & 0x7f) as u16) << 7) + ((kb1 & 0x7f) as u16);
        if protocol_id != 2031 {
            return Err(Error::new("Unexpected protocol identifier."));
        }

        // parity bit?
        kwp.kline.write_byte(0xff - kb2, false)?;

        // inverted address
        if kwp.kline.read_byte(false)? != (0xff - INIT_ADDRESS) {
            return Err(Error::new("Unexpected address complement."));
        }

        // Start diagnostic session. The 0x89 parameter might be manufacturer
        // specific and will have to be modified.
        kwp.obd_query(0x10, &[0x89])?;

        Ok(kwp)
    }

    fn write_block(&mut self, data: &[u8]) -> Result<(), Error> {
        let mut msg: Vec<u8> = vec![0x80 + data.len() as u8, 0x10, 0xf1];
        msg.extend(data);

        let crc: Wrapping<u8> = msg.iter().map(|x| Wrapping(*x)).sum();
        msg.push(crc.0);

        debug!("SEND {:02x?}", &msg);

        for byte in msg {
            self.kline.write_byte(byte, false)?;
        }

        Ok(())
    }

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

impl Obd2Protocol for Kwp2000 {
    fn obd_query(&mut self, service: u8, args: &[u8]) -> Result<Vec<u8>, Error> {
        let mut msg: Vec<u8> = vec![service];
        msg.extend(args);

        self.write_block(&msg)?;

        let response = self.read_block()?;

        if response[0] == 0x7f {
            return Err(Error::new(
                "ECU returned error code. Query may not be supported.",
            ));
        }

        if response[0] != service + 0x40 {
            return Err(Error::new("Service identifier of response did not match."));
        }

        if &response[1..(1 + args.len())] != args {
            return Err(Error::new("Arguments/PIDs did not match."));
        }

        Ok(response[(1 + args.len())..].to_vec())
    }
}
