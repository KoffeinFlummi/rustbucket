//! General diagnosis implementation for all OBD2 protocols

use crate::dtc::*;
use crate::error::*;

pub trait Obd2Protocol {
    /**
     * Sends an OBD query over the bus and returns the response, abstracting
     * away protocol-specific issues such as CAN multi-frame messages.
     *
     * The response data should be stripped of the service response identifier
     * (service + 0x40) and the args/PIDs, leaving just the value.
     */
    fn obd_query(&mut self, service: u8, args: &[u8]) -> Result<Vec<u8>, Error>;
}

impl<T: Obd2Protocol> Diagnose for T {
    fn read_dtcs(&mut self, pending: bool) -> Result<Vec<DiagnosticTroubleCode>, Error> {
        let service = if pending { 0x07 } else { 0x03 };
        let response = self.obd_query(service, &[])?;

        let _count = response[0];
        let data = response[1..].to_vec();

        let mut dtcs = Vec::new();
        for chunk in data.chunks(2) {
            let code = ((chunk[0] as u16) << 8) + (chunk[1] as u16);
            dtcs.push(DiagnosticTroubleCode::Obd(code));
        }

        Ok(dtcs)
    }

    fn clear_dtcs(&mut self) -> Result<(), Error> {
        self.obd_query(0x04, &[])?;
        Ok(())
    }

    fn read_data(&mut self, pid: u8, freeze_frame: bool) -> Result<Vec<u8>, Error> {
        let service = if freeze_frame { 0x02 } else { 0x01 };
        self.obd_query(service, &[pid])
    }

    fn read_data_formatted(&mut self, pid: u8, freeze_frame: bool) -> Result<String, Error> {
        // TODO
        Ok(format!("{:02x?}", self.read_data(pid, freeze_frame)?))
    }
}
