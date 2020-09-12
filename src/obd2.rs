//! General diagnosis implementation for all OBD2 protocols

use crate::dtc::*;
use crate::error::*;

/// Trait for abstracting general OBD2 functionality common to all protocols.
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
        let data = self.read_data(pid, freeze_frame)?;

        Ok(match pid {
            0x02 => {
                let code = ((data[0] as u16) << 8) + data[1] as u16;
                format!("Freeze DTC: {}", DiagnosticTroubleCode::Obd(code))
            },
            0x04 => {
                format!("Calculated engine load: {:6.2} %", data[0] as f32 / 2.55)
            },
            0x05 => {
                format!("Engine coolant temperature: {:3} C", data[0] as i16 - 40)
            },
            0x06 | 0x07 | 0x08 | 0x09 => {
                format!("{} term fuel trim - Bank {}: {:7.2} %",
                    if pid == 0x06 || pid == 0x08 { "Short" } else { "Long" },
                    if pid >= 0x08 { 2 } else { 1 },
                    data[0] as f32 / 1.25 - 100.0)
            },
            0x0a => {
                format!("Fuel pressure: {:3} kPa", data[0] as u16 * 3)
            },
            0x0b => {
                format!("Intake manifold absolute pressure: {:3} kPa", data[0])
            },
            0x0c => {
                format!("Engine speed: {:8.2} rpm", (256.0 * data[0] as f32 + data[1] as f32) / 4.0)
            },
            0x0d => {
                format!("Vehicle speed: {:3} km/h", data[0])
            },
            0x0e => {
                format!("Timing advance: {:5.1} deg before TDC", (data[0] as f32) / 2.0 - 64.0)
            },
            0x0f => {
                format!("Intake air temperature: {:3} C", data[0] as i16 - 40)
            },
            0x10 => {
                format!("MAF air flow rate: {:6.2} g/s", (256.0 * data[0] as f32 + data[1] as f32) / 100.0)
            },
            0x11 => {
                format!("Throttle position: {:3} %", data[0] as f32 / 2.55)
            },
            0x14 | 0x15 | 0x16 | 0x17 | 0x18 | 0x19 | 0x1a | 0x1b => {
                let id = pid - 0x13;
                if data[1] == 0xff {
                    format!("Oxygen Sensor {}: {:5.3} V, N/A %",
                        id,
                        data[0] as f32 / 200.0)
                } else {
                    format!("Oxygen Sensor {}: {:5.3} V, {:7.2} %",
                        id,
                        data[0] as f32 / 200.0,
                        data[1] as f32 / 1.28 - 100.0)
                }
            },
            0x1c => {
                format!("OBD standard: {}", match data[0] {
                    1 => "OBD-II as defined by the CARB",
                    2 => "OBD as defined by the EPA",
                    3 => "OBD and OBD-II",
                    4 => "OBD-I",
                    5 => "Not OBD compliant",
                    6 => "EOBD (Europe)",
                    7 => "EOBD and OBD-II",
                    8 => "EOBD and OBD",
                    9 => "EOBD, OBD and OBD II",
                    10 => "JOBD (Japan)",
                    11 => "JOBD and OBD II",
                    12 => "JOBD and EOBD",
                    13 => "JOBD, EOBD, and OBD II",
                    14 => "Reserved",
                    15 => "Reserved",
                    16 => "Reserved",
                    17 => "Engine Manufacturer Diagnostics (EMD)",
                    18 => "Engine Manufacturer Diagnostics Enhanced (EMD+)",
                    19 => "Heavy Duty On-Board Diagnostics (Child/Partial) (HD OBD-C)",
                    20 => "Heavy Duty On-Board Diagnostics (HD OBD)",
                    21 => "World Wide Harmonized OBD (WWH OBD)",
                    22 => "Reserved",
                    23 => "Heavy Duty Euro OBD Stage I without NOx control (HD EOBD-I)",
                    24 => "Heavy Duty Euro OBD Stage I with NOx control (HD EOBD-I N)",
                    25 => "Heavy Duty Euro OBD Stage II without NOx control (HD EOBD-II)",
                    26 => "Heavy Duty Euro OBD Stage II with NOx control (HD EOBD-II N)",
                    27 => "Reserved",
                    28 => "Brazil OBD Phase 1 (OBDBr-1)",
                    29 => "Brazil OBD Phase 2 (OBDBr-2)",
                    30 => "Korean OBD (KOBD)",
                    31 => "India OBD I (IOBD I)",
                    32 => "India OBD II (IOBD II)",
                    33 => "Heavy Duty Euro OBD Stage VI (HD EOBD-IV)",
                    _ => "Unknown"
                })
            },
            0x1f => {
                format!("Run time since engine start: {:5} s", (data[0] as u16) << 8 + data[1] as u16)
            },
            0x21 => {
                format!("Distance traveled with MIL on: {:5} km", (data[0] as u16) << 8 + data[1] as u16)
            },
            0x22 => {
                format!("Fuel rail pressure: {:8.3} kPa", (data[0] as f32 * 256.0 + data[1] as f32) * 0.079)
            },
            0x23 => {
                format!("Fuel rail gauge pressure: {:6} kPa", ((data[0] as u32) << 8 + data[1] as u32) * 10)
            },
            0x24 | 0x25 | 0x26 | 0x27 | 0x28 | 0x29 | 0x2a | 0x2b => {
                let id = pid - 0x23;
                format!("Oxygen Sensor {}: {:5.3}, {:6.4} V", id,
                    (2.0 / 65536.0) * (data[0] as f32 * 256.0 + data[1] as f32),
                    (8.0 / 65536.0) * (data[2] as f32 * 256.0 + data[3] as f32))
            },
            0x2c => {
                format!("Commanded EGR: {:6.2} %", data[0] as f32 / 2.55)
            },
            0x2d => {
                format!("EGR error: {:7.2} %", data[0] as f32 / 1.28 - 100.0)
            },
            0x2e => {
                format!("Commanded evaporative purge: {:6.2} %", data[0] as f32 / 2.55)
            },
            0x2f => {
                format!("Fuel tank level input: {:6.2} %", data[0] as f32 / 2.55)
            },
            0x30 => {
                format!("Warm-ups since codes cleared: {:3}", data[0])
            },
            0x31 => {
                format!("Distance traveled since codes cleared: {:5} km", (data[0] as u16) << 8 + data[1] as u16)
            },
            0x32 => {
                format!("Evaporative sytem vapor pressure: {:8.2} Pa", (data[0] as f32 * 256.0 + data[1] as f32) / 4.0)
            },
            0x33 => {
                format!("Absolute barometric pressure: {:3} kPa", data[0])
            },
            0x34 | 0x35 | 0x36 | 0x37 | 0x38 | 0x39 | 0x3a | 0x3b => {
                let id = pid - 0x33;
                format!("Oxygen sensor {}: {:5.3}, {:6.2} mA", id,
                    (2.0 / 65536.0) * (data[0] as f32 * 256.0 + data[1] as f32),
                    data[2] as f32 + (data[3] as f32 / 256.0) + 128.0)
            },
            0x3c | 0x3d | 0x3e | 0x3f => {
                format!("Catalyst temperature: Bank {}, Sensor {}: {:6.1} C",
                    if pid == 0x3c || pid == 0x3e { 1 } else { 2 },
                    if pid <= 0x3d { 1 } else { 2 },
                    (data[0] as f32 * 256.0 + data[1] as f32) / 10.0 - 40.0)
            },
            0x42 => {
                format!("Control module voltage: {:6.3} V", (data[0] as f32 * 256.0 + data[1] as f32) / 1000.0)
            },
            0x43 => {
                format!("Absolute load value: {:6.2} %", (data[0] as f32 * 256.0 + data[1] as f32) / 2.55)
            },
            0x44 => {
                format!("Fuel-Air commanded equiv. ratio: {:5.3}", (data[0] as f32 * 256.0 + data[1] as f32) * (2.0 / 65536.0))
            },
            0x45 => {
                format!("Relative throttle position: {:6.2} %", data[0] as f32 / 2.55)
            },
            0x46 => {
                format!("Ambient air temperature: {:3} C", data[0] as i16 - 40)
            },
            0x47 => {
                format!("Absolute throttle position B: {:6.2} %", data[0] as f32 / 2.55)
            },
            0x48 => {
                format!("Absolute throttle position C: {:6.2} %", data[0] as f32 / 2.55)
            },
            0x49 => {
                format!("Absolute pedal position D: {:6.2} %", data[0] as f32 / 2.55)
            },
            0x4a => {
                format!("Absolute pedal position E: {:6.2} %", data[0] as f32 / 2.55)
            },
            0x4b => {
                format!("Absolute pedal position F: {:6.2} %", data[0] as f32 / 2.55)
            },
            0x4c => {
                format!("Commanded throttle actuator: {:6.2} %", data[0] as f32 / 2.55)
            },
            0x4d => {
                format!("Time run with MIL on: {:5} m", ((data[0] as u16) << 8) + data[1] as u16)
            },
            0x4e => {
                format!("Time since trouble codes cleared: {:5} m", ((data[0] as u16) << 8) + data[1] as u16)
            },
            0x4f => {
                format!("Max. value for fuel-air equiv. ratio, oxygen sensor voltage, oxygen sensor current, and intake manifold absolute pressure: {:3}, {:3}, {:3}, {:4}",
                    data[0],
                    data[1],
                    data[2],
                    data[3] as u16 * 10)
            },
            0x50 => {
                format!("Max. value for MAF air flow rate: {:4}", data[0] as u16 * 10)
            },
            0x51 => {
                format!("Fuel type: {}", match data[0] {
                    0 => "Not available",
                    1 => "Gasoline",
                    2 => "Methanol",
                    3 => "Ethanol",
                    4 => "Diesel",
                    5 => "LPG",
                    6 => "CNG",
                    7 => "Propane",
                    8 => "Electric",
                    9 => "Bifuel running Gasoline",
                    10 => "Bifuel running Methanol",
                    11 => "Bifuel running Ethanol",
                    12 => "Bifuel running LPG",
                    13 => "Bifuel running CNG",
                    14 => "Bifuel running Propane",
                    15 => "Bifuel running Electricity",
                    16 => "Bifuel running electric and combustion engine",
                    17 => "Hybrid Gasoline",
                    18 => "Hybrid Ethanol",
                    19 => "Hybrid Diesel",
                    20 => "Hybrid Electric",
                    21 => "Hybrid running electric and combustion engine",
                    22 => "Hybrid Regenerative",
                    23 => "Bifuel running Diesel",
                    _ => "Unknown"
                })
            },
            0x52 => {
                format!("Ethanol fuel: {:6.2} %", data[0] as f32 / 2.55)
            },
            0x53 => {
                format!("Absolute evap. system vapor pressure: {:7.3} kPa",
                    (data[0] as f32 * 256.0 + data[1] as f32) / 200.0)
            },
            0x54 => {
                format!("Evap. system vapor pressure. {:6} Pa",
                    (((data[0] as i32) << 8) + data[1] as i32) - 32767)
            },
            0x55 | 0x56 | 0x57 | 0x58 => {
                if data.len() > 1 {
                    format!("{} term secondary oxygen sensor trim: bank {}: {:6.2} %, bank {}: {:6.2} %",
                        if pid == 0x55 || pid == 0x57 { "Short" } else { "Long" },
                        if pid <= 0x56 { 1 } else { 3 },
                        data[0] as f32 / 1.28 - 100.0,
                        if pid <= 0x56 { 2 } else { 4 },
                        data[1] as f32 / 1.28 - 100.0)
                } else {
                    // this is guessed, Golf Mk7 returned just 1 byte.
                    format!("{} term secondary oxygen sensor trim: bank {}: {:6.2} %",
                        if pid == 0x55 || pid == 0x57 { "Short" } else { "Long" },
                        if pid <= 0x56 { 1 } else { 2 },
                        data[0] as f32 / 1.28 - 100.0)
                }
            },
            0x59 => {
                format!("Fuel rail absolute pressure: {:6} kPa",
                    (((data[0] as u32) << 8) + data[1] as u32) * 10)
            },
            0x5a => {
                format!("Relative accelerator pedal position: {:6.2} %", data[0] as f32 / 2.55)
            },
            0x5b => {
                format!("Hybrid battery pack remaining life: {:6.2} %", data[0] as f32 / 2.55)
            },
            0x5c => {
                format!("Engine oil temperature: {:3} C", data[0] as i16 - 40)
            },
            0x5d => {
                format!("Fuel injection timing: {:8.3} deg",
                    (data[0] as f32 * 256.0 + data[1] as f32) / 128.0 - 210.0)
            },
            0x5e => {
                format!("Engine fuel rate: {:7.2} L/h",
                    (data[0] as f32 * 256.0 + data[1] as f32) / 20.0)
            },
            0x61 => {
                format!("Driver's demand engine: {:4} % torque", data[0] as i16 - 125)
            },
            0x62 => {
                format!("Actual engine: {:4} % torque", data[0] as i16 - 125)
            },
            0x63 => {
                format!("Engine reference torque: {:5} Nm", ((data[0] as u16) << 8) + data[1] as u16)
            },
            0x64 => {
                format!("Engine percent torque data: idle: {:4} %, P1: {:4} %, P2: {:4} %, P3: {:4} %, P4: {:4} %",
                    data[0] as i16 - 125,
                    data[1] as i16 - 125,
                    data[2] as i16 - 125,
                    data[3] as i16 - 125,
                    data[4] as i16 - 125)
            },
            0xa6 => {
                format!("Odometer: {:6} km",
                    ((data[0] as u64) << 24) + ((data[1] as u64) << 16) +
                    ((data[2] as u64) << 8) + data[3] as u64)
            },
            _ => format!("{:02x?}", data)
        })
    }
}
