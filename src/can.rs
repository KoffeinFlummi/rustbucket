//! Protocol implementation for the CAN bus / ISO 15765

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, SystemTime};

use log::{debug, warn};
use socketcan;

use crate::error::*;
use crate::misc::*;
use crate::obd2::*;

/// Protocol for talking to the vehicle via the CAN bus.
pub struct CanBus {
    /// CAN bus socket
    pub socket: socketcan::CANSocket,
}

impl CanBus {
    /**
     * Initialize the CAN protocol on the `can0` interface. This method expects
     * the `can0` network interface to not currently exist, and brings it up via
     * the `ip` command with the given bit rate.
     *
     * The [Drop] trait is implemented to ensure the network interface is
     * brought down again on termination.
     */
    pub fn init(bit_rate: Option<u64>) -> Result<Self, Error> {
        run_cmd_as_root(format!(
            "ip link set can0 up type can bitrate {}",
            bit_rate.unwrap_or(500000)
        ))?;

        let socket = socketcan::CANSocket::open("can0")?;
        socket.set_read_timeout(Duration::from_millis(500))?;
        socket.set_write_timeout(Duration::from_millis(500))?;

        Ok(Self { socket })
    }

    /**
     * Run a crude car simulator using the given bit rate.
     */
    pub fn run_simulator(bit_rate: u64) -> Result<(), Error> {
        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();

        ctrlc::set_handler(move || {
            r.store(false, Ordering::SeqCst);
        })
        .unwrap();

        let can_bus = Self::init(Some(bit_rate))?;

        while running.load(Ordering::SeqCst) {
            let frame = match can_bus.socket.read_frame() {
                Err(_) => {
                    continue;
                }
                Ok(f) => f,
            };

            debug!("RECV {:02X}", frame);

            match frame.data()[1] {
                0x03 | 0x07 => {
                    // send some unrelated message to keep the receiver on its toes
                    let response =
                        socketcan::CANFrame::new(0x484, &[0x02, 0x00, 0x42], false, false)?;
                    debug!("SEND {:02X}", response);
                    can_bus.socket.write_frame(&response)?;

                    let service = 0x40 + frame.data()[1];
                    let response = socketcan::CANFrame::new(
                        0x7e8,
                        &[0x06, service, 0x02, 0x00, 0x11, 0xd1, 0x01],
                        false,
                        false,
                    )?;
                    debug!("SEND {:02X}", response);
                    can_bus.socket.write_frame(&response)?;
                }
                0x04 => {
                    let response = socketcan::CANFrame::new(0x7e8, &[0x01, 0x44], false, false)?;
                    debug!("SEND {:02X}", response);
                    can_bus.socket.write_frame(&response)?;
                }
                0x09 => match frame.data()[2] {
                    0x02 => {
                        let mut frame1: Vec<u8> = vec![0x10, 0x0f, 0x49, 0x02, 0x01];
                        frame1.extend("VIN".as_bytes());

                        let response = socketcan::CANFrame::new(0x7e8, &frame1, false, false)?;
                        debug!("SEND {:02X}", response);
                        can_bus.socket.write_frame(&response)?;

                        // Handle only a single FC frame.
                        let flow_control = can_bus.socket.read_frame()?;
                        let flow_data = flow_control.data();
                        if &flow_data[0..2] != &[0x30, 0x00] {
                            return Err(Error::new("Unexpected flow control frame."));
                        }

                        let mut frame2: Vec<u8> = vec![0x20];
                        frame2.extend("VINVINV".as_bytes());

                        let response = socketcan::CANFrame::new(0x7e8, &frame2, false, false)?;
                        debug!("SEND {:02X}", response);
                        can_bus.socket.write_frame(&response)?;

                        let mut frame3: Vec<u8> = vec![0x21];
                        frame3.extend("IN".as_bytes());

                        let response = socketcan::CANFrame::new(0x7e8, &frame3, false, false)?;
                        debug!("SEND {:02X}", response);
                        can_bus.socket.write_frame(&response)?;
                    }
                    _ => {
                        warn!("Unknown query.");
                    }
                },
                _ => {
                    warn!("Unknown query.");
                }
            }
        }

        Ok(())
    }
}

impl Drop for CanBus {
    fn drop(&mut self) {
        if let Err(e) = run_cmd_as_root("ip link set can0 down") {
            warn!("Failed to shut down CAN interface: {}", e);
        }
    }
}

impl Obd2Protocol for CanBus {
    fn obd_query(&mut self, service: u8, args: &[u8]) -> Result<Vec<u8>, Error> {
        let mut data: Vec<u8> = vec![1 + args.len() as u8, service];
        data.extend(args);
        data.extend(vec![0xcc; 8 - data.len()]);

        let query = socketcan::CANFrame::new(0x7df, &data, false, false)?;
        debug!("SEND {:02X}", query);
        self.socket.write_frame_insist(&query)?;

        let mut response = Vec::new();
        let mut length: Option<usize> = None;

        let start = SystemTime::now();
        loop {
            if start.elapsed().unwrap().as_millis() > 2000 {
                return Err(Error::new("Timed out waiting for response."));
            }

            let frame = self.socket.read_frame()?;
            if !(frame.id() >= 0x7e8 && frame.id() <= 0x7ef) {
                continue;
            }

            debug!("RECV {:02X}", frame);

            let frame_type = frame.data()[0] >> 4;
            match frame_type {
                0x00 => {
                    // single frame
                    if length.is_some() {
                        return Err(Error::new("Unexpected CAN-TP message."));
                    }

                    length = Some(frame.data()[0] as usize);

                    response.extend(&frame.data()[1..]);
                    break;
                }
                0x01 => {
                    // first multi-frame message
                    if length.is_some() {
                        return Err(Error::new("Unexpected CAN-TP message."));
                    }

                    length =
                        Some(((frame.data()[0] as usize & 0xf) << 8) + frame.data()[1] as usize);
                    response.extend(&frame.data()[2..]);

                    // acknowledge, instruct sender to send the rest without
                    // waiting for further flow control messages
                    let msg = vec![0x30, 0x00, 0xff, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc];
                    let flow = socketcan::CANFrame::new(0x7df, &msg, false, false)?;
                    debug!("SEND {:02X}", flow);
                    self.socket.write_frame_insist(&flow)?;
                }
                0x02 => {
                    // consecutive multi-frame message
                    if length.is_none() {
                        return Err(Error::new("Unexpected CAN-TP message."));
                    }

                    // TODO: check index?

                    response.extend(&frame.data()[1..]);

                    if response.len() >= length.unwrap() {
                        break;
                    }
                }
                _ => {
                    return Err(Error::new("Unexpected CAN-TP message."));
                }
            }
        }

        if response[0] != service + 0x40 {
            return Err(Error::new("Service identifier of response did not match."));
        }

        if &response[1..(1 + args.len())] != args {
            return Err(Error::new("Arguments/PIDs did not match."));
        }

        response.truncate(length.unwrap());

        Ok(response[(1 + args.len())..].to_vec())
    }
}
