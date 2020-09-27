use std::fs::File;
use std::io::{stdout, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::SystemTime;

use colored::*;
use docopt::Docopt;
use env_logger;
use log::{error, info, Level, LevelFilter};
use serde::Deserialize;

mod can;
mod diagnose;
mod error;
mod kline;
mod kwp1281;
mod kwp2000;
mod misc;
mod obd2;

use crate::can::*;
use crate::diagnose::*;
use crate::error::*;
use crate::kline::*;
use crate::kwp1281::*;
use crate::kwp2000::*;
use crate::misc::*;
use crate::obd2::*;

const VERSION: &'static str = "v0.1";
const USAGE: &'static str = "
Usage:
    rustbucket <protocol> read-dtcs [-v] [--bitrate=<bps>] [--pending]
    rustbucket <protocol> clear-dtcs [-v] [--bitrate=<bps>]
    rustbucket <protocol> read-data <pid> [-v] [-t [--log=<logfile>]] [--freeze-frame]
    rustbucket <protocol> dump-data [-v] [-r] [--freeze-frame]
    rustbucket <protocol> simulator [-v] [--bitrate=<bps>]
    rustbucket test-hardware (tx|rx) [-v] [--bitrate=<bps>]
    rustbucket (-h | --help)
    rustbucket --version

Args:
    <protocol>          Protocol to use. One of:
                            - can       CAN Bus / ISO 15765
                            - kwp1281   KWP1281, K line only
                            - iso9141   ISO 9141, K & L line, unimplemented
                            - kwp2000   KWP2000 / ISO 14230, K & L line

Commands:
    read-dtcs           Read Diagnostic Trouble Codes.
    clear-dtcs          Clear Diagnostic Trouble Codes.
    read-data           Read either current or freeze frame data for a given
                            PID/group (either decimal or hex with 0x prefix).
                            Freeze frame not supported on KWP1281.
    dump-data           Enumerate through all data PIDs/groups, and dump it all
                            either formatted or in hex.
                            Freeze frame not supported on KWP1281.
    simulator           Run a car simulater for testing.
    test-hardware       Test K line logic level conversion hardware by either
                            transmitting or receiving serial data continuously.
                            Good for hooking up an oscilloscope.

Options:
    --pending           Read pending DTCs instead of stored ones.
                            (not supported by KWP1281)
    --bitrate=<bps>     Set baud/bit rate manually. For K line protocols this
                            will be determined automagically by default.
                            For the CAN bus, this defaults to 500,000.
    -t --tail           Keep requerying data.
    -l --log=<logfile>  Write floating point values to CSV file.
    --freeze-frame      Query data from freeze frame.
    -r --raw            Dump data in raw hex.
    -v --verbose        Show more output.
    -h --help           Show usage information.
    --version           Show version.

For more information on OBD2 PIDs, consult resources such as:
    https://en.wikipedia.org/wiki/OBD-II_PIDs#Service_01

For KWP1281, group IDs and their meanings differ by ECU.
";

/// Enum of protocols for CLI arg parsing
#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
enum Protocol {
    Can,
    Kwp1281,
    Iso9141,
    Kwp2000,
}

/// CLI args
#[derive(Debug, Deserialize)]
struct Args {
    cmd_read_dtcs: bool,
    cmd_clear_dtcs: bool,
    cmd_read_data: bool,
    cmd_dump_data: bool,
    cmd_simulator: bool,
    cmd_test_hardware: bool,
    cmd_tx: bool,
    cmd_rx: bool,
    arg_protocol: Option<Protocol>,
    arg_pid: Option<String>,
    flag_verbose: bool,
    flag_bitrate: Option<u64>,
    flag_pending: bool,
    flag_freeze_frame: bool,
    flag_tail: bool,
    flag_log: Option<String>,
    flag_raw: bool,
}

fn init_protocol(args: &Args) -> Result<Box<dyn Diagnose>, Error> {
    Ok(match args.arg_protocol {
        Some(Protocol::Can) => {
            let mut can = CanBus::init(args.flag_bitrate)?;
            if let Ok(vin) = can.obd_query(0x09, &[0x02]) {
                info!("VIN: {:?}", String::from_utf8_lossy(&vin[1..]));
            } else {
                // Display error but keep going. Might be an issue with multi-
                // frame messages, so other functionality might still work.
                error!("Failed to retrieve VIN.");
            }
            Box::new(can)
        }
        Some(Protocol::Kwp1281) => {
            let kwp = Kwp1281::init(args.flag_bitrate)?;
            info!("ECU data: {:?}", String::from_utf8_lossy(&kwp.ecu_data));
            Box::new(kwp)
        }
        Some(Protocol::Kwp2000) => {
            let mut kwp = Kwp2000::init(args.flag_bitrate)?;
            info!(
                "VIN: {:?}",
                String::from_utf8_lossy(&kwp.obd_query(0x1a, &[0x90])?)
            );
            Box::new(kwp)
        }
        _ => unimplemented!(),
    })
}

fn cmd_read_dtcs(args: Args) -> Result<(), Error> {
    if args.arg_protocol == Some(Protocol::Kwp1281) && args.flag_pending {
        return Err(Error::new("KWP1281 doesn't support pending DTCs."));
    }

    let mut protocol = init_protocol(&args)?;
    let dtcs = protocol.read_dtcs(args.flag_pending)?;

    if dtcs.len() == 0 {
        println!("\n{}", "No DTCs.".green().bold());
    }

    for (i, dtc) in dtcs.iter().enumerate() {
        println!(
            "\n{}: {} ({})",
            format!("DTC #{}", i + 1).green().bold(),
            format!("{}", dtc).bold(),
            dtc.more_info()
        );
        print!("{}", dtc.help());
    }

    // Insert a newline between the output and the CAN Drop debug log.
    if args.arg_protocol == Some(Protocol::Can) && args.flag_verbose {
        println!("");
    }

    Ok(())
}

fn cmd_clear_dtcs(args: Args) -> Result<(), Error> {
    if !confirm(format!("{}: Attempting to clear the DTCs may result in injury, fire, exploding airbags or death.\nNo warranty. Are you sure you wish to proceed?", "CAUTION".bold().red()))? {
        println!("Aborting.");
        return Ok(());
    }

    println!("Proceeding. No refunds!");

    let mut protocol = init_protocol(&args)?;
    protocol.clear_dtcs()?;

    println!("\n{}", "DTCs cleared successfully.".green().bold());
    println!("You may want to read the DTCs again to make sure errors have not reappeared.");

    // Insert a newline between the output and the CAN Drop debug log.
    if args.arg_protocol == Some(Protocol::Can) && args.flag_verbose {
        println!("");
    }

    Ok(())
}

fn cmd_read_data(args: Args) -> Result<(), Error> {
    let pid = args.arg_pid.clone().unwrap();
    let pid = if pid.len() >= 2 && &pid[0..2] == "0x" {
        u8::from_str_radix(&pid[2..], 16)?
    } else {
        u8::from_str_radix(&pid, 10)?
    };

    let mut protocol = init_protocol(&args)?;

    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })
    .unwrap();

    println!("");

    let mut logfile = None;
    if let Some(p) = args.flag_log {
        logfile = Some(File::create(p)?);
    }

    let start = SystemTime::now();

    loop {
        let data = protocol.read_data(pid, args.flag_freeze_frame)?;

        if let Some(f) = logfile.as_mut() {
            let floats = data.floats()?;

            f.write(
                format!(
                    "{},{}\n",
                    start.elapsed().unwrap().as_secs_f32(),
                    floats
                        .iter()
                        .map(|f| f.to_string())
                        .collect::<Vec<String>>()
                        .join(",")
                )
                .as_bytes(),
            )?;
        }

        print!(
            "\r{}: {}",
            if args.arg_protocol == Some(Protocol::Kwp1281) {
                format!("Group {} (0x{:02x})", pid, pid).green().bold()
            } else {
                format!("PID {} (0x{:02x})", pid, pid).green().bold()
            },
            data
        );

        // Since -v makes protocols print data, staying on the same line
        // doesn't work anyways, and sometimes it may be desirable to see
        // previous readings.
        if args.flag_verbose {
            println!("");
        }

        stdout().flush()?;

        if !args.flag_tail || !running.load(Ordering::SeqCst) {
            break;
        }
    }

    if !args.flag_verbose {
        println!("");
    }

    // Insert a newline between the output and the CAN Drop debug log.
    if args.arg_protocol == Some(Protocol::Can) && args.flag_verbose {
        println!("");
    }

    Ok(())
}

fn cmd_dump_data(args: Args) -> Result<(), Error> {
    let mut protocol = init_protocol(&args)?;

    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })
    .unwrap();

    for i in 0x00..=0xff {
        if !running.load(Ordering::SeqCst) {
            break;
        }

        let data = match protocol.read_data(i, args.flag_freeze_frame) {
            Ok(d) => d,
            Err(e) => {
                error!("Failed to read PID {:02}: {}", i, e);
                continue;
            }
        };

        if args.flag_raw {
            println!("{:02x} {:02x?}", i, data.raw());
        } else {
            println!("{:02x} {}", i, data);
        }
    }

    Ok(())
}

fn cmd_simulator(args: Args) -> Result<(), Error> {
    match args.arg_protocol.unwrap() {
        Protocol::Can => CanBus::run_simulator(args.flag_bitrate.unwrap_or(500000)),
        Protocol::Kwp1281 => Kwp1281::run_simulator(args.flag_bitrate.unwrap_or(9600)),
        Protocol::Iso9141 => unimplemented!(),
        Protocol::Kwp2000 => todo!(),
    }
}

fn cmd_test_hardware(args: Args) -> Result<(), Error> {
    KLine::test_hardware(args.cmd_tx, args.flag_bitrate.unwrap_or(9600))
}

fn do_main() -> Result<(), Error> {
    let args: Args = Docopt::new(USAGE)
        .map(|d| d.version(Some(VERSION.into())))
        .and_then(|d| d.deserialize())
        .unwrap_or_else(|e| e.exit());

    env_logger::Builder::new()
        .format(|buf, record| {
            writeln!(
                buf,
                "{}: {}",
                match record.level() {
                    Level::Error => "error".bold().red(),
                    Level::Warn => "warn".bold().yellow(),
                    Level::Info => "info".bold().green(),
                    Level::Debug => "debug".bold().blue(),
                    Level::Trace => "trace".bold(),
                },
                record.args()
            )
        })
        .filter(
            None,
            if args.flag_verbose {
                LevelFilter::Debug
            } else {
                LevelFilter::Info
            },
        )
        .init();

    if args.arg_protocol == Some(Protocol::Iso9141) {
        return Err(Error::new("Protocol currently unimplemented."));
    }

    if args.cmd_read_dtcs {
        cmd_read_dtcs(args)
    } else if args.cmd_clear_dtcs {
        cmd_clear_dtcs(args)
    } else if args.cmd_read_data {
        cmd_read_data(args)
    } else if args.cmd_dump_data {
        cmd_dump_data(args)
    } else if args.cmd_simulator {
        cmd_simulator(args)
    } else if args.cmd_test_hardware {
        cmd_test_hardware(args)
    } else {
        unreachable!()
    }
}

fn main() {
    if let Err(e) = do_main() {
        error!("{}", e);
        std::process::exit(1);
    }
}
