use std::io::Write;

use colored::*;
use docopt::Docopt;
use env_logger;
use log::{error, info, Level, LevelFilter};
use serde::Deserialize;

mod can;
mod dtc;
mod error;
mod kline;
mod kwp1281;
mod kwp2000;
mod misc;
mod obd2;

use crate::can::*;
use crate::dtc::*;
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
    rustbucket <protocol> simulator [-v] [--bitrate=<bps>]
    rustbucket test-hardware (tx|rx) [-v] [--bitrate=<bps>]
    rustbucket (-h | --help)
    rustbucket --version

Args:
    <protocol>          Protcol to use. One of:
                            - can       CAN Bus / ISO 15765
                            - kwp1281   KWP1281, K line only
                            - iso9141   ISO 9141, K & L line, unimplemented
                            - kwp2000   KWP2000 / ISO 14230, K & L line

Commands:
    read-dtcs           Read diagnostic trouble codes.
    clear-dtcs          Clear diagnostic trouble codes.
    simulator           Run a car simulater for testing.
    test-hardware       Test K line logic level conversion hardware by either
                            transmitting or receiving serial data continuously.
                            Good for connecting hooking up an oscilloscope.

Options:
    --pending           Read pending DTCs instead of stored ones.
                            (not supported by KWP1281)
--bitrate=<bps>     Set baud/bit rate manually. For KWP1281 protocol this
                        will be determined automagically by default.
                        For the CAN bus, this defaults to 500,000.
-v --verbose        Show more output.
-h --help           Show usage information.
--version           Show version.
";

/// Enum of protocols for CLI arg parsing
#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
enum ProtocolType {
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
    cmd_simulator: bool,
    cmd_test_hardware: bool,
    cmd_tx: bool,
    cmd_rx: bool,
    arg_protocol: Option<ProtocolType>,
    flag_verbose: bool,
    flag_bitrate: Option<u64>,
    flag_pending: bool,
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

    if args.cmd_test_hardware {
        return KLine::test_hardware(args.cmd_tx, args.flag_bitrate.unwrap_or(9600));
    }

    let protocol_type = args.arg_protocol.unwrap();

    if protocol_type == ProtocolType::Iso9141 {
        return Err(Error::new("Protocol currently unimplemented."));
    }

    if protocol_type == ProtocolType::Kwp1281 && args.flag_pending {
        return Err(Error::new("KWP1281 doesn't support pending DTCs."));
    }

    if args.cmd_simulator {
        return match protocol_type {
            ProtocolType::Can => CanBus::run_simulator(args.flag_bitrate.unwrap_or(500000)),
            ProtocolType::Kwp1281 => Kwp1281::run_simulator(args.flag_bitrate.unwrap_or(9600)),
            ProtocolType::Iso9141 => unimplemented!(),
            ProtocolType::Kwp2000 => todo!(),
        };
    }

    if args.cmd_clear_dtcs {
        if !confirm(format!("{}: Attempting to clear the DTCs may result in injury, fire, exploding airbags or death.\nNo warranty. Are you sure you wish to proceed?", "CAUTION".bold().red()))? {
            println!("Aborting.");
            return Ok(());
        }

        println!("Proceeding. No refunds!");
    }

    // Init protocol and get/print some identifying information to make
    // sure the protocol is working.
    let mut protocol: Box<dyn Diagnose> = match protocol_type {
        ProtocolType::Can => {
            let mut can = CanBus::init(args.flag_bitrate)?;
            info!(
                "VIN: {:?}",
                String::from_utf8_lossy(&can.obd_query(0x09, &[0x02])?[1..])
            );
            Box::new(can)
        }
        ProtocolType::Kwp1281 => {
            let kwp = Kwp1281::init(args.flag_bitrate)?;
            info!("ECU data: {:?}", String::from_utf8_lossy(&kwp.ecu_data));
            Box::new(kwp)
        }
        ProtocolType::Kwp2000 => {
            let mut kwp = Kwp2000::init(args.flag_bitrate)?;
            info!(
                "VIN: {:?}",
                String::from_utf8_lossy(&kwp.obd_query(0x1a, &[0x90])?)
            );
            Box::new(kwp)
        }
        _ => unimplemented!(),
    };

    if args.cmd_clear_dtcs {
        protocol.clear_dtcs()?;

        println!("\n{}", "DTCs cleared successfully.".green().bold());
        println!("You may want to read the DTCs again to make sure errors have not reappeared.");
    }

    if args.cmd_read_dtcs {
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
    }

    // Insert a newline between the output and the CAN Drop debug log.
    if protocol_type == ProtocolType::Can && args.flag_verbose {
        println!("");
    }

    Ok(())
}

fn main() {
    if let Err(e) = do_main() {
        error!("{}", e);
        std::process::exit(1);
    }
}
