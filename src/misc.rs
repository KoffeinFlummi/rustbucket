//! Various utility functions

use std::fs::File;
use std::io::Write;
use std::process::Command;
use std::time::SystemTime;

use colored::*;
use gpio_cdev::LineHandle;
use log::debug;
use nix;

use crate::error::*;

pub enum PinMode {
    Gpio,
    Uart,
}

impl Into<String> for PinMode {
    fn into(self) -> String {
        match self {
            Self::Gpio => String::from("gpio"),
            Self::Uart => String::from("uart"),
        }
    }
}

pub fn busy_wait<T: Into<u128> + Copy>(reference: SystemTime, elapsed_micros: T) {
    while reference.elapsed().unwrap().as_micros() < elapsed_micros.into() {}
}

pub fn busy_wait_until<T: Into<u128> + Copy>(
    rx: &LineHandle,
    value: u8,
    timeout_micros: T,
) -> Result<(), Error> {
    let start = SystemTime::now();
    while start.elapsed().unwrap().as_micros() < timeout_micros.into() {
        if rx.get_value()? == value {
            return Ok(());
        }
    }

    Err(Error::new("Timed out waiting for edge."))
}

pub fn set_pin_mode(chip: u32, pin: u32, mode: PinMode) -> Result<(), Error> {
    let path = format!(
        "/sys/devices/platform/ocp/ocp:P{}_{}_pinmux/state",
        chip, pin
    );
    let mut f = File::create(path)?;
    let string: String = mode.into();
    f.write(&string.into_bytes())?;
    Ok(())
}

pub fn confirm(msg: String) -> Result<bool, Error> {
    let mut stdout = std::io::stdout();
    print!("{} ({}/{}): ", msg, "y".bold().green(), "N".bold().red());
    stdout.flush()?;

    let mut input = String::new();
    std::io::stdin().read_line(&mut input)?;
    Ok(input.to_lowercase() == "y\n")
}

pub fn run_cmd_as_root<T: Into<String>>(cmd: T) -> Result<(), Error> {
    let mut cmd = cmd.into();
    if !nix::unistd::getuid().is_root() {
        cmd = format!("sudo {}", cmd);
    }

    debug!("$ {}", cmd);

    let chunks: Vec<&str> = cmd.split(" ").collect();
    let status = Command::new(chunks[0]).args(&chunks[1..]).status()?;

    if !status.success() {
        return Err(Error::new("Failed to initialize CAN interface."));
    }

    Ok(())
}
