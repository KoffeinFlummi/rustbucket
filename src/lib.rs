#![warn(missing_docs)]

/*!
 * Library for accessing vehicle diagnostics using a BeagleBone Blue.
 *
 * This library exports various protocols used for accessing vehicle
 * diagnostics. This includes standardized OBD2 protocols, and KWP1281, which
 * is used on older VAG cars.
 *
 * Diagnosis functionality common to all protocols is implemented using the
 * [dtc::Diagnose] trait, while functionality common to all OBD2 protocols is
 * implemented using [obd2::Obd2Protocol].
 */

pub mod can;
pub mod diagnose;
pub mod error;
pub mod kline;
pub mod kwp1281;
pub mod kwp2000;
pub mod misc;
pub mod obd2;
