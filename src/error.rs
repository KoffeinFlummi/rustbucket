//! General error type

#[derive(Debug, Default)]
pub struct Error {
    msg: String,
}

impl Error {
    pub fn new<T: Into<String>>(m: T) -> Self {
        Self { msg: m.into() }
    }
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}", self.msg)
    }
}

impl std::error::Error for Error {}

macro_rules! error_from {
    ( $t:ty ) => {
        impl From<$t> for Error {
            fn from(error: $t) -> Self {
                Error {
                    msg: format!("{}", error),
                }
            }
        }
    };
}

error_from!(std::io::Error);
error_from!(std::num::ParseIntError);
error_from!(gpio_cdev::errors::Error);
error_from!(serial::Error);
error_from!(socketcan::CANSocketOpenError);
error_from!(socketcan::ConstructionError);
