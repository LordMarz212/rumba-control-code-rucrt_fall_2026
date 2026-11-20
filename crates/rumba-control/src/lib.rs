#![cfg_attr(not(test), no_std)]

//! Driver-control and LED layer for Rumba.
//!
//! Hardware-specific UART/SPI setup belongs in the Teensy firmware crate. This
//! crate stays allocation-free and independent of the robot motion controller.

pub mod driver;
pub mod failsafe;
pub mod leds;
pub mod receiver;
pub mod robot_interface;

pub use driver::{ChannelMap, DriverConfig, DriverMapper, MappedDriverOutput};
pub use failsafe::{FailsafeConfig, FailsafeReason, FailsafeState};
pub use receiver::ReceiverFrame;
pub use robot_interface::{RobotFeedback, RobotOperator, RobotState};
