#![cfg_attr(not(test), no_std)]

//! Driver-control and LED layer for Rumba.
//!
//! Hardware-specific UART/SPI setup belongs in the Teensy firmware crate. This
//! crate stays allocation-free and independent of the robot motion controller.

pub mod driver;
pub mod failsafe;
pub mod firmware;
pub mod leds;
pub mod receiver;
pub mod robot_interface;

pub use driver::{AuxiliaryControlMap, ChannelMap, DriverConfig, DriverMapper, LedSwitchBinding, MappedDriverOutput, MAX_AUX_SWITCHES};
pub use failsafe::{FailsafeConfig, FailsafeReason, FailsafeState};
pub use firmware::{FirmwareTick, RobotControlPort, RumbaFirmware};
pub use receiver::ReceiverFrame;
pub use robot_interface::{RobotFeedback, RobotOperator, RobotState};
