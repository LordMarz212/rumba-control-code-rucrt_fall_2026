//! Allocation-free control-loop integration around the iBUS decoder and mapper.
//!
//! The board crate supplies the UART bytes and implements `RobotControlPort`
//! using the existing robosim controller. Board pin setup and motor/LED output
//! remain outside this portable crate.

use crate::driver::{DriverConfig, DriverMapper, MappedDriverOutput};
use crate::receiver::ibus::IbusDecoder;
use crate::receiver::ReceiverFrame;
use crate::robot_interface::{RobotFeedback, RobotOperator};

/// Adapter implemented by the Teensy firmware around the existing robot logic.
pub trait RobotControlPort {
    type Error;

    /// Apply one fresh operator command to robosim and return the resulting
    /// state and measured spin rate for the next control-loop iteration.
    fn update(&mut self, command: RobotOperator) -> Result<RobotFeedback, Self::Error>;

    /// Phase predicted for the next scheduled actuation, in radians.
    fn predicted_phase_rad(&self) -> f32;
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FirmwareTick {
    pub controls: MappedDriverOutput,
    pub feedback: RobotFeedback,
    pub predicted_phase_rad: f32,
}

/// Joins receiver parsing, channel mapping, and the robosim command boundary.
/// Call `tick` continuously at the controller's loop rate, even when no new
/// receiver bytes arrive, so stale input always reaches the failsafe path.
pub struct RumbaFirmware {
    decoder: IbusDecoder,
    latest_frame: Option<ReceiverFrame>,
    mapper: DriverMapper,
    feedback: RobotFeedback,
}

impl RumbaFirmware {
    pub const fn new(config: DriverConfig) -> Self {
        Self {
            decoder: IbusDecoder::new(),
            latest_frame: None,
            mapper: DriverMapper::new(config),
            feedback: RobotFeedback {
                state: crate::robot_interface::RobotState::Disarmed,
                spin_rpm: 0.0,
                phase_rad: 0.0,
                spin_valid: false,
            },
        }
    }

    /// Feed one byte from the receiver UART. Returns true only when a complete
    /// iBUS channel frame passes header and checksum validation.
    pub fn push_receiver_byte(&mut self, byte: u8, received_at_us: u64) -> bool {
        if let Some(frame) = self.decoder.push(byte, received_at_us) {
            self.latest_frame = Some(frame);
            true
        } else {
            false
        }
    }

    /// Call immediately after a UART overrun, framing error, or receiver reset.
    /// This drops retained input and forces the next tick into failsafe.
    pub fn receiver_io_error(&mut self) {
        self.decoder = IbusDecoder::new();
        self.latest_frame = None;
    }

    pub fn latest_frame(&self) -> Option<&ReceiverFrame> {
        self.latest_frame.as_ref()
    }

    pub fn tick<P: RobotControlPort>(
        &mut self,
        now_us: u64,
        robot: &mut P,
    ) -> Result<FirmwareTick, P::Error> {
        let controls = self.mapper.update(self.latest_frame.as_ref(), now_us, self.feedback);
        let feedback = robot.update(controls.operator)?;
        self.feedback = feedback;
        Ok(FirmwareTick {
            controls,
            feedback,
            predicted_phase_rad: robot.predicted_phase_rad(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::receiver::ibus::{IBUS_CHANNELS, IBUS_FRAME_LEN};
    use crate::robot_interface::RobotState;

    #[derive(Default)]
    struct FakeRobot {
        command: Option<RobotOperator>,
        feedback: RobotFeedback,
    }

    impl RobotControlPort for FakeRobot {
        type Error = ();

        fn update(&mut self, command: RobotOperator) -> Result<RobotFeedback, Self::Error> {
            self.command = Some(command);
            Ok(self.feedback)
        }

        fn predicted_phase_rad(&self) -> f32 {
            1.25
        }
    }

    fn ibus_frame(enable: bool) -> [u8; IBUS_FRAME_LEN] {
        let mut bytes = [0u8; IBUS_FRAME_LEN];
        bytes[0] = 0x20;
        bytes[1] = 0x40;
        let mut channels = [1500u16; IBUS_CHANNELS];
        channels[2] = 1750; // default left-stick throttle
        channels[4] = if enable { 2000 } else { 1000 }; // default spin-enable switch
        for (index, value) in channels.iter().enumerate() {
            let offset = 2 + index * 2;
            bytes[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
        }
        let sum = bytes[..30].iter().fold(0u16, |sum, byte| sum.wrapping_add(*byte as u16));
        bytes[30..32].copy_from_slice(&0xFFFFu16.wrapping_sub(sum).to_le_bytes());
        bytes
    }

    #[test]
    fn receiver_bytes_flow_to_robot_operator() {
        let mut firmware = RumbaFirmware::new(DriverConfig::default());
        let mut robot = FakeRobot::default();
        for byte in ibus_frame(false) {
            firmware.push_receiver_byte(byte, 10_000);
        }
        robot.feedback = RobotFeedback { state: RobotState::Disarmed, spin_rpm: 0.0, phase_rad: 0.25, spin_valid: true };
        firmware.tick(10_001, &mut robot).unwrap();
        for byte in ibus_frame(true) {
            firmware.push_receiver_byte(byte, 10_002);
        }
        let tick = firmware.tick(10_003, &mut robot).unwrap();
        let command = robot.command.unwrap();
        assert!(command.enable);
        assert_eq!(command.target_rpm, 2250.0);
        assert_eq!(tick.predicted_phase_rad, 1.25);
        assert_eq!(tick.controls.failsafe, crate::FailsafeState::Healthy);
    }

    #[test]
    fn io_error_immediately_drops_cached_receiver_input() {
        let mut firmware = RumbaFirmware::new(DriverConfig::default());
        let mut robot = FakeRobot { feedback: RobotFeedback { state: RobotState::Disarmed, spin_rpm: 0.0, phase_rad: 0.5, spin_valid: true }, ..FakeRobot::default() };
        for byte in ibus_frame(false) {
            firmware.push_receiver_byte(byte, 10_000);
        }
        firmware.tick(10_001, &mut robot).unwrap();
        for byte in ibus_frame(true) {
            firmware.push_receiver_byte(byte, 10_002);
        }
        robot.feedback.state = RobotState::Running;
        robot.feedback.spin_rpm = 1000.0;
        firmware.tick(10_003, &mut robot).unwrap();
        firmware.receiver_io_error();
        let tick = firmware.tick(10_002, &mut robot).unwrap();
        assert_eq!(tick.controls.failsafe, crate::FailsafeState::Active(crate::FailsafeReason::NoFrame));
        assert!(robot.command.unwrap().stop);
        assert!(firmware.latest_frame().is_none());
    }
}
