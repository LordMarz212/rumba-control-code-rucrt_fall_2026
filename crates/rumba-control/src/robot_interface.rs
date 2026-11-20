//! Stable command/feedback boundary to the separate robosim controller.
//!
//! Field names intentionally mirror control_core::Operator. Firmware glue can
//! copy these fields into that type without duplicating controller calculations.

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RobotOperator {
    pub timestamp_us: u64,
    pub enable: bool,
    pub stop: bool,
    pub brake: bool,
    pub target_rpm: f32,
    pub translation: [f32; 2],
    pub heading_trim_rad: f32,
    pub reference_id: u32,
    pub reference_heading_rad: f32,
    pub clear_fault: bool,
}

impl RobotOperator {
    pub const fn disarmed(timestamp_us: u64) -> Self {
        Self {
            timestamp_us,
            enable: false,
            stop: false,
            brake: false,
            target_rpm: 0.0,
            translation: [0.0, 0.0],
            heading_trim_rad: 0.0,
            reference_id: 0,
            reference_heading_rad: 0.0,
            clear_fault: false,
        }
    }

    pub const fn controlled_stop(timestamp_us: u64) -> Self {
        Self {
            timestamp_us,
            enable: true,
            stop: true,
            brake: false,
            target_rpm: 0.0,
            translation: [0.0, 0.0],
            heading_trim_rad: 0.0,
            reference_id: 0,
            reference_heading_rad: 0.0,
            clear_fault: false,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RobotState {
    Disarmed,
    Calibrating,
    SpinUp,
    Running,
    Stopping,
    Recovering,
    Fault,
}

impl Default for RobotState {
    fn default() -> Self {
        Self::Disarmed
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RobotFeedback {
    pub state: RobotState,
    /// Measured spin rate converted to RPM, preserving sign.
    pub spin_rpm: f32,
}

impl RobotFeedback {
    pub fn is_stopped(self, threshold_rpm: f32) -> bool {
        self.spin_rpm.is_finite()
            && self.spin_rpm.abs() <= threshold_rpm.max(0.0)
            && matches!(self.state, RobotState::Disarmed | RobotState::Stopping)
    }
}

/// Optional integration seam for firmware that owns the robosim controller.
pub trait RobotCommandSink {
    type Error;
    fn apply_operator(&mut self, command: RobotOperator) -> Result<(), Self::Error>;
}
