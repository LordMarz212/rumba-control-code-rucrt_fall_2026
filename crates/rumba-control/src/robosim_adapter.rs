//! Concrete bridge to the pinned robosim `control-core` controller.
//!
//! Board code owns sensor acquisition and output scheduling through
//! [`RobosimHardware`]. State estimation, phase, and motor control remain in
//! robosim; this module only transfers the operator and diagnostics.

use control_core::{Config, Controller, Input, Operator, Output, State};

use crate::firmware::RobotControlPort;
use crate::robot_interface::{RobotFeedback, RobotOperator, RobotState};

pub trait RobosimHardware {
    type Error;

    /// Return fresh timestamped measurements. The adapter fills `now_us` and
    /// the operator command before calling the controller.
    fn measurements(&mut self, logical_us: u64) -> Result<Input, Self::Error>;
    /// Schedule both motor outputs for `output.apply_at_us`.
    fn schedule_escs(&mut self, output: Output) -> Result<(), Self::Error>;
    fn heading_indicator(&mut self, on: bool) -> Result<(), Self::Error>;
    fn debug_output(&mut self, output: Output) -> Result<(), Self::Error>;
}

pub struct RobosimControlPort<H> {
    pub hardware: H,
    controller: Controller,
    latest_output: Output,
}

impl<H: RobosimHardware> RobosimControlPort<H> {
    pub fn new(hardware: H, config: Config) -> Result<Self, &'static str> {
        Ok(Self {
            hardware,
            controller: Controller::new(config)?,
            latest_output: Output::default(),
        })
    }

    pub fn latest_output(&self) -> Output {
        self.latest_output
    }
}

impl<H: RobosimHardware> RobotControlPort for RobosimControlPort<H> {
    type Error = H::Error;

    fn update(&mut self, command: RobotOperator) -> Result<RobotFeedback, Self::Error> {
        let mut input = self.hardware.measurements(command.timestamp_us)?;
        input.now_us = command.timestamp_us;
        input.operator = Operator {
            timestamp_us: command.timestamp_us,
            enable: command.enable,
            stop: command.stop,
            brake: command.brake,
            target_rpm: command.target_rpm,
            translation: command.translation,
            heading_trim_rad: command.heading_trim_rad,
            reference_id: command.reference_id,
            reference_heading_rad: command.reference_heading_rad,
            clear_fault: command.clear_fault,
        };
        let output = self.controller.update(input);
        self.hardware.schedule_escs(output)?;
        self.hardware.heading_indicator(output.indicator)?;
        self.hardware.debug_output(output)?;
        self.latest_output = output;
        Ok(RobotFeedback {
            state: map_state(output.state),
            spin_rpm: control_core::rad_s_to_rpm(output.diagnostics.spin_rad_s),
            phase_rad: output.diagnostics.phase_rad,
            spin_valid: output.diagnostics.spin_valid,
        })
    }

    fn predicted_phase_rad(&self) -> f32 {
        self.latest_output.diagnostics.predicted_phase_rad
    }
}

fn map_state(state: State) -> RobotState {
    match state {
        State::Disarmed => RobotState::Disarmed,
        State::Calibrating => RobotState::Calibrating,
        State::SpinUp => RobotState::SpinUp,
        State::Running => RobotState::Running,
        State::Stopping => RobotState::Stopping,
        State::Fault => RobotState::Fault,
        State::Recovering => RobotState::Recovering,
    }
}

