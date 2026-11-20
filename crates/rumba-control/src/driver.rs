//! Receiver-channel calibration and driver-intent state machine.

use crate::leds::LedMode;
use crate::failsafe::{validate_frame, FailsafeConfig, FailsafeReason, FailsafeState};
use crate::receiver::ReceiverFrame;
use crate::robot_interface::{RobotFeedback, RobotOperator};

pub const MAX_CHANNELS: usize = 16;
pub const MAX_AUX_SWITCHES: usize = 4;

/// A remappable two-position radio switch for optional LED content.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LedSwitchBinding {
    /// Zero-based receiver channel index.
    pub channel: usize,
    /// Mode selected while the mapped switch is ON. OFF returns to the default
    /// mode unless another mapped switch is ON.
    pub mode: LedMode,
}

/// Optional switch bindings are independent of the primary driving controls.
/// Leave entries `None` when a physical switch should have no software action.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AuxiliaryControlMap {
    pub led_switches: [Option<LedSwitchBinding>; MAX_AUX_SWITCHES],
}

impl Default for AuxiliaryControlMap {
    fn default() -> Self {
        Self { led_switches: [None; MAX_AUX_SWITCHES] }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChannelMap {
    /// Zero-based iBUS channel slots. Defaults assume the common AETR order.
    pub throttle: usize,
    pub translation_x: usize,
    pub translation_y: usize,
    pub enable: usize,
    pub invert_translation_x: bool,
    pub invert_translation_y: bool,
    pub invert_throttle: bool,
}

impl Default for ChannelMap {
    fn default() -> Self {
        Self {
            throttle: 2,       // CH3: left stick vertical in common AETR
            translation_x: 0,  // CH1: right stick horizontal
            translation_y: 1,  // CH2: right stick vertical
            enable: 4,         // CH5: configurable switch
            invert_translation_x: false,
            invert_translation_y: true,
            invert_throttle: false,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct DriverConfig {
    pub channels: ChannelMap,
    pub auxiliary: AuxiliaryControlMap,
    pub max_rpm: f32,
    pub translation_dead_zone: f32,
    pub stopped_threshold_rpm: f32,
    pub failsafe: FailsafeConfig,
}

impl Default for DriverConfig {
    fn default() -> Self {
        Self {
            channels: ChannelMap::default(),
            auxiliary: AuxiliaryControlMap::default(),
            max_rpm: 3000.0,
            translation_dead_zone: 0.05,
            stopped_threshold_rpm: 10.0,
            failsafe: FailsafeConfig::default(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MappedDriverOutput {
    pub operator: RobotOperator,
    pub failsafe: FailsafeState,
    /// Non-driving display selection; forced to the directional marker while
    /// stopped or in failsafe so optional graphics cannot distract the driver.
    pub led_mode: LedMode,
    /// True while the layer is requesting controlled stopping.
    pub stop_requested: bool,
}

pub struct DriverMapper {
    config: DriverConfig,
    stop_latched: bool,
    requires_toggle_cycle: bool,
    was_enabled: bool,
    virtual_front_heading_rad: f32,
    reference_id: u32,
}

impl DriverMapper {
    pub const fn new(config: DriverConfig) -> Self {
        Self {
            config,
            stop_latched: false,
            requires_toggle_cycle: false,
            was_enabled: false,
            virtual_front_heading_rad: 0.0,
            reference_id: 0,
        }
    }

    pub fn config(&self) -> DriverConfig {
        self.config
    }

    /// Called at the control-loop rate, not only when a new UART packet arrives.
    /// A missing/stale frame produces a fresh, safe Operator command so robosim
    /// can execute its controlled stop rather than seeing a stale timestamp.
    pub fn update(
        &mut self,
        frame: Option<&ReceiverFrame>,
        now_us: u64,
        feedback: RobotFeedback,
    ) -> MappedDriverOutput {
        let required = self
            .config
            .channels
            .throttle
            .max(self.config.channels.translation_x)
            .max(self.config.channels.translation_y)
            .max(self.config.channels.enable)
            .max(self.config.auxiliary.led_switches.iter().flatten().map(|binding| binding.channel).max().unwrap_or(0))
            .saturating_add(1);

        let frame = match validate_frame(frame, now_us, required, self.config.failsafe) {
            Ok(frame) => frame,
            Err(reason) => {
                self.requires_toggle_cycle = true;
                self.stop_latched = true;
                self.was_enabled = false;
                return self.stop_output(now_us, feedback, FailsafeState::Active(reason));
            }
        };

        let channels = &frame.channels_us;
        let enable_switch_on = channels[self.config.channels.enable] >= 1500;

        // After any receiver loss, the driver must first return the enable switch
        // to OFF. Returning RF packets with the switch still ON never re-arms.
        if self.requires_toggle_cycle {
            if !enable_switch_on {
                self.requires_toggle_cycle = false;
                self.stop_latched = true;
            }
            return self.stop_output(now_us, feedback, FailsafeState::Healthy);
        }

        if enable_switch_on
            && (!feedback.phase_rad.is_finite()
                || feedback.phase_rad.abs() > core::f32::consts::TAU)
        {
            self.requires_toggle_cycle = true;
            self.stop_latched = true;
            self.was_enabled = false;
            return self.stop_output(
                now_us,
                feedback,
                FailsafeState::Active(FailsafeReason::InvalidControllerPhase),
            );
        }

        if !enable_switch_on {
            if self.was_enabled || !feedback.is_stopped(self.config.stopped_threshold_rpm) {
                self.stop_latched = true;
            }
            self.was_enabled = false;
            if self.stop_latched {
                return self.stop_output(now_us, feedback, FailsafeState::Healthy);
            }
            return MappedDriverOutput {
                operator: RobotOperator::disarmed(now_us),
                failsafe: FailsafeState::Healthy,
                led_mode: LedMode::VirtualFront,
                stop_requested: false,
            };
        }

        if self.stop_latched {
            if !feedback.is_stopped(self.config.stopped_threshold_rpm) {
                return self.stop_output(now_us, feedback, FailsafeState::Healthy);
            }
            self.stop_latched = false;
        }

        if !self.was_enabled {
            self.virtual_front_heading_rad = normalize_angle(feedback.phase_rad);
            self.reference_id = self.reference_id.wrapping_add(1);
            if self.reference_id == 0 {
                self.reference_id = 1;
            }
        }

        self.was_enabled = true;
        let throttle_raw = channels[self.config.channels.throttle];
        let throttle = normalize_unipolar(throttle_raw, self.config.channels.invert_throttle);
        let mut x = normalize_bipolar(channels[self.config.channels.translation_x]);
        let mut y = normalize_bipolar(channels[self.config.channels.translation_y]);
        if self.config.channels.invert_translation_x {
            x = -x;
        }
        if self.config.channels.invert_translation_y {
            y = -y;
        }
        x = apply_dead_zone(x, self.config.translation_dead_zone);
        y = apply_dead_zone(y, self.config.translation_dead_zone);

        MappedDriverOutput {
            operator: RobotOperator {
                timestamp_us: now_us,
                enable: true,
                stop: false,
                brake: false,
                target_rpm: throttle * self.config.max_rpm.max(0.0),
                translation: [x, y],
                heading_trim_rad: 0.0,
                reference_id: self.reference_id,
                reference_heading_rad: self.virtual_front_heading_rad,
                clear_fault: false,
            },
            failsafe: FailsafeState::Healthy,
            led_mode: if self.is_spinning(feedback) {
                self.selected_led_mode(channels)
            } else {
                LedMode::VirtualFront
            },
            stop_requested: false,
        }
    }

    fn selected_led_mode(&self, channels: &[u16; MAX_CHANNELS]) -> LedMode {
        let mut selected = LedMode::VirtualFront;
        // Later bindings have priority if several switches are ON. This keeps
        // each switch independently configurable without hidden combination logic.
        for binding in self.config.auxiliary.led_switches.iter().flatten() {
            if channels[binding.channel] >= 1500 {
                selected = binding.mode;
            }
        }
        selected
    }

    fn is_spinning(&self, feedback: RobotFeedback) -> bool {
        feedback.spin_rpm.is_finite()
            && feedback.spin_rpm.abs() > self.config.stopped_threshold_rpm.max(0.0)
            && matches!(feedback.state, crate::robot_interface::RobotState::SpinUp | crate::robot_interface::RobotState::Running)
    }

    fn stop_output(
        &mut self,
        now_us: u64,
        feedback: RobotFeedback,
        failsafe: FailsafeState,
    ) -> MappedDriverOutput {
        let stopped = feedback.is_stopped(self.config.stopped_threshold_rpm);
        if stopped {
            self.stop_latched = false;
        }
        MappedDriverOutput {
            operator: if stopped {
                RobotOperator::disarmed(now_us)
            } else {
                RobotOperator::controlled_stop(now_us)
            },
            failsafe,
            led_mode: LedMode::VirtualFront,
            stop_requested: !stopped,
        }
    }
}

fn normalize_unipolar(value_us: u16, inverted: bool) -> f32 {
    let normalized = ((value_us as f32 - 1000.0) / 1000.0).clamp(0.0, 1.0);
    if inverted { 1.0 - normalized } else { normalized }
}

fn normalize_bipolar(value_us: u16) -> f32 {
    ((value_us as f32 - 1500.0) / 500.0).clamp(-1.0, 1.0)
}

fn normalize_angle(mut angle: f32) -> f32 {
    let tau = core::f32::consts::TAU;
    while angle < 0.0 {
        angle += tau;
    }
    while angle >= tau {
        angle -= tau;
    }
    angle
}

fn apply_dead_zone(value: f32, dead_zone: f32) -> f32 {
    let dead_zone = dead_zone.clamp(0.0, 0.95);
    if value.abs() <= dead_zone {
        0.0
    } else {
        value.signum() * ((value.abs() - dead_zone) / (1.0 - dead_zone))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot_interface::RobotState;

    fn frame(channels: [u16; 5], time: u64) -> ReceiverFrame {
        let mut values = [1500; MAX_CHANNELS];
        values[..5].copy_from_slice(&channels);
        ReceiverFrame { channels_us: values, channel_count: 5, received_at_us: time }
    }

    fn running_feedback() -> RobotFeedback {
        RobotFeedback { state: RobotState::Running, spin_rpm: 3000.0, phase_rad: 0.0 }
    }

    #[test]
    fn maps_aetr_controls_to_rpm_and_translation() {
        let mut mapper = DriverMapper::new(DriverConfig::default());
        let f = frame([2000, 1000, 1500, 1500, 2000], 10_000);
        let output = mapper.update(Some(&f), 10_001, RobotFeedback::default());
        assert_eq!(output.operator.target_rpm, 1500.0);
        assert_eq!(output.operator.translation, [1.0, 1.0]);
        assert!(output.operator.enable);
    }

    #[test]
    fn toggle_off_requests_controlled_stop_before_disarming() {
        let mut mapper = DriverMapper::new(DriverConfig::default());
        let on = frame([1500, 1500, 1800, 1500, 2000], 10_000);
        let _ = mapper.update(Some(&on), 10_001, RobotFeedback::default());
        let off = frame([1500, 1500, 1800, 1500, 1000], 20_000);
        let stopping = mapper.update(Some(&off), 20_001, running_feedback());
        assert!(stopping.operator.enable);
        assert!(stopping.operator.stop);
        assert_eq!(stopping.operator.target_rpm, 0.0);
        let stopped_feedback = RobotFeedback { state: RobotState::Stopping, spin_rpm: 0.0, phase_rad: 0.0 };
        let stopped = mapper.update(Some(&off), 21_000, stopped_feedback);
        assert!(!stopped.operator.enable);
        assert!(!stopped.operator.stop);
    }

    #[test]
    fn receiver_loss_stops_and_requires_switch_off_before_rearming() {
        let mut mapper = DriverMapper::new(DriverConfig::default());
        let on = frame([1500, 1500, 1800, 1500, 2000], 10_000);
        let _ = mapper.update(Some(&on), 10_001, RobotFeedback::default());
        let lost = mapper.update(None, 20_000, running_feedback());
        assert!(lost.operator.enable && lost.operator.stop);
        assert_eq!(lost.failsafe, FailsafeState::Active(FailsafeReason::NoFrame));

        let returned_on = frame([1500, 1500, 1800, 1500, 2000], 21_000);
        let held = mapper.update(Some(&returned_on), 21_001, running_feedback());
        assert!(held.operator.stop);

        let returned_off = frame([1500, 1500, 1800, 1500, 1000], 22_000);
        let _ = mapper.update(Some(&returned_off), 22_001, running_feedback());
        let stopped_feedback = RobotFeedback { state: RobotState::Stopping, spin_rpm: 0.0, phase_rad: 0.0 };
        let disarmed = mapper.update(Some(&returned_off), 23_000, stopped_feedback);
        assert!(!disarmed.operator.enable);
        let armed_again = frame([1500, 1500, 1800, 1500, 2000], 24_000);
        let armed = mapper.update(Some(&armed_again), 24_001, RobotFeedback::default());
        assert!(armed.operator.enable);
    }

    #[test]
    fn centered_translation_is_spin_in_place() {
        let mut mapper = DriverMapper::new(DriverConfig::default());
        let f = frame([1500, 1500, 2000, 1500, 2000], 10_000);
        let output = mapper.update(Some(&f), 10_001, RobotFeedback::default());
        assert_eq!(output.operator.target_rpm, 3000.0);
        assert_eq!(output.operator.translation, [0.0, 0.0]);
    }

    #[test]
    fn auxiliary_led_switches_are_remappable_and_only_apply_while_enabled() {
        let mut config = DriverConfig::default();
        config.auxiliary.led_switches[0] = Some(LedSwitchBinding {
            channel: 5,
            mode: LedMode::SponsorLoop,
        });
        let mut mapper = DriverMapper::new(config);

        let mut values = [1500; MAX_CHANNELS];
        values[4] = 2000; // spin-enable
        values[5] = 2000; // auxiliary LED switch
        let on = ReceiverFrame { channels_us: values, channel_count: 6, received_at_us: 10_000 };
        let active = mapper.update(
            Some(&on),
            10_001,
            RobotFeedback { state: RobotState::Running, spin_rpm: 1200.0, phase_rad: 0.0 },
        );
        assert_eq!(active.led_mode, LedMode::SponsorLoop);

        values[4] = 1000;
        let off = ReceiverFrame { channels_us: values, channel_count: 6, received_at_us: 11_000 };
        let stopped = mapper.update(Some(&off), 11_001, RobotFeedback::default());
        assert_eq!(stopped.led_mode, LedMode::VirtualFront);
    }

    #[test]
    fn virtual_front_is_latched_on_enable_and_kept_stable_while_spinning() {
        let mut mapper = DriverMapper::new(DriverConfig::default());
        let on = frame([1500, 1500, 1800, 1500, 2000], 10_000);
        let first = mapper.update(
            Some(&on),
            10_001,
            RobotFeedback { state: RobotState::Running, spin_rpm: 1200.0, phase_rad: 1.1 },
        );
        assert_eq!(first.operator.reference_heading_rad, 1.1);
        assert_eq!(first.operator.reference_id, 1);

        let next = frame([1500, 1500, 1800, 1500, 2000], 11_000);
        let later = mapper.update(
            Some(&next),
            11_001,
            RobotFeedback { state: RobotState::Running, spin_rpm: 1600.0, phase_rad: 2.4 },
        );
        assert_eq!(later.operator.reference_heading_rad, 1.1);
        assert_eq!(later.operator.reference_id, 1);
    }

    #[test]
    fn invalid_phase_while_enabled_enters_failsafe() {
        let mut mapper = DriverMapper::new(DriverConfig::default());
        let on = frame([1500, 1500, 1800, 1500, 2000], 10_000);
        let output = mapper.update(
            Some(&on),
            10_001,
            RobotFeedback { state: RobotState::Running, spin_rpm: 1200.0, phase_rad: f32::NAN },
        );
        assert_eq!(output.failsafe, FailsafeState::Active(FailsafeReason::InvalidControllerPhase));
        assert!(output.operator.stop);
        assert_eq!(output.operator.translation, [0.0, 0.0]);
    }
}
