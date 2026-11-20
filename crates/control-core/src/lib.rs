#![no_std]
//! The only controller implementation. All time is supplied by the caller; no truth inputs.
use core::f32::consts::{PI, TAU};
use serde::{Deserialize, Serialize};
pub mod tof;
use tof::{Heading, TofConfig};
pub const G: f32 = 9.80665;
/// Half of the 1 kHz high-g sample period. Independent chips need not share a clock.
/// Larger skew loses the common-acceleration cancellation needed for heading tracking.
pub const ACCEL_PAIR_MAX_SKEW_US: u64 = 500;
pub const fn rpm_to_rad_s(rpm: f32) -> f32 {
    rpm * TAU / 60.0
}
pub const fn rad_s_to_rpm(w: f32) -> f32 {
    w * 60.0 / TAU
}
pub const fn degrees_to_rad(d: f32) -> f32 {
    d * PI / 180.0
}
pub const fn pounds_to_kg(lb: f32) -> f32 {
    lb * 0.45359237
}
pub fn wrap(a: f32) -> f32 {
    a - TAU * libm::floorf((a + PI) / TAU)
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default, deny_unknown_fields)]
pub struct Config {
    pub target_rpm: f32,
    pub max_rpm: f32,
    pub kp: f32,
    pub ki: f32,
    pub feedforward_per_rad_s: f32,
    pub max_output: f32,
    pub translation_limit: f32,
    pub accel_radius_m: f32,
    pub gyro_enabled: bool,
    pub tof: Option<TofConfig>,
    pub high_g_crossover_rad_s: f32,
    pub gyro_bias_rad_s: f32,
    pub gyro_scale: f32,
    pub accel_scale: f32,
    pub filter_tau_s: f32,
    pub actuation_delay_s: f32,
    pub control_period_s: f32,
    pub sensor_timeout_us: u64,
    pub receiver_timeout_us: u64,
    pub minimum_voltage: f32,
    pub phase_uncertainty_rate: f32,
    pub phase_limit_rad: f32,
    pub stopped_rad_s: f32,
    pub calibration_us: u64,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            target_rpm: 3000.0,
            max_rpm: 3000.0,
            kp: 0.003,
            ki: 0.004,
            feedforward_per_rad_s: 0.0033,
            max_output: 0.95,
            translation_limit: 0.25,
            accel_radius_m: 0.02,
            gyro_enabled: true,
            tof: None,
            high_g_crossover_rad_s: 25.0,
            gyro_bias_rad_s: 0.0,
            gyro_scale: 1.0,
            accel_scale: 1.0,
            filter_tau_s: 0.001,
            actuation_delay_s: 0.002,
            control_period_s: 0.001,
            sensor_timeout_us: 20_000,
            receiver_timeout_us: 100_000,
            minimum_voltage: 9.0,
            phase_uncertainty_rate: 0.015,
            phase_limit_rad: 0.5,
            stopped_rad_s: 1.0,
            calibration_us: 100_000,
        }
    }
}
impl Config {
    pub fn validate(&self) -> Result<(), &'static str> {
        let vals = [
            self.target_rpm,
            self.max_rpm,
            self.kp,
            self.ki,
            self.feedforward_per_rad_s,
            self.max_output,
            self.translation_limit,
            self.accel_radius_m,
            self.gyro_bias_rad_s,
            self.high_g_crossover_rad_s,
            self.gyro_scale,
            self.accel_scale,
            self.filter_tau_s,
            self.actuation_delay_s,
            self.control_period_s,
            self.minimum_voltage,
            self.phase_uncertainty_rate,
            self.phase_limit_rad,
            self.stopped_rad_s,
        ];
        if vals.iter().any(|v| !v.is_finite()) {
            return Err("controller values must be finite");
        }
        if self.tof.is_some_and(|t| !t.valid()) || self.high_g_crossover_rad_s <= 0.0 {
            return Err("invalid ToF or high-g calibration");
        }
        if self.target_rpm < 0.0 || self.max_rpm < self.target_rpm || self.max_rpm > 3000.0 {
            return Err("RPM must satisfy 0 <= target <= max <= 3000");
        }
        if !(0.0..=1.0).contains(&self.max_output)
            || !(0.0..=0.5).contains(&self.translation_limit)
            || self.accel_radius_m <= 0.0
            || self.gyro_scale <= 0.0
            || self.accel_scale <= 0.0
            || self.kp < 0.0
            || self.ki < 0.0
            || self.filter_tau_s < 0.0
            || self.actuation_delay_s < 0.0
            || self.control_period_s <= 0.0
            || self.phase_uncertainty_rate <= 0.0
            || self.phase_limit_rad <= 0.0
            || self.stopped_rad_s <= 0.0
            || self.minimum_voltage <= 0.0
            || self.sensor_timeout_us == 0
            || self.receiver_timeout_us == 0
        {
            return Err("invalid controller gains, scales, timeouts, or limits");
        }
        Ok(())
    }
}
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Measurement {
    pub acquired_us: u64,
    pub delivered_us: u64,
    pub value: [f32; 3],
    pub valid: bool,
    pub saturated: bool,
}
impl Measurement {
    pub fn fresh(&self, now: u64, timeout: u64) -> bool {
        self.valid
            && !self.saturated
            && self.delivered_us <= now
            && self.acquired_us <= self.delivered_us
            && now.saturating_sub(self.acquired_us) <= timeout
            && self.value.iter().all(|v| v.is_finite())
    }
}
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
#[serde(default, deny_unknown_fields)]
pub struct Operator {
    pub timestamp_us: u64,
    pub enable: bool,
    pub stop: bool,
    pub brake: bool,
    pub target_rpm: f32,
    pub translation: [f32; 2],
    pub heading_trim_rad: f32,
    /// Increment for each deliberate alignment. Alignment is accepted only at rest.
    pub reference_id: u32,
    pub reference_heading_rad: f32,
    pub clear_fault: bool,
}
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Input {
    pub now_us: u64,
    pub gyro: Measurement,
    pub accel_positive: Measurement,
    pub accel_negative: Measurement,
    pub encoders: [Measurement; 2],
    pub battery_voltage: Measurement,
    pub tof: Measurement,
    pub operator: Operator,
    pub transport_ok: bool,
}
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, Eq)]
pub enum State {
    #[default]
    Disarmed,
    Calibrating,
    SpinUp,
    Running,
    Stopping,
    Fault,
    /// Spin is being recovered; translation waits for post-impact wall fixes.
    Recovering,
}
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, Eq)]
pub enum Fault {
    #[default]
    None,
    Receiver,
    Sensor,
    Undervoltage,
    Transport,
    Time,
    Overspeed,
}
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, Eq)]
pub enum DriveMode {
    #[default]
    Disarm,
    Coast,
    Brake,
    Drive,
}
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Diagnostics {
    pub spin_rad_s: f32,
    pub spin_valid: bool,
    pub phase_rad: f32,
    pub predicted_phase_rad: f32,
    pub phase_uncertainty_rad: f32,
    pub reference_confident: bool,
    pub gyro_used: bool,
    pub tof_used: bool,
    pub tof_corrections: u32,
    pub tof_residual_m: f32,
    pub tof_heading_age_us: u64,
    pub tof_position_m: [f32; 2],
    pub sensor_age_us: u64,
    pub integral: f32,
    pub spin_duty: f32,
    pub translation_duty: f32,
    pub saturated: bool,
    pub fault: Fault,
}
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Output {
    pub timestamp_us: u64,
    pub apply_at_us: u64,
    pub motors: [f32; 2],
    pub mode: DriveMode,
    pub indicator: bool,
    pub state: State,
    pub diagnostics: Diagnostics,
}
const RECOVERY_STABLE_SAMPLES: u8 = 3;
const RECOVERY_RAMP_US: u64 = 50_000;
// Retain a local phase search seed briefly while wall rays are occluded.
// This does not extend the 100 ms movement gate or permit a global relocalization.
const HEADING_REACQUIRE_US: u64 = 2_000_000;
// A rectangular arena repeats after a half-turn (a square after a quarter-turn).
// Keep the recovery search strictly inside one square-arena heading branch.
const RECOVERY_PHASE_GATE: f32 = core::f32::consts::FRAC_PI_4 - 0.01;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Recovery {
    started_us: Option<u64>,
    seed_verified_us: Option<u64>,
    settling_since_us: Option<u64>,
    blind_since_us: Option<u64>,
    phase_before: f32,
    uncertainty_before: f32,
    seed_trusted: bool,
    good_samples: u8,
    last_measured: f32,
    spin_limit: f32,
    fixes: u8,
    released_us: Option<u64>,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct Controller {
    config: Config,
    state: State,
    fault: Fault,
    arming_ready: bool,
    previous_us: Option<u64>,
    last_sample_us: Option<u64>,
    omega: f32,
    previous_omega: f32,
    phase: f32,
    uncertainty: f32,
    reference: bool,
    reference_id: u32,
    integral: f32,
    calibration_start: u64,
    calibration_sum: f32,
    calibration_count: u32,
    calibration_sample_us: Option<u64>,
    gyro_bias: f32,
    accel_radial_bias: f32,
    accel_tangent_bias: f32,
    calibration_radial_sum: f32,
    calibration_tangent_sum: f32,
    tof_heading: Heading,
    // Recording metadata stores only freshly reset controllers. Runtime recovery
    // is not a checkpoint and adds no fields to schema 3/4's metadata layout.
    #[serde(skip)]
    recovery: Recovery,
}
impl Controller {
    pub fn new(config: Config) -> Result<Self, &'static str> {
        config.validate()?;
        Ok(Self {
            config,
            state: State::Disarmed,
            fault: Fault::None,
            arming_ready: false,
            previous_us: None,
            last_sample_us: None,
            omega: 0.0,
            previous_omega: 0.0,
            phase: 0.0,
            uncertainty: PI,
            reference: false,
            reference_id: 0,
            integral: 0.0,
            calibration_start: 0,
            calibration_sum: 0.0,
            calibration_count: 0,
            calibration_sample_us: None,
            gyro_bias: config.gyro_bias_rad_s,
            accel_radial_bias: 0.0,
            accel_tangent_bias: 0.0,
            calibration_radial_sum: 0.0,
            calibration_tangent_sum: 0.0,
            tof_heading: Heading::default(),
            recovery: Recovery::default(),
        })
    }
    pub fn config(&self) -> Config {
        self.config
    }
    fn start_recovery(&mut self, now: u64) {
        let continuing = self.state == State::Recovering;
        let settling_since_us =
            if continuing && self.recovery.good_samples < RECOVERY_STABLE_SAMPLES {
                self.recovery.settling_since_us.unwrap_or(now)
            } else {
                now
            };
        let seed = if continuing {
            self.recovery.seed_trusted
        } else {
            self.reference
                && self.uncertainty < self.config.phase_limit_rad
                && self.config.tof.is_some()
                && self
                    .tof_heading
                    .last_fix_us
                    .is_none_or(|t| now.saturating_sub(t) <= HEADING_REACQUIRE_US)
        };
        let seed_verified_us = if continuing {
            self.recovery.seed_verified_us
        } else {
            Some(self.tof_heading.last_fix_us.unwrap_or(now))
        };
        self.recovery = Recovery {
            started_us: Some(now),
            seed_verified_us,
            settling_since_us: Some(settling_since_us),
            blind_since_us: self.last_sample_us,
            phase_before: wrap(self.phase + self.omega * self.config.filter_tau_s),
            uncertainty_before: self.uncertainty,
            seed_trusted: seed,
            ..Recovery::default()
        };
        self.tof_heading.restart_scan(now);
        self.reference = false;
        self.integral = 0.0;
        self.state = State::Recovering;
    }
    fn start_heading_recovery(&mut self, now: u64, measured: f32) {
        // Rate acquisition is still healthy: preserve the PI and spin output.
        // Only translation and the old ray window need to be held/reset.
        self.recovery = Recovery {
            started_us: Some(now),
            seed_verified_us: self.tof_heading.last_fix_us,
            seed_trusted: self.reference && self.uncertainty < RECOVERY_PHASE_GATE,
            good_samples: RECOVERY_STABLE_SAMPLES,
            last_measured: measured,
            spin_limit: self.config.max_output,
            ..Recovery::default()
        };
        self.tof_heading.restart_scan(now);
        self.reference = false;
        self.state = State::Recovering;
    }
    pub fn update(&mut self, input: Input) -> Output {
        let c = self.config;
        let now = input.now_us;
        let backwards = self.previous_us.is_some_and(|t| now <= t);
        let dt = self
            .previous_us
            .map_or(c.control_period_s, |t| now.saturating_sub(t) as f32 * 1e-6);
        self.previous_us = Some(now);
        let gyro_ok = c.gyro_enabled && input.gyro.fresh(now, c.sensor_timeout_us);
        let accel_ok = input.accel_positive.fresh(now, c.sensor_timeout_us)
            && input.accel_negative.fresh(now, c.sensor_timeout_us)
            && input
                .accel_positive
                .acquired_us
                .abs_diff(input.accel_negative.acquired_us)
                <= ACCEL_PAIR_MAX_SKEW_US;
        // Diametrically opposed, identically oriented accelerometers cancel common translational
        // acceleration. Tangential angular acceleration is orthogonal to their radial x axis.
        let radial = (input.accel_negative.value[0] - input.accel_positive.value[0])
            / (2.0 * c.accel_radius_m * c.accel_scale);
        let radial = radial - self.accel_radial_bias;
        let tangent = (input.accel_positive.value[1] - input.accel_negative.value[1])
            / (2.0 * c.accel_radius_m * c.accel_scale)
            - self.accel_tangent_bias;
        let gyro_rate = (input.gyro.value[2] - self.gyro_bias) / c.gyro_scale;
        let use_gyro = gyro_ok;
        let spin_valid = gyro_ok
            || (accel_ok
                && radial
                    >= if c.gyro_enabled {
                        0.0
                    } else {
                        -c.high_g_crossover_rad_s * c.high_g_crossover_rad_s
                    });
        let sample = if use_gyro {
            input.gyro
        } else {
            // The differential is centered between the two real acquisition times.
            // Preserve both original timestamps in Input and the recording.
            let earlier = input
                .accel_positive
                .acquired_us
                .min(input.accel_negative.acquired_us);
            let skew = input
                .accel_positive
                .acquired_us
                .abs_diff(input.accel_negative.acquired_us);
            Measurement {
                acquired_us: earlier + skew / 2,
                ..input.accel_positive
            }
        };
        let spin_valid = spin_valid && self.last_sample_us.is_none_or(|t| sample.acquired_us >= t);
        let measured = if use_gyro {
            gyro_rate
        } else if !c.gyro_enabled
            && radial.abs() < c.high_g_crossover_rad_s * c.high_g_crossover_rad_s
        {
            // Signed tangential differential supplies angular acceleration below the noisy
            // centripetal-rate floor. Disabled alignment/calibration is explicitly at rest.
            if !input.operator.enable && self.state == State::Disarmed {
                0.0
            } else {
                (self.omega + tangent * dt).max(0.0)
            }
        } else {
            libm::sqrtf(radial.max(0.0))
        };
        let new_sample = spin_valid && self.last_sample_us != Some(sample.acquired_us);
        let recent_spin = self
            .last_sample_us
            .is_some_and(|t| now.saturating_sub(t) <= c.sensor_timeout_us);
        let shock = new_sample
            && self.omega > c.high_g_crossover_rad_s
            // Near the high-g crossover, quantization/noise can exceed a 15%
            // relative rate change. Require an absolute drop too, so ordinary
            // spin-up cannot repeatedly restart heading recovery.
            && (self.omega - measured > (self.omega * 0.15).max(rpm_to_rad_s(c.max_rpm) * 0.05)
                || tangent.abs()
                    > rpm_to_rad_s(c.max_rpm) * 1.1 / (c.sensor_timeout_us as f32 * 1e-6));
        if !c.gyro_enabled
            && c.tof.is_some()
            && input.operator.enable
            && !input.operator.stop
            && recent_spin
            && matches!(
                self.state,
                State::SpinUp | State::Running | State::Recovering
            )
            && ((!spin_valid && self.recovery.blind_since_us.is_none())
                || (shock
                    && (self.state != State::Recovering
                        || self.recovery.good_samples >= RECOVERY_STABLE_SAMPLES)))
        {
            self.start_recovery(now);
        }
        let mut restarted_rate = false;
        // High-g supplies positive rate magnitude. A short clipped interval is
        // bridged only for spin recovery; it never supplies movement confidence.
        if spin_valid && self.last_sample_us != Some(sample.acquired_us) {
            let sample_dt = self.last_sample_us.map_or(c.control_period_s, |t| {
                sample.acquired_us.saturating_sub(t) as f32 * 1e-6
            });
            let a = sample_dt / (c.filter_tau_s + sample_dt).max(1e-6);
            if self.state == State::Recovering {
                if let Some(at) = self.recovery.blind_since_us.take() {
                    let gap = sample.acquired_us.saturating_sub(at) as f32 * 1e-6;
                    // Positive spin can advance anywhere in [0, max_rate * gap]
                    // while both H3 readings are untrusted. Center that interval;
                    // do not assume the impact deceleration was linear. ToF must
                    // verify it inside one arena branch before movement returns.
                    let half_span = rpm_to_rad_s(c.max_rpm) * 1.1 * gap * 0.5;
                    let at_sample = self.recovery.phase_before + half_span;
                    // Restore the phase at the actual acquisition time. The
                    // normal integration below and lag compensation then apply.
                    self.phase = wrap(
                        at_sample
                            - measured * c.filter_tau_s
                            - 0.5 * (measured + self.previous_omega) * dt,
                    );
                    self.uncertainty = (self.recovery.uncertainty_before + half_span).min(PI);
                    self.recovery.seed_trusted &= self.uncertainty < RECOVERY_PHASE_GATE;
                    self.omega = measured;
                    restarted_rate = true;
                } else {
                    self.omega += a.clamp(0.0, 1.0) * (measured - self.omega);
                }
                let stable = self.recovery.good_samples == 0
                    || (measured - self.recovery.last_measured).abs()
                        <= rpm_to_rad_s(c.max_rpm) * 0.03;
                let was_stable = self.recovery.good_samples == RECOVERY_STABLE_SAMPLES;
                self.recovery.good_samples = if stable {
                    (self.recovery.good_samples + 1).min(RECOVERY_STABLE_SAMPLES)
                } else {
                    1
                };
                self.recovery.last_measured = measured;
                if was_stable && !stable {
                    self.recovery.settling_since_us = Some(now);
                }
                if self.recovery.good_samples == RECOVERY_STABLE_SAMPLES && !was_stable {
                    self.recovery.settling_since_us = None;
                    self.tof_heading.restart_scan(now);
                    self.recovery.fixes = 0;
                }
            } else if !c.gyro_enabled
                && radial.abs() < c.high_g_crossover_rad_s * c.high_g_crossover_rad_s
            {
                self.omega = measured;
            } else {
                self.omega += a.clamp(0.0, 1.0) * (measured - self.omega);
            }
            self.last_sample_us = Some(sample.acquired_us);
        }
        self.phase = wrap(self.phase + 0.5 * (self.omega + self.previous_omega) * dt);
        if spin_valid {
            self.uncertainty = (self.uncertainty + c.phase_uncertainty_rate * dt).min(PI);
        } else {
            self.uncertainty = if self.state == State::Recovering {
                (self.uncertainty + rpm_to_rad_s(c.max_rpm) * 1.1 * dt).min(PI)
            } else {
                PI
            };
            self.reference = false;
        }
        let angular_acceleration = if dt > 0.0 && !restarted_rate {
            (self.omega - self.previous_omega) / dt
        } else {
            0.0
        };
        self.previous_omega = self.omega;
        let op = input.operator;
        if !op.enable {
            self.arming_ready = true;
        }
        if dt > c.sensor_timeout_us as f32 * 1e-6 {
            self.reference = false;
            self.uncertainty = PI;
            self.recovery.seed_trusted = false;
        }
        if op.reference_id != 0
            && op.reference_id != self.reference_id
            && op.reference_heading_rad.is_finite()
            && spin_valid
            && self.omega.abs() < c.stopped_rad_s
            && !op.enable
        {
            self.phase = wrap(op.reference_heading_rad);
            self.uncertainty = 0.01;
            self.reference = true;
            self.reference_id = op.reference_id;
            self.tof_heading = Heading::default();
            self.recovery = Recovery::default();
        }
        // The phase model is planar and assumes positive spin. Observed roll/pitch or
        // negative low-speed rotation cannot preserve an arena heading reference.
        if c.gyro_enabled
            && input.gyro.valid
            && input.gyro.delivered_us <= now
            && now.saturating_sub(input.gyro.acquired_us) <= c.sensor_timeout_us
            && (input.gyro.value[0].abs() > 0.5
                || input.gyro.value[1].abs() > 0.5
                || (gyro_ok && gyro_rate < -c.stopped_rad_s))
        {
            self.reference = false;
            self.uncertainty = PI;
        }
        let lag = c.filter_tau_s + now.saturating_sub(sample.acquired_us) as f32 * 1e-6;
        if let Some(tc) = c.tof {
            if spin_valid
                && self.reference
                && matches!(self.state, State::SpinUp | State::Running)
                && self.tof_heading.last_fix_us.is_some()
                && !self.tof_heading.fresh(now, tc)
            {
                self.start_heading_recovery(now, measured);
            }
            if self.state == State::Recovering {
                if self.recovery.fixes < RECOVERY_STABLE_SAMPLES {
                    self.recovery.seed_trusted &= self
                        .recovery
                        .seed_verified_us
                        .is_some_and(|t| now.saturating_sub(t) <= HEADING_REACQUIRE_US)
                        && (self.recovery.blind_since_us.is_some()
                            || self.uncertainty < RECOVERY_PHASE_GATE);
                } else {
                    self.recovery.seed_trusted &= self
                        .tof_heading
                        .last_fix_us
                        .is_some_and(|t| now.saturating_sub(t) <= HEADING_REACQUIRE_US)
                        && self.uncertainty < RECOVERY_PHASE_GATE;
                    // A new occlusion requires three fresh fits again.
                    if !self.tof_heading.fresh(now, tc) {
                        self.recovery.fixes = 0;
                        self.tof_heading.restart_scan(now);
                    }
                }
                if !self.recovery.seed_trusted {
                    self.uncertainty = PI;
                }
            }
            let recovering_heading = self.state == State::Recovering
                && self.recovery.seed_trusted
                && self.recovery.good_samples >= RECOVERY_STABLE_SAMPLES;
            if spin_valid
                && (self.reference || recovering_heading)
                && let Some(correction) = self.tof_heading.observe(
                    input.tof,
                    now,
                    wrap(self.phase + self.omega * lag),
                    self.omega,
                    if recovering_heading || self.tof_heading.corrections == 0 {
                        // First acquisition must also bridge low-speed high-g
                        // integration error, within the manually selected branch.
                        TofConfig {
                            phase_gate_rad: RECOVERY_PHASE_GATE,
                            ..tc
                        }
                    } else {
                        tc
                    },
                )
            {
                self.phase = wrap(self.phase + correction);
                self.uncertainty = 0.03;
                self.recovery.fixes = (self.recovery.fixes + 1).min(RECOVERY_STABLE_SAMPLES);
                if self.recovery.fixes == RECOVERY_STABLE_SAMPLES {
                    self.recovery.seed_verified_us = Some(now);
                }
            }
            // Low-speed stopping can reverse below the high-g sign/noise floor.
            // A stopped/disarmed restart needs a new physical alignment once
            // its wall reference is stale; spinning holdover applies only above.
            if !matches!(
                self.state,
                State::SpinUp | State::Running | State::Recovering
            ) && self.tof_heading.last_fix_us.is_some()
                && !self.tof_heading.fresh(now, tc)
            {
                self.reference = false;
                self.uncertainty = PI;
            }
        }
        let tof_ready = c.tof.is_none_or(|tc| {
            self.tof_heading.fresh(now, tc) && self.recovery.fixes >= RECOVERY_STABLE_SAMPLES
        });
        if self.uncertainty >= c.phase_limit_rad {
            self.reference = false;
        }
        let receiver_ok = op.timestamp_us <= now
            && now.saturating_sub(op.timestamp_us) <= c.receiver_timeout_us
            && op.translation.iter().all(|v| v.is_finite())
            && op.target_rpm.is_finite()
            && op.heading_trim_rad.is_finite();
        let voltage_ok = input.battery_voltage.fresh(now, c.sensor_timeout_us)
            && input.battery_voltage.value[0] >= c.minimum_voltage;
        let sensor_grace = self.state == State::Recovering
            && self
                .last_sample_us
                .is_some_and(|t| now.saturating_sub(t) <= c.sensor_timeout_us)
            && (self.recovery.good_samples >= RECOVERY_STABLE_SAMPLES
                || self
                    .recovery
                    .started_us
                    .is_some_and(|t| now.saturating_sub(t) <= c.sensor_timeout_us));
        if self.state == State::Fault
            && !op.enable
            && op.clear_fault
            && receiver_ok
            && voltage_ok
            && spin_valid
            && input.transport_ok
            && !backwards
        {
            self.fault = Fault::None;
            self.state = State::Disarmed;
            self.integral = 0.0;
        }
        let fault = if backwards {
            Fault::Time
        } else if !input.transport_ok {
            Fault::Transport
        } else if !receiver_ok {
            Fault::Receiver
        } else if !voltage_ok {
            Fault::Undervoltage
        } else if (!spin_valid && !sensor_grace)
            || (self.state == State::Recovering
                && self
                    .recovery
                    .settling_since_us
                    .is_some_and(|t| now.saturating_sub(t) > c.sensor_timeout_us))
        {
            Fault::Sensor
        } else if self.omega > rpm_to_rad_s(c.max_rpm) * 1.1 {
            Fault::Overspeed
        } else {
            Fault::None
        };
        if fault != Fault::None && (op.enable || self.state != State::Disarmed) {
            self.state = State::Fault;
            self.fault = fault;
            self.integral = 0.0;
            self.recovery = Recovery::default();
        }
        if self.state != State::Fault {
            if !op.enable {
                self.state = State::Disarmed;
                self.integral = 0.0;
                self.recovery = Recovery::default();
            } else if op.stop {
                self.state = State::Stopping;
            } else if self.state == State::Disarmed && self.arming_ready {
                self.arming_ready = false;
                self.state = State::Calibrating;
                self.calibration_start = now;
                self.calibration_sum = 0.0;
                self.calibration_radial_sum = 0.0;
                self.calibration_tangent_sum = 0.0;
                self.calibration_count = 0;
                self.calibration_sample_us = None;
            }
            if self.state == State::Calibrating {
                let at_rest = if c.gyro_enabled {
                    gyro_ok && gyro_rate.abs() < c.stopped_rad_s
                } else {
                    accel_ok && radial.abs() < c.high_g_crossover_rad_s * c.high_g_crossover_rad_s
                };
                if at_rest {
                    if sample.acquired_us >= self.calibration_start
                        && self.calibration_sample_us != Some(sample.acquired_us)
                    {
                        self.calibration_sample_us = Some(sample.acquired_us);
                        if c.gyro_enabled {
                            self.calibration_sum += input.gyro.value[2];
                        }
                        self.calibration_radial_sum += radial + self.accel_radial_bias;
                        self.calibration_tangent_sum += tangent + self.accel_tangent_bias;
                        self.calibration_count += 1;
                    }
                    if now.saturating_sub(self.calibration_start) >= c.calibration_us
                        && self.calibration_count >= 10
                    {
                        if c.gyro_enabled {
                            self.gyro_bias = self.calibration_sum / self.calibration_count as f32;
                        } else {
                            self.accel_radial_bias =
                                self.calibration_radial_sum / self.calibration_count as f32;
                            self.accel_tangent_bias =
                                self.calibration_tangent_sum / self.calibration_count as f32;
                            self.omega = 0.0;
                            self.previous_omega = 0.0;
                        }
                        self.state = State::SpinUp;
                    }
                } else {
                    self.calibration_start = now;
                    self.calibration_sum = 0.0;
                    self.calibration_radial_sum = 0.0;
                    self.calibration_tangent_sum = 0.0;
                    self.calibration_count = 0;
                    self.calibration_sample_us = None;
                }
            }
        }
        let target = rpm_to_rad_s(op.target_rpm.clamp(0.0, c.max_rpm));
        let recovery_drive = self.state == State::Recovering
            && spin_valid
            && self.recovery.good_samples >= RECOVERY_STABLE_SAMPLES;
        if recovery_drive {
            self.recovery.spin_limit = (self.recovery.spin_limit
                + c.max_output * dt / (RECOVERY_RAMP_US as f32 * 1e-6))
                .min(c.max_output);
            let heading_ready = self.recovery.seed_trusted
                && self.recovery.fixes >= RECOVERY_STABLE_SAMPLES
                && tof_ready;
            if self.recovery.spin_limit >= c.max_output
                && (target - self.omega).abs() <= (target * 0.1).max(c.stopped_rad_s)
                && (heading_ready || !self.recovery.seed_trusted)
            {
                self.state = State::Running;
                self.reference = heading_ready;
                self.recovery.started_us = None;
                self.recovery.released_us = heading_ready.then_some(now);
            }
        }
        let mut spin = 0.0;
        let mut translation = 0.0;
        let mut saturated = false;
        let mut mode = DriveMode::Disarm;
        let age = now.saturating_sub(sample.acquired_us);
        let alpha = angular_acceleration;
        let measurement_lag = c.filter_tau_s + age as f32 * 1e-6;
        let current_phase = wrap(self.phase + self.omega * measurement_lag);
        let horizon = c.actuation_delay_s + c.control_period_s * 0.5;
        let predicted = wrap(
            current_phase
                + self.omega * horizon
                + 0.5 * alpha * horizon * horizon
                + op.heading_trim_rad,
        );
        if spin_valid
            && (matches!(self.state, State::SpinUp | State::Running | State::Stopping)
                || recovery_drive)
        {
            mode = DriveMode::Drive;
            let desired = if self.state == State::Stopping {
                0.0
            } else {
                target
            };
            let error = desired - self.omega;
            let raw = c.feedforward_per_rad_s * desired + c.kp * error + self.integral;
            let limit = if self.state == State::Recovering {
                self.recovery.spin_limit
            } else {
                c.max_output
            };
            spin = raw.clamp(-limit, limit);
            // Conditional integration: spin has first priority, translation uses remaining headroom.
            if raw == spin || error.signum() != raw.signum() {
                self.integral =
                    (self.integral + c.ki * error * dt).clamp(-c.max_output, c.max_output);
            }
            saturated = raw != spin;
            let requested = c.translation_limit
                * (op.translation[0] * libm::cosf(predicted)
                    + op.translation[1] * libm::sinf(predicted));
            if self.state == State::Running && self.reference && tof_ready {
                let headroom = (c.max_output - spin.abs()).max(0.0);
                let ramp = self.recovery.released_us.map_or(1.0, |t| {
                    (now.saturating_sub(t) as f32 / RECOVERY_RAMP_US as f32).min(1.0)
                });
                translation = (requested * ramp).clamp(-headroom, headroom);
                saturated |= translation != requested;
            }
            if self.state == State::SpinUp && (target - self.omega).abs() < target * 0.1 {
                self.state = State::Running;
            }
            if self.state == State::Stopping
                && self.omega.abs()
                    < if c.gyro_enabled {
                        c.stopped_rad_s
                    } else {
                        c.high_g_crossover_rad_s
                    }
            {
                mode = if op.brake {
                    DriveMode::Brake
                } else {
                    DriveMode::Coast
                };
                spin = 0.0;
            }
        }
        if self.state == State::Fault {
            mode = DriveMode::Coast;
        }
        if self.state == State::Recovering && !recovery_drive {
            mode = DriveMode::Coast;
        }
        Output {
            timestamp_us: now,
            apply_at_us: now + (c.actuation_delay_s * 1e6) as u64,
            // Wheel 0 at +y, wheel 1 at -y, both positive commands mean body +x force.
            motors: [-spin + translation, spin + translation],
            mode,
            indicator: self.reference && tof_ready && wrap(predicted).abs() < 0.12,
            state: self.state,
            diagnostics: Diagnostics {
                spin_rad_s: self.omega,
                spin_valid,
                phase_rad: current_phase,
                predicted_phase_rad: predicted,
                phase_uncertainty_rad: self.uncertainty,
                reference_confident: self.reference && tof_ready,
                gyro_used: use_gyro,
                tof_used: c.tof.is_some() && tof_ready,
                tof_corrections: self.tof_heading.corrections,
                tof_residual_m: self.tof_heading.residual_m,
                tof_heading_age_us: now.saturating_sub(self.tof_heading.last_fix_us.unwrap_or(0)),
                tof_position_m: self.tof_heading.position_m,
                sensor_age_us: age,
                integral: self.integral,
                spin_duty: spin,
                translation_duty: translation,
                saturated,
                fault: self.fault,
            },
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn gyro_free_spinup_tof_is_required_and_outage_blocks_translation() {
        let tc = TofConfig::default();
        let mut c = Controller::new(Config {
            gyro_enabled: false,
            tof: Some(tc),
            feedforward_per_rad_s: 0.002,
            ..Config::default()
        })
        .unwrap();
        let omega = rpm_to_rad_s(3000.0);
        let phase = |t: f32| {
            let u = (t - 0.35).max(0.0);
            if u < 1.0 {
                0.5 * omega * u * u
            } else {
                0.5 * omega + omega * (u - 1.0)
            }
        };
        let mut last = Output::default();
        let mut had_heading = false;
        let mut had_translation = false;
        for n in 0..3000u64 {
            let t = n * 1000;
            let seconds = t as f32 * 1e-6;
            let w = omega * (seconds - 0.35).clamp(0.0, 1.0);
            let alpha = if seconds > 0.35 && seconds < 1.35 {
                omega
            } else {
                0.0
            };
            let m = Measurement {
                acquired_us: t,
                delivered_us: t,
                valid: true,
                ..Measurement::default()
            };
            let at = t.saturating_sub(1000);
            let range = tof::wall_range([0.0; 2], phase(at as f32 * 1e-6), tc)
                .unwrap()
                .0;
            let o = c.update(Input {
                now_us: t,
                // A deliberately contradictory legacy gyro cannot affect this hardware path.
                gyro: Measurement {
                    value: [999.0, 999.0, -999.0],
                    ..m
                },
                accel_positive: Measurement {
                    value: [-0.02 * w * w, 0.02 * alpha, G],
                    ..m
                },
                accel_negative: Measurement {
                    value: [0.02 * w * w, -0.02 * alpha, G],
                    ..m
                },
                battery_voltage: Measurement {
                    value: [15.0, 0.0, 0.0],
                    ..m
                },
                tof: Measurement {
                    acquired_us: at,
                    value: [range, 1000.0, 0.0005],
                    valid: !(2.0..2.5).contains(&seconds),
                    ..m
                },
                operator: Operator {
                    timestamp_us: t,
                    enable: n >= 200,
                    reference_id: 1,
                    target_rpm: 3000.0,
                    translation: if n >= 1800 { [1.0, 0.0] } else { [0.0; 2] },
                    ..Operator::default()
                },
                transport_ok: true,
                ..Input::default()
            });
            assert!(!o.diagnostics.gyro_used);
            assert!(o.diagnostics.spin_valid);
            last = o;
            if n == 1500 {
                assert_eq!(o.state, State::Running, "{o:?}");
            }
            if n < 1500 {
                had_heading |= o.diagnostics.tof_used;
            }
            if (1800..2000).contains(&n) {
                had_translation |= o.diagnostics.translation_duty.abs() > 0.01;
            }
            if (2200..2500).contains(&n) {
                assert_eq!(o.diagnostics.translation_duty, 0.0);
                assert!(!o.diagnostics.reference_confident);
            }
            if n >= 2700 {
                assert!(o.diagnostics.reference_confident && o.diagnostics.tof_used);
            }
        }
        assert!(
            had_heading && had_translation,
            "heading={had_heading} translation={had_translation} last={last:?}"
        );
    }
    fn input(t: u64) -> Input {
        let m = Measurement {
            acquired_us: t,
            delivered_us: t,
            valid: true,
            ..Default::default()
        };
        Input {
            now_us: t,
            gyro: m,
            accel_positive: m,
            accel_negative: m,
            battery_voltage: Measurement {
                value: [12.0, 0.0, 0.0],
                ..m
            },
            operator: Operator {
                timestamp_us: t,
                ..Default::default()
            },
            transport_ok: true,
            ..Default::default()
        }
    }
    #[test]
    fn alignment_is_operator_only_and_loss_is_not_restored_by_spin() {
        let mut c = Controller::new(Config::default()).unwrap();
        let mut i = input(0);
        assert!(!c.update(i).diagnostics.reference_confident);
        i = input(1000);
        i.operator.reference_id = 1;
        i.operator.reference_heading_rad = 0.7;
        assert!(c.update(i).diagnostics.reference_confident);
        i = input(2000);
        i.gyro.valid = false;
        i.accel_positive.valid = false;
        assert!(!c.update(i).diagnostics.reference_confident);
        assert!(!c.update(input(3000)).diagnostics.reference_confident);
    }
    #[test]
    fn observed_tipping_loses_reference_without_losing_spin_measurement() {
        let mut c = Controller::new(Config::default()).unwrap();
        let mut i = input(0);
        i.operator.reference_id = 1;
        assert!(c.update(i).diagnostics.reference_confident);
        i = input(1000);
        i.gyro.value[0] = 1.0;
        let d = c.update(i).diagnostics;
        assert!(d.spin_valid);
        assert!(!d.reference_confident);
        assert!(!c.update(input(2000)).diagnostics.reference_confident);
    }
    #[test]
    fn calibration_counts_unique_acquisitions() {
        let mut c = Controller::new(Config {
            calibration_us: 10_000,
            ..Config::default()
        })
        .unwrap();
        c.update(input(0));
        for t in (1000..=15_000).step_by(500) {
            let mut i = input(t);
            i.operator.enable = true;
            i.gyro.acquired_us = 1000;
            i.gyro.delivered_us = 1000;
            assert_eq!(c.update(i).state, State::Calibrating);
        }
        assert_eq!(c.calibration_count, 1);
    }
    #[test]
    fn finite_range_gyro_falls_back_to_high_g() {
        let mut c = Controller::new(Config {
            filter_tau_s: 0.0,
            ..Config::default()
        })
        .unwrap();
        let mut i = input(0);
        i.gyro.saturated = true;
        let w = rpm_to_rad_s(3000.0);
        let a = w * w * 0.02;
        i.accel_positive.value[0] = -a;
        i.accel_negative.value[0] = a;
        let o = c.update(i);
        assert!((o.diagnostics.spin_rad_s - w).abs() < 0.01);
        assert!(!o.diagnostics.gyro_used);
        assert!(o.diagnostics.spin_valid);
    }
    #[test]
    fn independent_high_g_acquisitions_use_midpoint_and_reject_excessive_skew() {
        let mut c = Controller::new(Config {
            filter_tau_s: 0.0,
            ..Config::default()
        })
        .unwrap();
        let mut aligned = input(0);
        aligned.operator.reference_id = 1;
        assert!(c.update(aligned).diagnostics.reference_confident);
        let mut i = input(1000);
        i.gyro.saturated = true;
        let w = rpm_to_rad_s(3000.0);
        // Quantized H3LIS331DL ±400 g samples, with common COM acceleration.
        let q = 0.195 * G;
        let a = w * w * 0.02;
        i.accel_positive.value[0] = ((12.0 - a) / q).round() * q;
        i.accel_negative.value[0] = ((12.0 + a) / q).round() * q;
        i.accel_positive.acquired_us = 500;
        let o = c.update(i);
        assert!(o.diagnostics.spin_valid);
        assert!(o.diagnostics.reference_confident);
        assert!(rad_s_to_rpm(o.diagnostics.spin_rad_s - w).abs() < 2.0);
        assert_eq!(o.diagnostics.sensor_age_us, 250);
        assert_eq!(c.last_sample_us, Some(750));
        i.now_us = 1500;
        i.operator.timestamp_us = 1500;
        i.accel_negative.acquired_us = 1001;
        i.accel_negative.delivered_us = 1500;
        let o = c.update(i);
        assert!(!o.diagnostics.spin_valid);
        assert!(!o.diagnostics.reference_confident);
    }
    #[test]
    fn receiver_failure_latches_and_requires_disarmed_clear() {
        let mut c = Controller::new(Config::default()).unwrap();
        let mut i = input(0);
        i.operator.enable = true;
        c.update(i);
        i = input(200_000);
        i.operator.enable = true;
        i.operator.timestamp_us = 0;
        assert_eq!(c.update(i).diagnostics.fault, Fault::Receiver);
        i = input(201_000);
        i.operator.enable = true;
        assert_eq!(c.update(i).state, State::Fault);
        i = input(202_000);
        i.operator.clear_fault = true;
        assert_eq!(c.update(i).state, State::Disarmed);
    }
    #[test]
    fn invalid_configuration_is_rejected() {
        assert!(
            Controller::new(Config {
                accel_radius_m: 0.0,
                ..Config::default()
            })
            .is_err()
        );
    }
}
