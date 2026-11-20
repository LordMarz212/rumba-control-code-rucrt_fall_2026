use control_core::*;

/// A sensor-only fixture: stationary XY, independently integrated body phase,
/// two opposed H3 samples and delayed raw rectangular-arena ranges. No gyro.
pub struct Rig {
    pub controller: Controller,
    pub tick: u64,
    pub phase: f32,
    omega: f32,
    config: Config,
}

impl Rig {
    pub fn running() -> Self {
        let config = Config {
            gyro_enabled: false,
            tof: Some(tof::TofConfig {
                mount_m: [0.02, 0.0],
                mount_angle_rad: core::f32::consts::FRAC_PI_4,
                ..Default::default()
            }),
            feedforward_per_rad_s: 0.002,
            ..Default::default()
        };
        let mut rig = Self {
            controller: Controller::new(config).unwrap(),
            tick: 0,
            phase: 0.0,
            omega: 0.0,
            config,
        };
        let mut output = Output::default();
        for n in 0..1700 {
            let rpm = 3000.0 * ((n as f32 - 350.0) / 1000.0).clamp(0.0, 1.0);
            output = rig.step(rpm, |_| {});
        }
        assert_eq!(output.state, State::Running);
        assert!(output.diagnostics.reference_confident && output.diagnostics.tof_used);
        rig
    }

    pub fn step(&mut self, rpm: f32, change: impl FnOnce(&mut Input)) -> Output {
        let now = self.tick * 1000;
        let omega = rpm_to_rad_s(rpm);
        let alpha = (omega - self.omega) / 0.001;
        let range = tof::wall_range([0.0; 2], self.phase, self.config.tof.unwrap())
            .unwrap()
            .0;
        self.phase = wrap(self.phase + (omega + self.omega) * 0.0005);
        self.omega = omega;
        let m = Measurement {
            acquired_us: now,
            delivered_us: now,
            valid: true,
            ..Default::default()
        };
        let mut input = Input {
            now_us: now,
            gyro: Measurement {
                value: [999.0, 999.0, -999.0],
                ..m
            },
            accel_positive: Measurement {
                value: [
                    -self.config.accel_radius_m * omega * omega,
                    self.config.accel_radius_m * alpha,
                    G,
                ],
                ..m
            },
            accel_negative: Measurement {
                value: [
                    self.config.accel_radius_m * omega * omega,
                    -self.config.accel_radius_m * alpha,
                    G,
                ],
                ..m
            },
            battery_voltage: Measurement {
                value: [15.0, 0.0, 0.0],
                ..m
            },
            tof: Measurement {
                acquired_us: now.saturating_sub(1000),
                value: [range, 1000.0, 0.0005],
                ..m
            },
            operator: Operator {
                timestamp_us: now,
                reference_id: 1,
                enable: self.tick >= 200,
                target_rpm: 3000.0,
                translation: [1.0, 0.0],
                ..Default::default()
            },
            transport_ok: true,
            ..Default::default()
        };
        change(&mut input);
        self.tick += 1;
        self.controller.update(input)
    }
}
