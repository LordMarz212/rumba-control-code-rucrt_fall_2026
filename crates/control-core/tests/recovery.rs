mod support;
use control_core::*;
use support::Rig;

#[test]
fn an_unclipped_sudden_spin_drop_also_triggers_recovery() {
    let mut rig = Rig::running();
    let hit = rig.step(2400.0, |_| {});
    assert_eq!(hit.state, State::Recovering);
    assert_eq!(hit.mode, DriveMode::Coast);
    assert!(hit.diagnostics.spin_valid);
    assert_eq!(hit.diagnostics.translation_duty, 0.0);
    let mut last = hit;
    for n in 0..200 {
        last = rig.step((2400.0 + n as f32 * 6.0).min(3000.0), |_| {});
        assert_eq!(last.diagnostics.fault, Fault::None);
        if last.state == State::Recovering {
            assert_eq!(last.diagnostics.translation_duty, 0.0);
        }
    }
    assert_eq!(last.state, State::Running);
    assert!(last.diagnostics.reference_confident && last.diagnostics.tof_used);
    assert!(wrap(last.diagnostics.phase_rad - rig.phase).abs() < 0.08);
}

#[test]
fn clipped_impacts_coast_then_restore_spin_and_fresh_tof_movement() {
    let mut rig = Rig::running();
    for clipped in [1, 2, 3, 1] {
        let before = rig.step(3000.0, |_| {});
        for _ in 0..clipped {
            let hit = rig.step(2700.0, |i| i.accel_positive.saturated = true);
            assert_eq!(hit.state, State::Recovering);
            assert_eq!(hit.mode, DriveMode::Coast);
            assert_eq!(hit.motors, [0.0; 2]);
            assert!(!hit.diagnostics.reference_confident);
            assert!(!hit.diagnostics.tof_used);
            assert_eq!(hit.diagnostics.integral, 0.0);
        }
        let mut restored = false;
        let mut moved = false;
        let mut driven = false;
        for n in 0..200 {
            let rpm = (2700.0 + n as f32 * 6.0).min(3000.0);
            let o = rig.step(rpm, |_| {});
            assert_eq!(o.diagnostics.fault, Fault::None);
            assert!(!o.diagnostics.gyro_used);
            if n < 2 {
                assert_eq!(
                    o.mode,
                    DriveMode::Coast,
                    "distinct healthy samples must settle"
                );
                assert_eq!(o.motors, [0.0; 2]);
            }
            if o.state == State::Recovering {
                assert_eq!(o.diagnostics.translation_duty, 0.0);
                assert!(!o.diagnostics.reference_confident);
                if o.mode == DriveMode::Drive {
                    driven = true;
                    assert!(o.diagnostics.spin_duty.abs() <= (n as f32 - 1.0) * 0.019 + 0.0001);
                }
            } else {
                assert_eq!(o.state, State::Running);
                assert!(o.diagnostics.reference_confident && o.diagnostics.tof_used);
                assert!(o.diagnostics.tof_corrections >= before.diagnostics.tof_corrections + 3);
                assert!(wrap(o.diagnostics.phase_rad - rig.phase).abs() < 0.08);
                if !restored {
                    assert_eq!(
                        o.diagnostics.translation_duty, 0.0,
                        "movement starts at zero duty"
                    );
                    restored = true;
                }
                moved |= o.diagnostics.translation_duty.abs() > 0.01;
            }
        }
        assert!(restored && moved && driven);
    }
}

#[test]
fn a_second_hit_during_recovery_discards_the_partial_heading_scan() {
    let mut rig = Rig::running();
    rig.step(2900.0, |i| i.accel_positive.saturated = true);
    for _ in 0..30 {
        rig.step(2900.0, |_| {});
    }
    let hit = rig.step(2800.0, |i| i.accel_negative.saturated = true);
    assert_eq!(hit.state, State::Recovering);
    let fixes = hit.diagnostics.tof_corrections;
    for _ in 0..55 {
        let o = rig.step(2900.0, |_| {});
        assert_eq!(o.diagnostics.fault, Fault::None);
        if o.diagnostics.reference_confident {
            assert!(o.diagnostics.tof_corrections >= fixes + 3);
        } else {
            assert_eq!(o.diagnostics.translation_duty, 0.0);
        }
    }
}

#[test]
fn persistent_clipping_remains_a_latched_sensor_fault() {
    let mut rig = Rig::running();
    let mut o = Output::default();
    for _ in 0..25 {
        o = rig.step(2800.0, |i| i.accel_positive.saturated = true);
        assert_eq!(o.motors, [0.0; 2]);
    }
    assert_eq!(o.state, State::Fault);
    assert_eq!(o.diagnostics.fault, Fault::Sensor);
    for _ in 0..100 {
        o = rig.step(3000.0, |_| {});
        assert_eq!(o.state, State::Fault);
        assert_eq!(o.mode, DriveMode::Coast);
        assert_eq!(o.motors, [0.0; 2]);
    }
}

#[test]
fn intermittent_bad_samples_cannot_restart_the_sensor_deadline_forever() {
    let mut rig = Rig::running();
    let mut o = Output::default();
    for n in 0..30 {
        o = rig.step(2800.0, |i| i.accel_positive.saturated = n % 2 == 0);
        assert_eq!(o.motors, [0.0; 2]);
    }
    assert_eq!(o.state, State::Fault);
    assert_eq!(o.diagnostics.fault, Fault::Sensor);
}

#[test]
fn repeated_or_out_of_order_acquisitions_cannot_satisfy_recovery() {
    for backwards in [false, true] {
        let mut rig = Rig::running();
        let stamp = rig.tick * 1000;
        rig.step(2800.0, |i| i.accel_positive.saturated = true);
        for n in 0..25 {
            let o = rig.step(2800.0, |i| {
                let acquired = if backwards { stamp - 2000 } else { stamp };
                i.accel_positive.acquired_us = acquired;
                i.accel_negative.acquired_us = acquired;
            });
            assert_eq!(o.motors, [0.0; 2]);
            assert!(!o.diagnostics.reference_confident);
            if n >= 21 {
                assert_eq!(o.state, State::Fault);
                assert_eq!(o.diagnostics.fault, Fault::Sensor);
            }
        }
    }
}

#[test]
fn impact_with_temporary_tof_obstruction_reacquires_before_moving() {
    for obstruction in [false, true] {
        let mut rig = Rig::running();
        rig.step(2800.0, |i| i.accel_positive.saturated = true);
        for n in 0..500 {
            let o = rig.step(3000.0, |i| {
                // A second impact while the wall view is still blocked must
                // discard its partial scan without extending the seed deadline.
                i.accel_positive.saturated = n == 250;
                if obstruction {
                    i.tof.value[0] = 0.4;
                } else {
                    i.tof.valid = false;
                }
            });
            assert_eq!(o.diagnostics.fault, Fault::None);
            assert_eq!(o.diagnostics.translation_duty, 0.0);
            assert!(!o.diagnostics.reference_confident);
        }
        let mut restored = false;
        for _ in 0..200 {
            let o = rig.step(3000.0, |_| {});
            assert_eq!(o.mode, DriveMode::Drive);
            if o.diagnostics.reference_confident {
                assert_eq!(o.state, State::Running);
                assert!(o.diagnostics.tof_used);
                assert!(wrap(o.diagnostics.phase_rad - rig.phase).abs() < 0.08);
                restored = true;
            } else {
                assert_eq!(o.diagnostics.translation_duty, 0.0);
            }
        }
        assert!(restored);
    }
}

#[test]
fn wall_scan_outages_recover_after_three_new_fits_and_ramp_movement() {
    for kind in 0..3 {
        let mut rig = Rig::running();
        let before = rig.step(3000.0, |_| {});
        let frozen = rig.tick * 1000 - 1000;
        for n in 0..500 {
            let o = rig.step(3000.0, |i| match kind {
                0 => i.tof.valid = false,
                1 => i.tof.value[0] = 0.4,
                _ => i.tof.acquired_us = frozen,
            });
            assert_eq!(o.diagnostics.fault, Fault::None);
            assert_eq!(o.mode, DriveMode::Drive);
            if n > 150 {
                assert_eq!(o.state, State::Recovering);
                assert_eq!(o.diagnostics.translation_duty, 0.0);
                assert!(!o.diagnostics.reference_confident);
                assert!(!o.diagnostics.tof_used);
            }
        }
        let mut restored = false;
        let mut moved = false;
        for _ in 0..200 {
            let o = rig.step(3000.0, |_| {});
            if o.diagnostics.reference_confident {
                assert!(o.diagnostics.tof_corrections >= before.diagnostics.tof_corrections + 3);
                assert!(o.diagnostics.tof_used);
                assert!(wrap(o.diagnostics.phase_rad - rig.phase).abs() < 0.08);
                if !restored {
                    assert_eq!(o.diagnostics.translation_duty, 0.0);
                }
                restored = true;
                moved |= o.diagnostics.translation_duty.abs() > 0.01;
            } else {
                assert_eq!(o.diagnostics.translation_duty, 0.0);
            }
        }
        assert!(restored && moved);
    }
}

#[test]
fn an_expired_wall_search_seed_cannot_restore_an_arena_branch() {
    let mut rig = Rig::running();
    for _ in 0..2500 {
        let o = rig.step(3000.0, |i| i.tof.valid = false);
        assert_eq!(o.diagnostics.fault, Fault::None);
    }
    for _ in 0..200 {
        let o = rig.step(3000.0, |_| {});
        assert_eq!(o.state, State::Running);
        assert_eq!(o.mode, DriveMode::Drive);
        assert_eq!(o.diagnostics.translation_duty, 0.0);
        assert!(!o.diagnostics.reference_confident);
        assert!(!o.diagnostics.tof_used);
    }
}

#[test]
fn an_ambiguous_blind_interval_cannot_pick_an_arena_quarter_turn() {
    let mut rig = Rig::running();
    for _ in 0..5 {
        rig.step(2800.0, |i| i.accel_positive.saturated = true);
    }
    let mut last = Output::default();
    for _ in 0..150 {
        last = rig.step(3000.0, |_| {});
        assert_eq!(last.diagnostics.fault, Fault::None);
        assert_eq!(last.diagnostics.translation_duty, 0.0);
        assert!(!last.diagnostics.reference_confident);
    }
    assert_eq!(last.state, State::Running);
    assert_eq!(last.mode, DriveMode::Drive);
}

#[test]
fn hard_faults_and_stop_override_recovery() {
    for fault in [
        Fault::Time,
        Fault::Transport,
        Fault::Receiver,
        Fault::Undervoltage,
        Fault::Overspeed,
    ] {
        let mut rig = Rig::running();
        rig.step(2800.0, |i| i.accel_positive.saturated = true);
        let last = rig.step(
            if fault == Fault::Overspeed {
                3700.0
            } else {
                2800.0
            },
            |i| match fault {
                Fault::Time => i.now_us -= 1000,
                Fault::Transport => i.transport_ok = false,
                Fault::Receiver => i.operator.timestamp_us -= 101_000,
                Fault::Undervoltage => i.battery_voltage.value[0] = 3.0,
                _ => {}
            },
        );
        assert_eq!(last.state, State::Fault);
        assert_eq!(last.mode, DriveMode::Coast);
        assert_eq!(last.motors, [0.0; 2]);
        assert_eq!(last.diagnostics.fault, fault);
        for _ in 0..5 {
            assert_eq!(rig.step(3000.0, |_| {}).state, State::Fault);
        }
    }
    let mut rig = Rig::running();
    rig.step(2800.0, |i| i.accel_positive.saturated = true);
    let stop = rig.step(2800.0, |i| i.operator.stop = true);
    assert_eq!(stop.state, State::Stopping);
    assert_eq!(stop.diagnostics.translation_duty, 0.0);
    assert!(stop.diagnostics.spin_duty <= 0.0);
    let disabled = rig.step(2800.0, |i| i.operator.enable = false);
    assert_eq!(disabled.state, State::Disarmed);
    assert_eq!(disabled.mode, DriveMode::Disarm);
    assert_eq!(disabled.motors, [0.0; 2]);
}
