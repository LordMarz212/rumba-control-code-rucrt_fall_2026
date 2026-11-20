use std::{
    alloc::{GlobalAlloc, Layout, System},
    cell::Cell,
};
mod support;
thread_local! {static TRACK:Cell<bool>=const{Cell::new(false)};static ALLOCATIONS:Cell<usize>=const{Cell::new(0)};}
struct Counting;
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if TRACK.try_with(Cell::get).unwrap_or(false) {
            let _ = ALLOCATIONS.try_with(|n| n.set(n.get() + 1));
        }
        // SAFETY: delegate the allocator contract to the system allocator.
        unsafe { System.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        // SAFETY: delegate the original allocation and matching layout.
        unsafe { System.dealloc(ptr, layout) }
    }
}
#[global_allocator]
static ALLOCATOR: Counting = Counting;
#[test]
fn updates_allocate_nothing() {
    use control_core::*;
    let mut c = Controller::new(Config::default()).unwrap();
    ALLOCATIONS.set(0);
    TRACK.set(true);
    for tick in 0..100_000u64 {
        let t = tick * 1000;
        let w = if tick < 200 {
            0.0
        } else {
            rpm_to_rad_s(1800.0)
        };
        let m = Measurement {
            acquired_us: t,
            delivered_us: t,
            valid: true,
            ..Default::default()
        };
        let i = Input {
            now_us: t,
            gyro: Measurement {
                value: [0.0, 0.0, w],
                saturated: w > degrees_to_rad(2000.0),
                ..m
            },
            accel_positive: Measurement {
                value: [-w * w * 0.02, 0.0, G],
                ..m
            },
            accel_negative: Measurement {
                value: [w * w * 0.02, 0.0, G],
                ..m
            },
            battery_voltage: Measurement {
                value: [14.8, 0.0, 0.0],
                ..m
            },
            operator: Operator {
                timestamp_us: t,
                reference_id: 1,
                enable: tick > 200,
                target_rpm: 1800.0,
                translation: [1.0, 0.0],
                ..Default::default()
            },
            transport_ok: true,
            ..Default::default()
        };
        std::hint::black_box(c.update(i));
    }
    TRACK.set(false);
    assert_eq!(ALLOCATIONS.get(), 0);
}

#[test]
fn tof_scan_matching_allocates_nothing() {
    use control_core::*;
    let tc = tof::TofConfig::default();
    let mut heading = tof::Heading::default();
    ALLOCATIONS.set(0);
    TRACK.set(true);
    for n in 1..1000u64 {
        let at = n * 1000;
        let phase = rpm_to_rad_s(3000.0) * at as f32 * 1e-6;
        let range = tof::wall_range([0.6, -0.3], phase, tc).unwrap().0;
        let m = Measurement {
            acquired_us: at,
            delivered_us: at,
            value: [range, 1000.0, 0.0005],
            valid: true,
            saturated: false,
        };
        std::hint::black_box(heading.observe(m, at, phase, rpm_to_rad_s(3000.0), tc));
    }
    TRACK.set(false);
    assert_eq!(ALLOCATIONS.get(), 0);
    assert!(heading.corrections > 100);
}

#[test]
fn impact_recovery_and_tof_reverification_allocate_nothing() {
    use control_core::*;
    let mut rig = support::Rig::running();
    let mut restored = 0;
    ALLOCATIONS.set(0);
    TRACK.set(true);
    for _ in 0..10 {
        rig.step(2800.0, |i| i.accel_positive.saturated = true);
        let mut last = Output::default();
        for n in 0..200 {
            last = std::hint::black_box(rig.step((2800.0 + n as f32 * 4.0).min(3000.0), |_| {}));
        }
        restored +=
            usize::from(last.state == State::Running && last.diagnostics.reference_confident);
    }
    TRACK.set(false);
    assert_eq!(ALLOCATIONS.get(), 0);
    assert_eq!(restored, 10);
}
