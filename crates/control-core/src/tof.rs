//! Bounded local scan matching against a known rectangular arena. No pose truth input.
use crate::{Measurement, wrap};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default, deny_unknown_fields)]
pub struct TofConfig {
    pub arena_half_m: [f32; 2],
    pub mount_m: [f32; 2],
    pub mount_angle_rad: f32,
    pub max_age_us: u64,
    pub heading_timeout_us: u64,
    pub max_window_us: u64,
    pub max_sweep_rad: f32,
    pub phase_gate_rad: f32,
    pub max_residual_m: f32,
}
impl Default for TofConfig {
    fn default() -> Self {
        Self {
            arena_half_m: [3.0; 2],
            mount_m: [0.1016, 0.0],
            mount_angle_rad: 0.0,
            max_age_us: 10_000,
            heading_timeout_us: 100_000,
            max_window_us: 50_000,
            max_sweep_rad: 0.35,
            phase_gate_rad: 0.35,
            max_residual_m: 0.08,
        }
    }
}
impl TofConfig {
    pub fn valid(&self) -> bool {
        self.arena_half_m.iter().all(|v| v.is_finite() && *v > 0.5)
            && self.mount_m.iter().all(|v| v.is_finite())
            && self.mount_angle_rad.is_finite()
            && self.max_age_us > 0
            && self.heading_timeout_us > self.max_age_us
            && self.max_window_us > 0
            && self.max_sweep_rad.is_finite()
            && self.max_sweep_rad > 0.0
            && self.phase_gate_rad.is_finite()
            && self.phase_gate_rad > 0.0
            && self.phase_gate_rad < core::f32::consts::FRAC_PI_4
            && self.max_residual_m.is_finite()
            && self.max_residual_m > 0.0
    }
}

/// Horizontal distance and wall index; the origin must be inside the known arena.
pub fn wall_range(position: [f32; 2], heading: f32, c: TofConfig) -> Option<(f32, usize)> {
    let (s, co) = (libm::sinf(heading), libm::cosf(heading));
    let p = [
        position[0] + co * c.mount_m[0] - s * c.mount_m[1],
        position[1] + s * c.mount_m[0] + co * c.mount_m[1],
    ];
    if (0..2).any(|i| p[i].abs() >= c.arena_half_m[i]) {
        return None;
    }
    let a = heading + c.mount_angle_rad;
    let d = [libm::cosf(a), libm::sinf(a)];
    let mut hit = (f32::MAX, 0);
    for i in 0..2 {
        if d[i].abs() > 1e-6 {
            let sign = if d[i] > 0.0 { 1.0 } else { -1.0 };
            let t = (sign * c.arena_half_m[i] - p[i]) / d[i];
            if t > 0.0 && t < hit.0 {
                hit = (t, i * 2 + usize::from(sign < 0.0));
            }
        }
    }
    Some(hit)
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
struct Point {
    range: f32,
    phase: f32,
    at_us: u64,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct Heading {
    points: [Point; 32],
    count: usize,
    next: usize,
    last_sample_us: Option<u64>,
    pub position_m: [f32; 2],
    pub last_fix_us: Option<u64>,
    pub corrections: u32,
    pub residual_m: f32,
}
impl Default for Heading {
    fn default() -> Self {
        Self {
            points: [Point::default(); 32],
            count: 0,
            next: 0,
            last_sample_us: None,
            position_m: [0.0; 2],
            last_fix_us: None,
            corrections: 0,
            residual_m: 0.0,
        }
    }
}
impl Heading {
    /// Discard pre-impact rays and delayed exposures, retaining the estimated
    /// position as a seed and the lifetime accepted-fix counter.
    pub fn restart_scan(&mut self, after_us: u64) {
        self.count = 0;
        self.next = 0;
        self.last_sample_us = Some(after_us);
        self.last_fix_us = None;
        self.residual_m = 0.0;
    }
    pub fn fresh(&self, now: u64, c: TofConfig) -> bool {
        self.last_fix_us
            .is_some_and(|t| now.saturating_sub(t) <= c.heading_timeout_us)
    }
    /// Range values: distance in metres, return strength, integration duration in seconds.
    /// `phase_now` is the rate-integrated estimate at NOW, corrected for IMU group delay.
    pub fn observe(
        &mut self,
        m: Measurement,
        now: u64,
        phase_now: f32,
        omega: f32,
        c: TofConfig,
    ) -> Option<f32> {
        if !m.fresh(now, c.max_age_us)
            || m.value[0] < 0.1
            || m.value[0] > 12.0
            || m.value[1] < 100.0
            || m.value[1] >= 65535.0
            || m.value[2] < 0.0
            || omega.abs() * m.value[2] > c.max_sweep_rad
            || self.last_sample_us.is_some_and(|t| m.acquired_us <= t)
        {
            return None;
        }
        self.last_sample_us = Some(m.acquired_us);
        self.points[self.next] = Point {
            range: m.value[0],
            phase: wrap(phase_now - omega * (now - m.acquired_us) as f32 * 1e-6),
            at_us: m.acquired_us,
        };
        self.next = (self.next + 1) % 32;
        self.count = (self.count + 1).min(32);
        if self.count < 16 || !self.next.is_multiple_of(4) {
            return None;
        }
        let mut span = 0.0;
        let mut previous = None;
        let mut used = 0;
        for n in 0..self.count {
            let p = self.points[(self.next + 32 - self.count + n) % 32];
            if now.saturating_sub(p.at_us) > c.max_window_us {
                continue;
            }
            if let Some(last) = previous {
                span += wrap(p.phase - last).abs();
            }
            previous = Some(p.phase);
            used += 1;
        }
        if used < 16 || span < 2.8 {
            return None;
        }
        // A center seed can stall on a rectangular wall/corner switch. The
        // wider startup/recovery search has two bounded fallback seeds, all
        // constrained to the same manually selected arena branch. Normal
        // tracking retains one six-iteration fit.
        let (pose, rms) = self.fit(now, used, c, 0.0).or_else(|| {
            if c.phase_gate_rad > 0.5 {
                let step = c.phase_gate_rad * 0.5;
                self.fit(now, used, c, -step)
                    .or_else(|| self.fit(now, used, c, step))
            } else {
                None
            }
        })?;
        self.residual_m = rms;
        self.position_m = [pose[0], pose[1]];
        self.last_fix_us = Some(now);
        self.corrections = self.corrections.saturating_add(1);
        for p in &mut self.points {
            p.phase = wrap(p.phase + pose[2]);
        }
        Some(pose[2])
    }
    fn fit(&self, now: u64, used: usize, c: TofConfig, phase_seed: f32) -> Option<([f32; 3], f32)> {
        let mut pose = [self.position_m[0], self.position_m[1], phase_seed];
        for _ in 0..if c.phase_gate_rad > 0.5 { 12 } else { 6 } {
            let mut a = [[0.0f32; 3]; 3];
            let mut b = [0.0; 3];
            // Near objects return before the walls. Reject the worst residuals
            // during fitting, rather than letting them bias the pose and only
            // rejecting them afterwards. Keep all points within the existing
            // inlier tolerance; the final 75% / three-wall gate still applies.
            let mut errors = [f32::MAX; 32];
            let mut error_count = 0;
            for p in &self.points[..self.count] {
                if now.saturating_sub(p.at_us) <= c.max_window_us {
                    let (range, _) = wall_range([pose[0], pose[1]], p.phase + pose[2], c)?;
                    errors[error_count] = (p.range - range).abs();
                    error_count += 1;
                }
            }
            errors[..error_count].sort_unstable_by(f32::total_cmp);
            let cutoff = errors[(error_count * 3).div_ceil(4) - 1].max(c.max_residual_m * 2.5);
            for p in &self.points[..self.count] {
                if now.saturating_sub(p.at_us) > c.max_window_us {
                    continue;
                }
                let (range, _) = wall_range([pose[0], pose[1]], p.phase + pose[2], c)?;
                let residual = p.range - range;
                if residual.abs() > cutoff {
                    continue;
                }
                let mut j = [0.0; 3];
                for k in 0..3 {
                    let step = if k == 2 { 0.001 } else { 0.003 };
                    let mut q = pose;
                    q[k] += step;
                    let (rr, _) = wall_range([q[0], q[1]], p.phase + q[2], c)?;
                    j[k] = (rr - range) / step;
                }
                let weight = (0.15 / residual.abs().max(0.15)).min(1.0);
                for k in 0..3 {
                    b[k] += weight * j[k] * residual;
                    for l in 0..3 {
                        a[k][l] += weight * j[k] * j[l];
                    }
                }
            }
            // Bounded three-variable Gaussian elimination with pivoting. Degenerate scans fail.
            for k in 0..3 {
                let pivot = (k..3).max_by(|i, j| a[*i][k].abs().total_cmp(&a[*j][k].abs()))?;
                a.swap(k, pivot);
                b.swap(k, pivot);
                if a[k][k].abs() < 1e-4 {
                    return None;
                }
                let scale = a[k][k];
                for v in &mut a[k][k..] {
                    *v /= scale;
                }
                b[k] /= scale;
                let pivot_row = a[k];
                for (l, row) in a.iter_mut().enumerate() {
                    if l != k {
                        let f = row[k];
                        for (cell, pivot) in row.iter_mut().zip(pivot_row).skip(k) {
                            *cell -= f * pivot;
                        }
                        b[l] -= f * b[k];
                    }
                }
            }
            pose[0] += b[0].clamp(-0.3, 0.3);
            pose[1] += b[1].clamp(-0.3, 0.3);
            pose[2] += b[2].clamp(-0.1, 0.1);
            if pose[2].abs() > c.phase_gate_rad {
                return None;
            }
        }
        let mut square = 0.0;
        let mut walls = [false; 4];
        let mut inliers = 0;
        for p in &self.points[..self.count] {
            if now.saturating_sub(p.at_us) > c.max_window_us {
                continue;
            }
            let (range, wall) = wall_range([pose[0], pose[1]], p.phase + pose[2], c)?;
            let error = p.range - range;
            if error.abs() <= c.max_residual_m * 2.5 {
                square += error * error;
                walls[wall] = true;
                inliers += 1;
            }
        }
        if inliers < 16 || inliers * 4 < used * 3 || walls.iter().filter(|v| **v).count() < 3 {
            return None;
        }
        let rms = libm::sqrtf(square / inliers as f32);
        if rms > c.max_residual_m {
            return None;
        }
        Some((pose, rms))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn a_small_obstacle_sector_does_not_bias_the_wall_heading_fit() {
        let c = TofConfig::default();
        let mut h = Heading::default();
        let omega = crate::rpm_to_rad_s(3000.0);
        let mut bias = 0.12;
        for n in 1..200u64 {
            let at = n * 1000;
            let phase = omega * at as f32 * 1e-6;
            let mut range = wall_range([0.6, -0.3], phase, c).unwrap().0;
            // A nearby solid hides the walls over three of twenty bearings.
            if n % 20 < 3 {
                range = 0.5;
            }
            let m = Measurement {
                acquired_us: at,
                delivered_us: at,
                value: [range, 1000.0, 0.0005],
                valid: true,
                saturated: false,
            };
            if let Some(delta) = h.observe(m, at, phase + bias, omega, c) {
                bias += delta;
            }
        }
        assert!(h.corrections > 20);
        assert!(bias.abs() < 0.005, "phase bias {bias}");
        assert!((h.position_m[0] - 0.6).abs() < 0.01);
        assert!((h.position_m[1] + 0.3).abs() < 0.01);
        // Losing all wall returns must still expire the heading reference.
        for n in 200..400u64 {
            let at = n * 1000;
            let m = Measurement {
                acquired_us: at,
                delivered_us: at,
                value: [0.5, 1000.0, 0.0005],
                valid: true,
                saturated: false,
            };
            h.observe(m, at, omega * at as f32 * 1e-6 + bias, omega, c);
        }
        assert!(!h.fresh(400_000, c));
    }
    #[test]
    fn delayed_ranges_correct_phase_and_unknown_position_without_heading_input() {
        let c = TofConfig::default();
        let mut h = Heading::default();
        let mut bias = 0.12;
        let omega = 314.15927;
        for n in 1..200u64 {
            let at = n * 1000;
            let theta = omega * at as f32 * 1e-6;
            // Independent four-wall geometry for a sensor displaced 101.6 mm radially.
            let (s, co) = (libm::sinf(theta), libm::cosf(theta));
            let p = [0.6 + 0.1016 * co, -0.3 + 0.1016 * s];
            let d = [co, s];
            let range = (0..2)
                .map(|i| ((if d[i] > 0.0 { 3.0 } else { -3.0 }) - p[i]) / d[i])
                .fold(f32::MAX, f32::min);
            let now = at + 1500;
            let m = Measurement {
                acquired_us: at,
                delivered_us: now,
                value: [range, 1000.0, 0.0005],
                valid: true,
                saturated: false,
            };
            if let Some(delta) = h.observe(m, now, omega * now as f32 * 1e-6 + bias, omega, c) {
                bias += delta;
            }
        }
        assert!(h.corrections > 20);
        assert!(bias.abs() < 0.005);
        assert!((h.position_m[0] - 0.6).abs() < 0.01 && (h.position_m[1] + 0.3).abs() < 0.01);
        assert!(h.fresh(201000, c));
        assert!(!h.fresh(400000, c));
        let count = h.corrections;
        let stale = Measurement {
            acquired_us: 1000,
            delivered_us: 201000,
            value: [3.0, 1000.0, 0.0005],
            valid: true,
            saturated: false,
        };
        assert!(h.observe(stale, 201000, 0.0, omega, c).is_none());
        let smeared = Measurement {
            acquired_us: 201000,
            delivered_us: 201000,
            value: [3.0, 1000.0, 0.02],
            valid: true,
            saturated: false,
        };
        assert!(h.observe(smeared, 201000, 0.0, omega, c).is_none());
        assert_eq!(h.corrections, count);
    }
    #[test]
    fn a_single_direction_and_sparse_aliased_ranges_cannot_supply_heading() {
        let c = TofConfig::default();
        let mut h = Heading::default();
        for n in 1..200u64 {
            let t = n * 10000;
            let m = Measurement {
                acquired_us: t,
                delivered_us: t,
                value: [2.8984, 1000.0, 0.0005],
                valid: true,
                saturated: false,
            };
            assert!(h.observe(m, t, 0.0, 314.15927, c).is_none());
        }
        assert_eq!(h.corrections, 0);
    }
}
