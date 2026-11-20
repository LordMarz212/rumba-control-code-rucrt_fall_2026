//! Virtual-front and SK9822 rendering.

mod sk9822;
pub use sk9822::{required_sk9822_bytes, Sk9822Error};

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Rgb {
    pub red: u8,
    pub green: u8,
    pub blue: u8,
}

impl Rgb {
    pub const BLACK: Self = Self { red: 0, green: 0, blue: 0 };
    pub const VIRTUAL_FRONT: Self = Self { red: 255, green: 180, blue: 24 };

    pub const fn new(red: u8, green: u8, blue: u8) -> Self {
        Self { red, green, blue }
    }
}

/// Selectable LED content. The hardware renderer always overlays the virtual
/// front marker, regardless of the selected background mode.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum LedMode {
    #[default]
    VirtualFront,
    RutgersLogo,
    SponsorLoop,
}

#[derive(Debug, Clone, Copy)]
pub struct LedConfig {
    /// Mechanical phase-to-LED alignment, calibrated on the assembled robot.
    pub phase_offset_rad: f32,
    /// Flip this if increasing controller phase runs opposite the LED index order.
    pub reverse_direction: bool,
    pub marker_width: usize,
    pub marker_color: Rgb,
    /// SK9822 global brightness, valid range 0..=31.
    pub global_brightness: u8,
}

impl Default for LedConfig {
    fn default() -> Self {
        Self {
            phase_offset_rad: 0.0,
            reverse_direction: false,
            marker_width: 3,
            marker_color: Rgb::VIRTUAL_FRONT,
            global_brightness: 16,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LedFrame<const N: usize> {
    pub pixels: [Rgb; N],
}

/// Firmware-owned artwork renderer. Asset storage and animation timing stay
/// outside the driver and receiver modules, making images easy to replace.
pub trait LedArtworkProvider {
    fn render<const N: usize>(&mut self, mode: LedMode, now_us: u64) -> [Rgb; N];
}

impl<const N: usize> LedFrame<N> {
    pub const fn black() -> Self {
        Self { pixels: [Rgb::BLACK; N] }
    }

    /// Encode one SK9822 strip frame (start frame, BGR pixels, end frame).
    pub fn encode_sk9822(&self, brightness: u8, output: &mut [u8]) -> Result<usize, Sk9822Error> {
        let required = required_sk9822_bytes(N);
        if output.len() < required {
            return Err(Sk9822Error::OutputBufferTooSmall { required });
        }
        output[..4].fill(0);
        let brightness = 0xE0 | brightness.min(31);
        let mut offset = 4;
        for pixel in &self.pixels {
            output[offset] = brightness;
            output[offset + 1] = pixel.blue;
            output[offset + 2] = pixel.green;
            output[offset + 3] = pixel.red;
            offset += 4;
        }
        let end_bytes = (N + 15) / 16;
        output[offset..offset + end_bytes].fill(0xFF);
        Ok(required)
    }
}

pub struct LedDisplay<const N: usize> {
    config: LedConfig,
}

/// Independent renderers for the two physical faces. Each side can have a
/// different LED count, mounting offset, and index direction.
pub struct LedFaces<const FACE_A: usize, const FACE_B: usize> {
    pub face_a: LedDisplay<FACE_A>,
    pub face_b: LedDisplay<FACE_B>,
}

impl<const FACE_A: usize, const FACE_B: usize> LedFaces<FACE_A, FACE_B> {
    pub const fn new(face_a: LedConfig, face_b: LedConfig) -> Self {
        Self {
            face_a: LedDisplay::new(face_a),
            face_b: LedDisplay::new(face_b),
        }
    }

    pub fn render(
        &self,
        predicted_body_phase_rad: f32,
        virtual_front_heading_rad: f32,
        mode: LedMode,
        now_us: u64,
        artwork: &mut impl LedArtworkProvider,
    ) -> (LedFrame<FACE_A>, LedFrame<FACE_B>) {
        let (face_a_art, face_b_art) = if mode == LedMode::VirtualFront {
            ([Rgb::BLACK; FACE_A], [Rgb::BLACK; FACE_B])
        } else {
            (artwork.render::<FACE_A>(mode, now_us), artwork.render::<FACE_B>(mode, now_us))
        };
        (
            self.face_a.render_virtual_front(predicted_body_phase_rad, virtual_front_heading_rad, &face_a_art),
            self.face_b.render_virtual_front(predicted_body_phase_rad, virtual_front_heading_rad, &face_b_art),
        )
    }
}

impl<const N: usize> LedDisplay<N> {
    pub const fn new(config: LedConfig) -> Self {
        Self { config }
    }

    /// Render optional artwork first, then overlay the virtual-front marker.
    /// Send the resulting identical frame to the strip on each face.
    pub fn render(&self, predicted_phase_rad: f32, artwork: &[Rgb; N]) -> LedFrame<N> {
        let mut frame = LedFrame { pixels: *artwork };
        if N == 0 {
            return frame;
        }

        let mut phase = predicted_phase_rad + self.config.phase_offset_rad;
        if !phase.is_finite() {
            // A decorative pattern without a trustworthy directional marker
            // could mislead the driver, so fail dark instead.
            return LedFrame::black();
        }
        let tau = core::f32::consts::TAU;
        while phase < 0.0 {
            phase += tau;
        }
        while phase >= tau {
            phase -= tau;
        }
        let mut position = ((phase / tau) * N as f32) as usize;
        if self.config.reverse_direction {
            position = (N - (position % N)) % N;
        } else {
            position %= N;
        }

        let width = self.config.marker_width.max(1).min(N);
        let left = width / 2;
        for step in 0..width {
            let index = (position + N + step - left) % N;
            frame.pixels[index] = self.config.marker_color;
        }
        frame
    }

    /// Place the marker for a fixed arena heading on the rotating body. The
    /// face-specific offset and direction in `LedConfig` handle mounting.
    pub fn render_virtual_front(
        &self,
        body_phase_rad: f32,
        virtual_front_heading_rad: f32,
        artwork: &[Rgb; N],
    ) -> LedFrame<N> {
        self.render(virtual_front_heading_rad - body_phase_rad, artwork)
    }

    pub const fn brightness(&self) -> u8 {
        if self.config.global_brightness > 31 {
            31
        } else {
            self.config.global_brightness
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn marker_overwrites_artwork_and_is_centered_at_phase_zero() {
        let artwork = [Rgb::new(1, 2, 3); 8];
        let display = LedDisplay::<8>::new(LedConfig { marker_width: 1, ..LedConfig::default() });
        let frame = display.render(0.0, &artwork);
        assert_eq!(frame.pixels[0], Rgb::VIRTUAL_FRONT);
        assert_eq!(frame.pixels[1], artwork[1]);
    }

    #[test]
    fn both_faces_can_use_the_same_rendered_frame() {
        let artwork = [Rgb::BLACK; 16];
        let display = LedDisplay::<16>::new(LedConfig::default());
        let face_a = display.render(1.0, &artwork);
        let face_b = face_a.clone();
        assert_eq!(face_a, face_b);
    }

    #[test]
    fn face_calibration_can_differ_and_directional_marker_survives_artwork() {
        struct SolidArtwork;
        impl LedArtworkProvider for SolidArtwork {
            fn render<const N: usize>(&mut self, _mode: LedMode, _now_us: u64) -> [Rgb; N] {
                [Rgb::new(1, 2, 3); N]
            }
        }

        let faces = LedFaces::<8, 8>::new(
            LedConfig { marker_width: 1, ..LedConfig::default() },
            LedConfig { marker_width: 1, phase_offset_rad: core::f32::consts::PI, ..LedConfig::default() },
        );
        let (face_a, face_b) = faces.render(0.0, 0.0, LedMode::RutgersLogo, 0, &mut SolidArtwork);
        assert_eq!(face_a.pixels[0], Rgb::VIRTUAL_FRONT);
        assert_eq!(face_b.pixels[4], Rgb::VIRTUAL_FRONT);
        assert_eq!(face_a.pixels[1], Rgb::new(1, 2, 3));
    }

    #[test]
    fn invalid_phase_fails_dark_instead_of_showing_unmarked_artwork() {
        let display = LedDisplay::<4>::new(LedConfig::default());
        let artwork = [Rgb::new(1, 2, 3); 4];
        assert_eq!(display.render(f32::NAN, &artwork), LedFrame::black());
    }

    #[test]
    fn virtual_front_marker_tracks_fixed_heading_as_body_rotates() {
        let display = LedDisplay::<8>::new(LedConfig { marker_width: 1, ..LedConfig::default() });
        let artwork = [Rgb::BLACK; 8];
        let frame = display.render_virtual_front(core::f32::consts::FRAC_PI_2, 0.0, &artwork);
        assert_eq!(frame.pixels[6], Rgb::VIRTUAL_FRONT);
    }

    #[test]
    fn encodes_start_pixels_and_end_frame() {
        let frame = LedFrame { pixels: [Rgb::new(1, 2, 3); 2] };
        let mut bytes = [0u8; 13];
        assert_eq!(frame.encode_sk9822(16, &mut bytes).unwrap(), 13);
        assert_eq!(&bytes[..4], &[0, 0, 0, 0]);
        assert_eq!(&bytes[4..8], &[0xF0, 3, 2, 1]);
        assert_eq!(&bytes[12..], &[0xFF]);
    }
}
