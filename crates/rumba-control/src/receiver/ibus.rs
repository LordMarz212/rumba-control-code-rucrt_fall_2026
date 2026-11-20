//! FlySky iBUS receiver (servo-channel) frame decoder.
//!
//! Standard 32-byte channel frame: 0x20 length, 0x40 command, fourteen
//! little-endian u16 channels, then the little-endian checksum
//! (0xFFFF - sum of the preceding 30 bytes).
//!
//! A valid iBUS channel frame does not itself prove RF link health. Fresh-frame
//! timeout and receiver-configured failsafe channel values remain important.

use super::ReceiverFrame;
use crate::driver::MAX_CHANNELS;

pub const IBUS_FRAME_LEN: usize = 32;
pub const IBUS_CHANNELS: usize = 14;
const IBUS_LENGTH: u8 = 0x20;
const IBUS_COMMAND: u8 = 0x40;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IbusError {
    BadHeader,
    BadChecksum,
    InvalidChannel,
}

#[derive(Debug, Clone)]
pub struct IbusDecoder {
    buffer: [u8; IBUS_FRAME_LEN],
    len: usize,
    rejected_frames: u32,
}

impl Default for IbusDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl IbusDecoder {
    pub const fn new() -> Self {
        Self {
            buffer: [0; IBUS_FRAME_LEN],
            len: 0,
            rejected_frames: 0,
        }
    }

    pub const fn rejected_frames(&self) -> u32 {
        self.rejected_frames
    }

    /// Feed one UART byte. Returns a frame only after its header and checksum
    /// have both been validated.
    pub fn push(&mut self, byte: u8, received_at_us: u64) -> Option<ReceiverFrame> {
        match self.len {
            0 => {
                if byte == IBUS_LENGTH {
                    self.buffer[0] = byte;
                    self.len = 1;
                }
                None
            }
            1 => {
                if byte == IBUS_COMMAND {
                    self.buffer[1] = byte;
                    self.len = 2;
                } else if byte == IBUS_LENGTH {
                    self.buffer[0] = byte;
                    self.len = 1;
                } else {
                    self.len = 0;
                }
                None
            }
            _ => {
                self.buffer[self.len] = byte;
                self.len += 1;
                if self.len < IBUS_FRAME_LEN {
                    return None;
                }

                if let Some(frame) = self.decode_complete(received_at_us) {
                    self.len = 0;
                    return Some(frame);
                }

                self.rejected_frames = self.rejected_frames.saturating_add(1);
                self.resynchronize();
                None
            }
        }
    }

    fn decode_complete(&self, received_at_us: u64) -> Option<ReceiverFrame> {
        if self.buffer[0] != IBUS_LENGTH || self.buffer[1] != IBUS_COMMAND {
            return None;
        }

        let mut sum = 0u16;
        for byte in &self.buffer[..30] {
            sum = sum.wrapping_add(*byte as u16);
        }
        let expected = 0xFFFFu16.wrapping_sub(sum);
        let actual = u16::from_le_bytes([self.buffer[30], self.buffer[31]]);
        if actual != expected {
            return None;
        }

        let mut channels = [1500u16; MAX_CHANNELS];
        for (index, channel) in channels.iter_mut().take(IBUS_CHANNELS).enumerate() {
            let offset = 2 + index * 2;
            *channel = u16::from_le_bytes([self.buffer[offset], self.buffer[offset + 1]]);
        }

        Some(ReceiverFrame {
            channels_us: channels,
            channel_count: IBUS_CHANNELS,
            received_at_us,
        })
    }

    /// Preserve a possible header inside a rejected frame so a dropped byte
    /// does not permanently desynchronize the stream.
    fn resynchronize(&mut self) {
        let mut start = None;
        for index in 1..(IBUS_FRAME_LEN - 1) {
            if self.buffer[index] == IBUS_LENGTH
                && self.buffer[index + 1] == IBUS_COMMAND
            {
                start = Some(index);
                break;
            }
        }

        if let Some(index) = start {
            let remaining = IBUS_FRAME_LEN - index;
            self.buffer.copy_within(index..IBUS_FRAME_LEN, 0);
            self.len = remaining;
        } else if self.buffer[IBUS_FRAME_LEN - 1] == IBUS_LENGTH {
            self.buffer[0] = IBUS_LENGTH;
            self.len = 1;
        } else {
            self.len = 0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_frame(channels: [u16; IBUS_CHANNELS]) -> [u8; IBUS_FRAME_LEN] {
        let mut bytes = [0u8; IBUS_FRAME_LEN];
        bytes[0] = IBUS_LENGTH;
        bytes[1] = IBUS_COMMAND;
        for (index, channel) in channels.iter().enumerate() {
            let offset = 2 + index * 2;
            bytes[offset..offset + 2].copy_from_slice(&channel.to_le_bytes());
        }
        let mut sum = 0u16;
        for byte in &bytes[..30] {
            sum = sum.wrapping_add(*byte as u16);
        }
        bytes[30..32].copy_from_slice(&0xFFFFu16.wrapping_sub(sum).to_le_bytes());
        bytes
    }

    #[test]
    fn decodes_fourteen_little_endian_channels() {
        let mut channels = [1500u16; IBUS_CHANNELS];
        channels[0] = 1000;
        channels[2] = 2000;
        channels[13] = 1777;
        let bytes = make_frame(channels);
        let mut decoder = IbusDecoder::new();
        let mut decoded = None;
        for byte in bytes {
            decoded = decoder.push(byte, 42_000).or(decoded);
        }
        let frame = decoded.unwrap();
        assert_eq!(frame.channel_count, 14);
        assert_eq!(frame.channels_us[0], 1000);
        assert_eq!(frame.channels_us[2], 2000);
        assert_eq!(frame.channels_us[13], 1777);
        assert_eq!(frame.received_at_us, 42_000);
    }

    #[test]
    fn rejects_bad_checksum_and_recovers_for_next_frame() {
        let mut bad = make_frame([1500; IBUS_CHANNELS]);
        bad[10] ^= 0x55;
        let good = make_frame([1600; IBUS_CHANNELS]);
        let mut decoder = IbusDecoder::new();
        for byte in bad {
            assert!(decoder.push(byte, 1_000).is_none());
        }
        let mut decoded = None;
        for byte in good {
            decoded = decoder.push(byte, 2_000).or(decoded);
        }
        assert_eq!(decoded.unwrap().channels_us[0], 1600);
        assert_eq!(decoder.rejected_frames(), 1);
    }
}
