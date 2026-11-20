//! Radio receiver protocol adapters.

pub mod ibus;

use crate::driver::MAX_CHANNELS;

/// A successfully decoded receiver snapshot. Channel values are pulse-width
/// equivalents in microseconds, regardless of the serial wire protocol.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReceiverFrame {
    pub channels_us: [u16; MAX_CHANNELS],
    pub channel_count: usize,
    pub received_at_us: u64,
}

impl ReceiverFrame {
    pub const EMPTY: Self = Self {
        channels_us: [1500; MAX_CHANNELS],
        channel_count: 0,
        received_at_us: 0,
    };
}
