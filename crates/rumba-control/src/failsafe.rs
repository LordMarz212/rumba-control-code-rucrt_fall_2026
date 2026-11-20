//! Validation and receiver-loss state.

use crate::receiver::ReceiverFrame;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FailsafeReason {
    NoFrame,
    FrameTimedOut,
    FrameFromFuture,
    MissingRequiredChannel,
    InvalidChannelValue,
    InvalidConfiguration,
    InvalidControllerPhase,
    VirtualFrontNotSet,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FailsafeState {
    Healthy,
    Active(FailsafeReason),
}

#[derive(Debug, Clone, Copy)]
pub struct FailsafeConfig {
    pub frame_timeout_us: u64,
    pub min_channel_us: u16,
    pub max_channel_us: u16,
}

impl Default for FailsafeConfig {
    fn default() -> Self {
        Self {
            frame_timeout_us: 100_000,
            min_channel_us: 750,
            max_channel_us: 2250,
        }
    }
}

pub fn validate_frame(
    frame: Option<&ReceiverFrame>,
    now_us: u64,
    required_channels: usize,
    config: FailsafeConfig,
) -> Result<&ReceiverFrame, FailsafeReason> {
    let frame = frame.ok_or(FailsafeReason::NoFrame)?;
    if frame.received_at_us > now_us {
        return Err(FailsafeReason::FrameFromFuture);
    }
    if now_us.saturating_sub(frame.received_at_us) > config.frame_timeout_us {
        return Err(FailsafeReason::FrameTimedOut);
    }
    if frame.channel_count < required_channels || frame.channel_count > frame.channels_us.len() {
        return Err(FailsafeReason::MissingRequiredChannel);
    }
    if frame.channels_us[..frame.channel_count]
        .iter()
        .any(|value| *value < config.min_channel_us || *value > config.max_channel_us)
    {
        return Err(FailsafeReason::InvalidChannelValue);
    }
    Ok(frame)
}
