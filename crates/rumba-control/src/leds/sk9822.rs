//! SK9822 byte framing helpers.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sk9822Error {
    OutputBufferTooSmall { required: usize },
}

pub const fn required_sk9822_bytes(led_count: usize) -> usize {
    4 + led_count * 4 + (led_count + 15) / 16
}
