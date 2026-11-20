# Rumba control code

Rust control and LED layer for Rutgers Combat Robotics' Rumba meltybrain.

This repository is deliberately separate from the existing **robosim** project. It does not reimplement the robot's motion controller. It turns receiver packets into driver intent, applies receiver-loss safety behavior, exposes a narrow command interface for robosim, and renders the SK9822 virtual-front display.

## Architecture

```text
RadioMaster Zorro 4-in-1
          |
          v
FlySky receiver -- iBUS serial (provisional; verify exact receiver)
          |
          v
receiver::ibus -> ReceiverFrame
          |
          v
DriverMapper -> RobotOperator + failsafe state
          |
          v
robot_interface::RobotCommandSink
          |
          v
robosim control_core::Operator / Input -> existing movement controller

robosim phase estimate -> leds::LedDisplay -> SK9822 frames -> both LED strips
```

### What lives here

- **Receiver decoding:** incremental iBUS frame parser; protocol is isolated so another serial protocol can be added without changing driver mapping.
- **Driver mapping:** configurable channel assignments, stick inversion, throttle-to-RPM mapping, and translation dead zone.
- **Failsafe:** frame timeout, malformed/missing/out-of-range data validation, stop latch, and a required toggle-off cycle after link loss.
- **Robot interface:** a small `RobotOperator` contract mirroring the fields robosim already accepts. A firmware integration adapter copies it into `control_core::Operator`; no motion math is duplicated here.
- **LEDs:** phase-to-virtual-front mapping, persistent directional marker layered above optional artwork, two-face mirroring, and SK9822 byte framing.

## Driver mapping

Default channel indices use the common AETR order as a starting point; **verify and calibrate them against the actual Zorro model before powering motors**:

| iBUS channel | Default use | Driver control |
|---:|---|---|
| CH1 | Right X | Translation X |
| CH2 | Right Y | Translation Y |
| CH3 | Left Y | Spin throttle |
| CH5 | Switch | Spin enable |

All indices are zero-based in `ChannelMap`. They are configuration, not assumptions baked into the decoder. Throttle maps 1000..2000 us to 0..max_rpm (default 3000). Translation maps 1000..2000 us to -1..1 with a configurable dead zone.

## Stop and receiver-loss behavior

The existing robosim controller distinguishes two commands:

- `enable = false`: disarm immediately.
- `enable = true, stop = true`: enter its controlled stopping state (zero spin target); after feedback confirms the robot is stopped, this layer releases enable.

Therefore, the physical enable toggle is **not** copied directly into `enable`. Turning it off requests a controlled stop. If the receiver stream times out or the frame is invalid, the layer also requests controlled stopping and requires the driver toggle to be observed OFF before it will accept a later ON. It does not automatically re-arm just because packets return.

Standard iBUS channel frames do not contain a universal, explicit RF-link-valid bit. The parser can detect valid frames and silence; it cannot prove that a receiver is not repeatedly transmitting preset failsafe channel values. Configure the exact receiver's RF-loss failsafe to put the enable channel OFF and throttle at minimum if supported, then test transmitter power-off behavior on the bench. Do not treat this software as a substitute for receiver configuration, ESC signal-loss behavior, watchdogs, or physical safety procedures.

## LEDs

The LED renderer takes the controller's predicted phase and maps it to a strip index using a configurable phase offset and direction. The directional marker is drawn **after** the optional background/animation, so graphics cannot cover it. Send the same rendered frame to the strip on each face. The phase-to-physical-index offset and direction must be calibrated on the assembled robot; this repo intentionally does not guess the mechanical orientation.

SK9822 uses clock + data and BGR pixel ordering. The renderer produces bytes; the board-specific SPI/DMA transport and power system are not included yet.

## Build and test

Requires Rust (edition 2021):

```sh
cargo test --workspace
cargo check --workspace --no-default-features
```

The core library is `no_std` and allocation-free. Tests use the host test harness.

## Current integration boundary

This repository is a **control-layer library**, not a flashable Teensy firmware image. The exact FlySky receiver variant, UART pins/baud, electrical signal level, Teensy board startup/HAL, LED SPI/DMA transport, ESC output implementation, and robosim firmware packaging still need hardware-specific integration and bench verification.

See [hardware integration](docs/HARDWARE_INTEGRATION.md) and [driver controls](docs/DRIVER_CONTROLS.md).
