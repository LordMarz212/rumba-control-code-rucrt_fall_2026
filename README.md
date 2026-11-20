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
firmware::RobotControlPort
          |
          v
robosim control_core::Operator / Input -> existing movement controller

robosim phase estimate + latched virtual front -> LedFaces -> SK9822 frames
```

### What lives here

- **Receiver decoding:** incremental iBUS frame parser; protocol is isolated so another serial protocol can be added without changing driver mapping.
- **Firmware loop:** `RumbaFirmware` retains the last valid frame, feeds receiver bytes, requests safe output on UART errors/timeouts, and calls a `RobotControlPort` implementation every control tick.
- **Driver mapping:** configurable channel assignments, stick inversion, throttle-to-RPM mapping, translation dead zone, and up to four optional LED-mode switch bindings.
- **Failsafe:** frame timeout, malformed/missing/out-of-range data validation, stop latch, and a required toggle-off cycle after link loss.
- **Robot interface:** `RobotControlPort` accepts `RobotOperator` and returns state, spin rate, validity, and phase. The default `robosim` feature provides `RobosimControlPort`, backed by the vendored robosim `control-core` source at a documented upstream commit. Board code implements `RobosimHardware` for sensor acquisition and ESC/indicator/debug output. No motion math is duplicated here.
- **LEDs:** phase-to-virtual-front mapping, persistent directional marker layered above optional artwork, independent calibration for both faces, and SK9822 byte framing.

## Driver mapping

Default channel indices use the common AETR order as a starting point; **verify and calibrate them against the actual Zorro model before powering motors**:

| iBUS channel | Default use | Driver control |
|---:|---|---|
| CH1 | Right X | Translation X |
| CH2 | Right Y | Translation Y |
| CH3 | Left Y | Spin throttle |
| CH5 | Switch | Spin enable |

All indices are zero-based in `ChannelMap`. They are configuration, not assumptions baked into the decoder. Throttle maps 1000..2000 us to 0..max_rpm (default 3000). Translation maps 1000..2000 us to -1..1 with a configurable dead zone.

Optional LED switches are configured separately in `DriverConfig::auxiliary`. Each binding selects a mode while that switch is ON; OFF returns to `VirtualFront` unless another mapped switch is ON. Unbound switches have no software effect. The default map binds none. If more than one mapped switch is ON, the last configured binding has priority. The mapper reports only a `LedMode` selection: the firmware's artwork provider can add or replace images without changing receiver parsing or drive mapping. Optional modes are reported only while feedback says the robot is spinning; `VirtualFront` is always the fallback.

## Stop and receiver-loss behavior

The existing robosim controller distinguishes two commands:

- `enable = false`: disarm immediately.
- `enable = true, stop = true`: enter its controlled stopping state (zero spin target); after feedback confirms the robot is stopped, this layer releases enable.

Therefore, the physical enable toggle is **not** copied directly into `enable`. Turning it off requests a controlled stop. If the receiver stream times out or the frame is invalid, the layer also requests controlled stopping and requires the driver toggle to be observed OFF before it will accept a later ON. It does not automatically re-arm just because packets return.

Standard iBUS channel frames do not contain a universal, explicit RF-link-valid bit. The parser can detect valid frames and silence; it cannot prove that a receiver is not repeatedly transmitting preset failsafe channel values. Configure the exact receiver's RF-loss failsafe to put the enable channel OFF and throttle at minimum if supported, then test transmitter power-off behavior on the bench. Do not treat this software as a substitute for receiver configuration, ESC signal-loss behavior, watchdogs, or physical safety procedures.

## LEDs

The virtual front is captured while the enable switch is OFF and robosim reports a valid, stopped phase. The mapper sends that reference to robosim while disarmed, as required by the controller's stopped-alignment contract. Enabling is refused until alignment succeeds. The reference stays fixed while Rumba spins. Driver translation is rotated from the virtual-front frame into arena coordinates before passing it to the controller. LEDs show the front's position on the rotating body from `reference_heading_rad - predicted_phase_rad`. Use `LedFaces` with a separate `LedConfig` for each side so each phase offset and index direction can be calibrated independently. `LedArtworkProvider` owns replaceable logo/animation assets. The marker is drawn **after** optional artwork, so graphics cannot cover it; invalid phase fails dark rather than showing unmarked artwork. Physical phase mapping must be calibrated on the assembled robot.

The currently described strip is a single row of pixels. It supports directional markers and one-dimensional color/pattern effects. A detailed 2D Rutgers logo or sponsor video needs additional radial rows or a matrix display; the artwork interface does not create pixels the hardware does not have.

SK9822 uses clock + data and BGR pixel ordering. The renderer produces bytes; the board-specific SPI/DMA transport and power system are not included yet.

## Build and test

Requires Rust (edition 2021):

```sh
cargo test --workspace
cargo check --workspace --no-default-features
```

The control and Rumba libraries are `no_std` and allocation-free. Tests use the host test harness. `crates/control-core/README.md` records the upstream source commit and sync policy.

## Functionality status

This sheet describes the code on `main`. “Implemented” means the portable host-side control code is present; it does not mean the feature has been validated on the assembled robot.

| Status | Functionality |
|---|---|
| Implemented | iBUS frame decoding and validation |
| Implemented | Remappable throttle and translation channels |
| Implemented | Spin control, controlled stop, receiver timeout, stop latch, and re-arm cycle |
| Implemented | SK9822 frame encoding and configurable LED-mode switch mapping |
| Implemented | Virtual-front capture while stopped, fixed reference while spinning, and translation in arena coordinates |
| Implemented | Phase-based virtual-front rendering with independent calibration for each face |
| Implemented | Host build/test commands and portable `no_std` control libraries (hardware verification remains pending) |
| Not implemented | Board-specific Teensy startup, UART/SPI transport, ESC outputs, and watchdog integration |
| Not implemented | Physical robot calibration and receiver/ESC failsafe bench testing |
| Not implemented | Rutgers/sponsor artwork and animation assets |

## Current integration boundary

This repository includes a portable control-loop library, not a flashable Teensy firmware image. The exact FlySky receiver variant, UART pins/baud, electrical signal level, Teensy board startup/HAL, LED SPI/DMA transport, ESC output implementation, and robosim firmware packaging still need hardware-specific integration and bench verification.

See [hardware integration](docs/HARDWARE_INTEGRATION.md) and [driver controls](docs/DRIVER_CONTROLS.md).
