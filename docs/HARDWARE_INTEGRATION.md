# Hardware integration notes

This repository contains the portable control loop, but not a board startup or direct Teensy HAL implementation.

## Receiver

The planned path is RadioMaster Zorro 4-in-1 -> compatible FlySky receiver -> serial iBUS -> Teensy UART -> `IbusDecoder`.

Before wiring:
1. Confirm the exact receiver model and that its configured output is iBUS servo-channel data (not sensor telemetry iBUS, SBUS, or PWM).
2. Confirm UART baud, inversion requirements, signal voltage, connector pinout, and common ground from the receiver documentation.
3. Teensy 4.0 GPIO is 3.3 V and is not 5 V tolerant. Verify receiver output voltage and use a suitable level shifter if required.
4. Confirm the Zorro RF module protocol and receiver binding are compatible.

Implement the UART task so each received byte is passed to `RumbaFirmware::push_receiver_byte(byte, now_us)`. Call `receiver_io_error()` after a UART overrun/framing error. Call `RumbaFirmware::tick(now_us, &mut robot_port)` continuously at the controller loop rate, including when no bytes arrive; the retained frame is checked for staleness on every tick.

### RF failsafe limitation

A standard iBUS channel frame contains channel values and checksum, not a universal RF-link-status flag. A receiver may continue outputting configured failsafe channel values after transmitter loss. Configure RF-loss failsafe values so enable is OFF and throttle is minimum if supported. The timeout remains a second layer, not a replacement. Test transmitter power-off, receiver power cycling, UART unplug, MCU reset, and watchdog behavior with motors physically disconnected.

## robosim integration

The existing robosim controller accepts `control_core::Operator` inside `control_core::Input`. This crate's `RobotOperator` mirrors those fields. Implement `RobotControlPort` in the firmware adapter: copy the command into the real `control_core::Operator`, build the required `Input` with current sensor measurements, call the existing `Controller::update`, then translate its state and diagnostics into `RobotFeedback` and `predicted_phase_rad()`. Keep the controller's sensor acquisition, estimator, heading math, and motor calculations in robosim. The exact source checkout is needed to compile and validate this adapter; it is not included in this repository.

The mapper needs feedback each loop:
- `RobotState` from the controller output state;
- `spin_rpm` from `Output.diagnostics.spin_rad_s` converted with `rad_s_to_rpm`.
- `phase_rad` from the controller's current estimated body phase in the arena frame.

The mapper uses that feedback to keep `enable=true, stop=true` during controlled stopping, then disarms after the controller reports a stopped state. It deliberately does not set `enable=false` immediately when the physical toggle goes OFF, because robosim interprets that as immediate disarm. On a fresh enable transition it latches the current `phase_rad` as the virtual-front heading, increments `reference_id`, and continues sending that stable heading while driving. This assumes the robosim `Operator` contract interprets `reference_heading_rad` as the translation reference; the source checkout must be restored to verify this against `control-core`.

## LEDs

Use `LedFaces<FACE_A, FACE_B>` with a separate `LedConfig` for each side. Feed it `FirmwareTick.predicted_phase_rad`, `controls.operator.reference_heading_rad`, and `controls.led_mode`. Implement `LedArtworkProvider` for the patterns supported by the installed strip arrangement. Calibrate each `phase_offset_rad` and `reverse_direction` on the assembled robot with the wheels disabled. Encode and send each face's frame separately; the two faces may require different pixel orderings.

The current single-row strip supports directional and one-dimensional effects. Detailed 2D logos or sponsor video require radial rows or a matrix display. No Rutgers or sponsor assets are included yet, so the firmware's artwork provider must supply them before those modes can display actual branding.

The SK9822 output uses clock + data and requires a suitable SPI/DMA implementation. Power the LEDs from a correctly sized external 5 V supply; do not power the strip from a Teensy GPIO pin. Join signal ground to the controller ground, verify signal-level compatibility, fuse the LED supply, and account for peak current and voltage drop.

## Still to verify before powered testing

- exact receiver model, protocol mode, UART settings, output voltage, and RF-loss failsafe configuration;
- actual Zorro channel order and switch channel;
- Teensy board, UART/SPI pin assignment, startup, watchdog, and firmware build target;
- robosim source checkout and a compile-verified `RobotControlPort`/sensor adapter;
- sponsor/Rutgers artwork assets and confirmation that the installed LED geometry supports the requested visuals;
- ESC signal-loss and safe-output behavior;
- LED count, strip length, current budget, wiring, and mechanical protection;
- phase-to-LED orientation and driver translation sign convention.
