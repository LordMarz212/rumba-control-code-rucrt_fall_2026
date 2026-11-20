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

Do not assume an all-zero (`000`) channel value is a universal kill signal. This software rejects channel values outside its configured 750–2250 µs range and asks the controller to stop, but the receiver may instead stop sending frames, hold the last values, or keep sending its own configured fallback. Set and verify the exact receiver's RF-loss behavior separately; the desired fallback is enable OFF and throttle minimum.

## robosim integration

The `robosim` feature supplies `RobosimControlPort` using the vendored `control-core` source at the revision documented in that crate's README. This keeps builds reproducible without access to robosim's private repository. The board layer implements `RobosimHardware` to return timestamped sensor `Input` data and schedule ESC, indicator, and debug outputs. The bridge inserts the `RobotOperator` into `Input`, calls `Controller::update`, and maps controller state and diagnostics to `RobotFeedback`. It keeps the controller's estimation and motor calculations in the vendored robosim core. Board startup, sensor bus drivers, receiver UART configuration, output scheduling, and watchdog remain board work.

The mapper needs feedback each loop:
- `RobotState` from the controller output state;
- `spin_rpm` from `Output.diagnostics.spin_rad_s` converted with `rad_s_to_rpm`.
- `phase_rad` from the controller's current estimated body phase in the arena frame.

The mapper keeps `enable=true, stop=true` during controlled stopping, then disarms after robosim reports a stopped state. While disarmed, it captures a valid phase as the virtual front and sends the increasing `reference_id` plus heading. The controller source accepts alignment only while stopped and disabled. Spin enable remains blocked until this alignment is available. Translation is rotated from the driver's virtual-front frame into arena coordinates; the controller then projects it against predicted phase. The pinned robosim source defines this contract.

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
- receiver model and its confirmed UART output mode/baud. The transmitter context is RadioMaster Zorro 4-in-1 and the intended receiver protocol is iBUS, but the exact receiver model remains unconfirmed. The decoder expects standard iBUS frames; verify receiver and Zorro model configuration before wiring.
- exact Teensy pin assignments and any required UART/SPI voltage/interface hardware. Teensy 4.0 GPIO is 3.3 V; board wiring is not specified in the robosim source docs.
- installed LED strip count, pixel count per face, physical orientation, and power/data wiring. Previous project context identifies BTF-LIGHTING SK9822 (5 V, 144 LEDs/m) and two faces, but the installed segment geometry still needs confirmation.
