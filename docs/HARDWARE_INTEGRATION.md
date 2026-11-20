# Hardware integration notes

This repository contains the portable control layer. It does not contain a board startup/runtime or direct Teensy HAL implementation.

## Receiver

The planned path is RadioMaster Zorro 4-in-1 -> compatible FlySky receiver -> serial iBUS -> Teensy UART -> `IbusDecoder`.

Before wiring:
1. Confirm the exact receiver model and that its configured output is iBUS servo-channel data (not sensor telemetry iBUS, SBUS, or PWM).
2. Confirm UART baud, inversion requirements, signal voltage, connector pinout, and common ground from the receiver documentation.
3. Teensy 4.0 GPIO is 3.3 V and is not 5 V tolerant. Verify receiver output voltage and use a suitable level shifter if required.
4. Confirm the Zorro RF module protocol and receiver binding are compatible.

Feed each UART byte to `IbusDecoder::push(byte, now_us)`. On each valid frame, retain the newest `ReceiverFrame`. Call `DriverMapper::update` at the control-loop rate, passing `None` when no recent frame exists. Do not call the mapper only when a packet arrives: it must continue producing fresh stop commands after packet loss.

### RF failsafe limitation

A standard iBUS channel frame contains channel values and checksum, not a universal RF-link-status flag. A receiver may continue outputting configured failsafe channel values after transmitter loss. Configure RF-loss failsafe values so enable is OFF and throttle is minimum if supported. The timeout remains a second layer, not a replacement. Test transmitter power-off, receiver power cycling, UART unplug, MCU reset, and watchdog behavior with motors physically disconnected.

## robosim integration

The existing robosim controller accepts `control_core::Operator` inside `control_core::Input`. This crate's `RobotOperator` mirrors those fields. In the firmware adapter, copy the command fields into the real `control_core::Operator`, then pass the resulting `Input` to the existing `Controller::update`. Keep the controller's sensor acquisition, estimator, heading math, and motor calculations in robosim.

The mapper needs feedback each loop:
- `RobotState` from the controller output state;
- `spin_rpm` from `Output.diagnostics.spin_rad_s` converted with `rad_s_to_rpm`.

The mapper uses that feedback to keep `enable=true, stop=true` during controlled stopping, then disarms after the controller reports a stopped state. It deliberately does not set `enable=false` immediately when the physical toggle goes OFF, because robosim interprets that as immediate disarm.

## LEDs

Use `LedDisplay<N>::render(output.diagnostics.predicted_phase_rad, artwork)`. Calibrate `phase_offset_rad` and `reverse_direction` on the assembled robot with the wheels disabled. Send the same encoded frame to both face strips.

The SK9822 output uses clock + data and requires a suitable SPI/DMA implementation. Power the LEDs from a correctly sized external 5 V supply; do not power the strip from a Teensy GPIO pin. Join signal ground to the controller ground, verify signal-level compatibility, fuse the LED supply, and account for peak current and voltage drop.

## Still to verify before powered testing

- exact receiver model, protocol mode, UART settings, output voltage, and RF-loss failsafe configuration;
- actual Zorro channel order and switch channel;
- Teensy board, UART/SPI pin assignment, startup, watchdog, and firmware build target;
- ESC signal-loss and safe-output behavior;
- LED count, strip length, current budget, wiring, and mechanical protection;
- phase-to-LED orientation and driver translation sign convention.
