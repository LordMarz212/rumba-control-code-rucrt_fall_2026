# Rumba hardware details needed for Teensy firmware

Please answer what you can and mark unknown items **unknown**. Photos of labels, both sides of the Teensy, receiver, sensor boards, ESCs, and a wiring sketch are useful. Please include links or exact part numbers rather than approximate names.

## 1. Board and wiring

- Confirm the board is Teensy 4.0 (not 4.1 or another controller). Provide a clear photo of the board and any carrier PCB.
- Send the current wiring diagram or pin table. For each connection, give the Teensy pin number, device pin name, signal direction, and voltage level.
- Identify which Teensy pins are already occupied or reserved.
- Confirm whether any signal needs a level shifter, inverter, pull-up, or shared ground. Teensy 4.0 signal pins accept 0–3.3 V and are not 5 V tolerant; do not connect a 5 V signal directly. [PJRC Teensy 4.0 pin reference](https://www.pjrc.com/store/teensy40.html).
- Give the regulated voltages available to the Teensy, receiver, sensors, ESC signal side, and LEDs, including ground connections.

## 2. Receiver and radio failsafe

- Exact receiver manufacturer/model and hardware revision; exact RadioMaster Zorro 4-in-1 model/module and firmware/model configuration.
- Receiver output mode selected for the wire to the Teensy: iBUS channel data, SBUS, PWM, or something else. Confirm whether iBUS is inverted and its baud rate, frame format, signal voltage, and receiver connector pinout from the manual.
- State the actual receiver-to-Teensy TX/RX wiring and which Teensy UART pins are used.
- Confirm the channel table observed by the Teensy: channel number, stick/switch, and low/center/high values in microseconds. In particular identify spin throttle, translation X, translation Y, and the spin-enable switch. Current software defaults are CH3 throttle, CH1/CH2 translation, CH5 enable; these are provisional until measured.
- On transmitter power-off / RF loss, what does the receiver actually do to iBUS? Choose the manual's setting and report it: stop output, send configured per-channel failsafe values, hold last values, or another behavior.
- If it sends configured values, give the measured/configured microsecond value for every channel, especially enable and throttle. We expect enable OFF (normally near 1000 µs) and throttle minimum (normally near 1000 µs), subject to the real channel mapping.
- Send a short description/video of a bench test with transmitter powered on, then off: does iBUS stop, continue with fallback values, or hold its last frame? Include the values if available.

**Do not assume `000` is a generic kill signal.** Our software treats missing/stale frames and invalid channel values as a failsafe and asks robosim to stop. The receiver's RF-loss behavior is configured separately. A receiver that continues sending valid held values can prevent the software timeout from detecting RF loss, so its exact fallback behavior must be known and configured.

## 3. Spin sensors and ToF

- Confirm the exact two accelerometer boards/part numbers. Current robosim source assumes H3LIS331DL sensors; provide breakout-board vendor/link and schematic if available.
- Confirm sensor interface (SPI or I²C), bus voltage, clock rate, each sensor's chip-select pin, each data-ready/interrupt pin, and which Teensy pins carry SCK/MOSI/MISO or SDA/SCL.
- Provide the physical orientation of each accelerometer on the robot, distance from the spin axis, sign/orientation of its X/Y/Z axes, and whether both chips are mounted opposite each other.
- Confirm configured range and output data rate. The robosim hardware notes assume ±400 g and 1000 samples/s; say if the real boards/configuration differ.
- Confirm the exact ToF unit: current robosim documents conflict between TFmini-S and TFmini Plus. Give exact product label/revision and breakout board.
- Give ToF UART pin mapping, voltage level, baud, frame format, configured sample rate, supply, mounting location, and beam direction. The robosim model assumes 1000 Hz and 921600 baud, which must be checked against the actual sensor.
- Confirm whether there are wheel encoders or battery-voltage measurements wired to the controller; give part numbers and pins, or say none.

## 4. ESCs, motors, and safe stop

- Exact ESC and motor models/revisions and ESC firmware/configuration.
- For each ESC, identify signal wire to Teensy pin, signal protocol (PWM/OneShot/DShot/other), update rate, pulse/duty range, neutral/arm values, signal polarity, and which motor/side it drives.
- What does each ESC do if its control signal disappears or becomes invalid? Provide its configured failsafe behavior.
- Explain what physical mechanism, if any, disconnects or inhibits motor power during setup and bench checks. The current software maps the pilot switch to controlled spin-down; it does not add an independent power cutoff.
- Confirm the requested controller loop rate and watchdog/reset behavior, if the team has already selected them.

## 5. LEDs and power

- Confirm exact SK9822 strip product, number of strips, installed pixel count on each face, and whether both strips are wired independently or daisy-chained.
- Give Teensy data/clock pin assignments, any level shifter used, strip direction/start pixel, physical front marker location, and pixel order when viewed from each face.
- Give the 5 V LED supply rating, fuse/current limit, expected maximum brightness, ground connection, and connector/wire size if known.
- Send Rutgers/sponsor image files and say whether the team expects a one-row pattern or a multi-row display. The current code can render one-dimensional effects; a detailed 2D logo/video needs a matrix or multiple rows.

## 6. Build and bench setup

- Confirm Rust firmware is the intended Teensy software environment. If the team expects Arduino/C++, say so before board drivers are written.
- List available programming/debug equipment and confirm how the team currently flashes a Teensy 4.0.
- Confirm the robot can be checked first with ESCs/motors disconnected and the weapon mechanically disabled, then secured for a controlled motor test.
- State what instruments are available: multimeter, oscilloscope/logic analyzer, current-limited supply, and USB serial logging.

The most useful first reply is sections 1–4 with exact part numbers, pin numbers, voltages, and the receiver's transmitter-off behavior. Sections 5–6 can follow if those details are not at hand.
