# Rumba driver controls

## Physical controls

| Control | Meaning |
|---|---|
| Left stick vertical | Spin throttle, 0–100% |
| Right stick X/Y | Translation relative to the LED-defined virtual front |
| Enable toggle | ON permits spin; OFF requests controlled spin-down |
| Right stick centered | Spin in place when throttle is raised and enable is ON |

The default `AuxiliaryControlMap` leaves all optional switches unassigned. Up to four two-position switches can be mapped to select `RutgersLogo` or `SponsorLoop` while ON; OFF returns to `VirtualFront` unless another mapped switch is ON. Channel indices and each switch's action are configurable. If multiple mapped switches are ON, the last configured binding has priority. These modes are surfaced to the LED artwork layer only while feedback reports active spin. The virtual-front marker remains the baseline and is overlaid after any artwork.

The default software channel map assumes common AETR channel order: CH1 right X, CH2 right Y, CH3 left Y/throttle, CH5 enable switch. This is only a starting point. Calibrate the actual EdgeTX model and confirm every channel before connecting motor power.

## Virtual front

The LED strip marks the chosen virtual front. The driver uses that light marker as the reference for directional input while the robot is spinning. Both faces display the same logical marker. Decorative effects are drawn underneath the marker and must never obscure it.

## Safety behavior

Turning the enable toggle OFF requests a controlled stop through the existing robot controller. If receiver packets stop or become invalid, the control layer also requests a controlled stop. After radio link recovery, the enable switch must be moved OFF before the system will accept ON again.

The receiver must also be configured for a safe RF-loss output if its firmware supports it. Standard iBUS channel packets do not provide a universal RF failsafe flag, so transmitter-off testing is mandatory.
