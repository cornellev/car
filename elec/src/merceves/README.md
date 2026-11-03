# Merceves steering

The steering float in `MSG_DRIVE` requests a position in `[-1, 1]`. With the
defaults, `-1` is -900 pulses, `0` is center, and `+1` is +900 pulses.

Configure these constants near the top of `main.cpp`:

- `STEERING_HALF_RANGE_STEPS`: pulses from center to either end (default 900).
- `STEPPER_MAX_FREQ_HZ`: maximum pulse rate (default 17,500 steps/s).
- `STEPPER_ACCELERATION`: acceleration and deceleration (default 40,000 steps/s²).

The controller counts emitted pulses and assumes the steering is centered at boot.
There is no homing or sensor correction, so missed steps or movement while the
driver is disabled are not reflected in the estimate. On arrival, the driver stays
enabled to hold position. A new target inside the stopping distance can be passed
while braking before the controller returns to it; travel stays within the configured
limits. Pulses come from the PWM hardware, one PWM period per step, so their timing
doesn't depend on the main loop. A PWM wrap interrupt counts each pulse as it ends and
loads the period after the next (see `steering_pulses.hpp`).

Keep sending drive commands while moving or holding. The existing 200 ms command
timeout cancels motion: at most two pulses already loaded into the PWM still go out,
then the driver is disabled, preserving the step count. The next valid command resumes from that count. The drivetrain float still
requests speed.

`MSG_TELEMETRY` (`0x02`) returns little-endian `f32` left RPM, `f32` right
RPM, then `i32` steering position in emitted pulses relative to boot, obtained
from `SteeringPulses::position()`, then the AS5600 sample described below. The decoded
frame is 17 bytes including the message type; COBS framing is unchanged. Firmware and
teleop must both use this layout (old 13-byte telemetry is rejected by the new host).

| Decoded byte offset | Field | Type |
| --- | --- | --- |
| 0 | Message type (`0x02`) | `u8` |
| 1 | Left RPM | `f32` |
| 5 | Right RPM | `f32` |
| 9 | Steering pulse estimate | `i32` |
| 13 | Encoder raw angle, 0–4095 | `u16`, little-endian |
| 15 | AS5600 magnetic status register | `u8` |
| 16 | Encoder I2C read succeeded, 0 or 1 | `u8` |

## AS5600 encoder telemetry

Wire SDA to **GP2 (physical pin 4)**, SCL to **GP3 (physical pin 5)**, the breakout's
3.3 V supply connection to **3V3 OUT (physical pin 36)**, and GND to Pico GND (e.g. pin 3).
Use I2C pull-ups to 3.3 V; internal pull-ups are also enabled. GP4 is not used for power.

The firmware reads I2C1 at address `0x36`, using STATUS (`0x0B`) and RAW_ANGLE
(`0x0C`/`0x0D`) together. Pin assignments, bus speed (100 kHz), and transaction timeout
(1 ms per operation) are constants near the encoder code in `main.cpp`. Sampling occurs
once per telemetry frame (100 ms), even without drive commands. Each attempt is bounded
and runs outside the steering interrupt/critical section.

The raw value is absolute within one revolution: it wraps at 4095 -> 0. It is not zeroed,
unwrapped, or converted to steering angle. Status bit 5 is magnet detected, bit 4 is too
weak, and bit 3 is too strong. All readable samples are returned, including warnings.
Read success is independent of magnet status. On an I2C error/short read, raw and status
are zero and read success is 0; the next frame retries, with no stale sample or latched fault.
Encoder measurements do not change steering, drivetrain control, or the watchdog.

Teleop shows `encoder raw=... status=0x..` or `encoder I2C failed` alongside existing
telemetry. Its published steering angle continues to use the existing pulse estimate;
no encoder-based angle or additional Zenoh topic is introduced.

Teleop publishes `cev/sensors/steering/steering_angle` as a JSON float in degrees,
with boot center at zero and counterclockwise positive. Its temporary mapping
assumes positive pulses turn counterclockwise and maps 900 pulses to 45 degrees.
Calibrate `STEERING_DEGREES_PER_PULSE` in `auto/teleop/src/main.rs` before relying
on the angle. This is an estimate from emitted pulses, not an angle sensor.

Run the host motion tests from this directory:

```sh
g++ -std=c++20 -Wall -Wextra -Werror tests/steering_motion_test.cpp -o /tmp/steering_motion_test
/tmp/steering_motion_test
g++ -std=c++20 -Wall -Wextra -Werror tests/steering_pulses_test.cpp -o /tmp/steering_pulses_test
/tmp/steering_pulses_test
g++ -std=c++20 -Wall -Wextra -Werror tests/cobs_test.cpp -o /tmp/cobs_test
/tmp/cobs_test
```
