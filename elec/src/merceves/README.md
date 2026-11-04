# Merceves steering

The steering float in `MSG_DRIVE` requests a position in `[-1, 1]`. With the
defaults, `-1` is -900 pulses, `0` is center, and `+1` is +900 pulses.

Configure these constants near the top of `main.cpp`:

- `STEERING_HALF_RANGE_STEPS`: pulses from center to either end (default 900).
- `STEPPER_MAX_FREQ_HZ`: maximum pulse rate (default 17,500 steps/s).
- `STEPPER_ACCELERATION`: acceleration and deceleration (default 40,000 steps/s²).

On the first valid `MSG_DRIVE` command in each communication session, steering
homes before accepting steering or drivetrain targets. Keep sending commands during
homing: the 200 ms watchdog remains active. The drivetrain stays stopped throughout.
No homing movement occurs on boot or merely opening USB.

Homing sends positive (right) pulses at 500 Hz, outside the normal pulse soft limits.
The encoder must remain in the inclusive **1700–2100** window continuously
for **200 ms** to mark the right wall. Leaving the window resets the timer.
Movement within the window is allowed; no stall/noise threshold is applied.
The firmware drains pending pulses, records the reading, and pulses left toward
the fixed AS5600 center **3220** (within 16 encoder counts, plus up to two pending
pulses). Incremental feedback handles encoder wrap during the return to center.
The left bound of 3006 is not used to calculate a midpoint. These unequal ranges do not change the existing
±900 pulse limits for normal driving.

Homing requires successful I2C reads with magnet detected. Weak/strong field
warnings are ignored for homing and remain visible in telemetry. Invalid feedback or a 15 second homing timeout stops steering and leaves
the drivetrain off until a command timeout followed by a new session retries.
A communication timeout cancels homing and invalidates the center; the next valid
command homes again. Check on hardware that positive pulses turn right, and tune
the homing pulse rate, right-wall window, and center tolerance for the mechanism.

After homing, emitted pulse counts are rebased to zero at the measured center.
Normal driving remains open loop, so missed steps or movement while disabled are
not reflected in the estimate. On arrival, the driver stays
enabled to hold position. A new target inside the stopping distance can be passed
while braking before the controller returns to it; travel stays within the configured
limits. Pulses come from the PWM hardware, one PWM period per step, so their timing
doesn't depend on the main loop. A PWM wrap interrupt counts each pulse as it ends and
loads the period after the next (see `steering_pulses.hpp`).

Keep sending drive commands while moving or holding. The existing 200 ms command
timeout cancels motion: at most two pulses already loaded into the PWM still go out,
then the driver is disabled, preserving the step count. The next valid command starts homing again. The drivetrain float still
requests speed.

`MSG_TELEMETRY` (`0x02`) returns little-endian `f32` left RPS, `f32` right
RPS, then `i32` steering position in emitted pulses relative to the most recent homed center (boot until first homing), obtained
from `SteeringPulses::position()`, then the AS5600 sample described below. The decoded
frame is 21 bytes including the message type; COBS framing is unchanged. Firmware and
teleop must both use this layout (old 13/17-byte telemetry is rejected by the new host).

| Decoded byte offset | Field | Type |
| --- | --- | --- |
| 0 | Message type (`0x02`) | `u8` |
| 1 | Left RPS | `f32` |
| 5 | Right RPS | `f32` |
| 9 | Steering pulse estimate | `i32` |
| 13 | Encoder raw angle, 0–4095 | `u16`, little-endian |
| 15 | AS5600 magnetic status register | `u8` |
| 16 | Encoder I2C read succeeded, 0 or 1 | `u8` |
| 17 | Encoder steering angle, degrees, positive left | `f32` |

## AS5600 encoder telemetry

Wire SDA to **GP2 (physical pin 4)**, SCL to **GP3 (physical pin 5)**, the breakout's
3.3 V supply connection to **3V3 OUT (physical pin 36)**, and GND to Pico GND (e.g. pin 3).
Use I2C pull-ups to 3.3 V; internal pull-ups are also enabled. GP4 is not used for power.

The firmware reads I2C1 at address `0x36`, using STATUS (`0x0B`) and RAW_ANGLE
(`0x0C`/`0x0D`) together. Pin assignments, bus speed (100 kHz), and transaction timeout
(1 ms per operation) are constants near the encoder code in `main.cpp`. Encoder sampling occurs every 5 ms, even without drive commands. Telemetry
returns the latest sample every 100 ms. Each attempt is bounded
and runs outside the steering interrupt/critical section.

The raw field is absolute within one revolution: it wraps at 4095 -> 0. Status bit 5 is magnet detected, bit 4 is too
weak, and bit 3 is too strong. All readable samples are returned, including warnings.
Read success is independent of magnet status. On an I2C error/short read, raw and status
are zero and read success is 0; the next frame retries, with no stale sample or latched fault.
Encoder measurements control steering during homing; normal drive control uses pulse counts.

The Pico also returns an encoder angle using
`STEERING_DEGREES_PER_ENCODER_INCREMENT = 0.00553967623263` in
`steering_angle.hpp`, with calibrated center 3220. Increasing encoder values turn
right, so angle is negative right and positive left. Homing establishes the center's
revolution; subsequent 5 ms samples accumulate signed deltas corrected for wrap.
This tracks travel beyond half or one encoder revolution without using pulse counts.
It assumes less than 2048 encoder counts of movement between valid samples.

The angle field is NaN before homing or while encoder feedback is invalid. A gap
longer than 100 ms invalidates wrap tracking until homing runs again. Weak/strong
magnet warnings remain ignored; I2C success and magnet detection are required.
The measured angle retains any small offset from 3220 when homing stops within its
center tolerance. Teleop displays this angle and publishes finite values on
`cev/sensors/steering/steering_angle` as JSON degrees. It skips angle publication
when unavailable. Pulse telemetry remains available for diagnostics.

Rebuild both firmware and teleop for the new 21-byte telemetry layout.

Run the host motion tests from this directory:

```sh
g++ -std=c++20 -Wall -Wextra -Werror tests/steering_motion_test.cpp -o /tmp/steering_motion_test
/tmp/steering_motion_test
g++ -std=c++20 -Wall -Wextra -Werror tests/steering_pulses_test.cpp -o /tmp/steering_pulses_test
/tmp/steering_pulses_test
g++ -std=c++20 -Wall -Wextra -Werror tests/cobs_test.cpp -o /tmp/cobs_test
/tmp/cobs_test
```

Homing state tests:

```sh
g++ -std=c++20 -Wall -Wextra -Werror tests/steering_homing_test.cpp -o /tmp/steering_homing_test
/tmp/steering_homing_test
```

Encoder angle tests:

```sh
g++ -std=c++20 -Wall -Wextra -Werror tests/steering_angle_test.cpp -o /tmp/steering_angle_test
/tmp/steering_angle_test
```

Wheel speeds in telemetry are rotations per second (RPS), calculated as
encoder edges / 68 / elapsed seconds. Teleop displays RPS and publishes the values
directly. Existing Zenoh wheel topic names retain their `_rpm` suffix for subscriber
compatibility; their payloads remain RPS. Update both firmware and teleop together
when switching from the previous RPM serial telemetry.
