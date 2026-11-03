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

`MSG_TELEMETRY` (`0x02`) returns little-endian `f32` left rot/s, `f32` right
rot/s, then `i32` encoder raw and `i32` encoder wrap. The decoded frame is 17
bytes including the message type; COBS framing is unchanged. Wheel speed is
still counted from the magnet sensors and divided by 60 on the way out. The
magnetic encoder is not wired yet: raw is the stepper pulse count from boot
and wrap is 0. Firmware and teleop must both use this layout.

Teleop publishes `cev/sensors/steering/steering_angle` as a JSON float in
degrees, positive left. It treats 900 pulses as the measured 13.5 degree stop
and negates the count, because positive pulses turn clockwise. Replace
`PULSES_PER_DEG` and `COUNTS_PER_DEG` in `auto/teleop/src/main.rs` once the
encoder is counted. Drive commands arriving as radians and m/s are scaled to
the Pico's [-1, 1] position and duty, with 2 m/s at full duty.

Run the host motion tests from this directory:

```sh
g++ -std=c++20 -Wall -Wextra -Werror tests/steering_motion_test.cpp -o /tmp/steering_motion_test
/tmp/steering_motion_test
g++ -std=c++20 -Wall -Wextra -Werror tests/steering_pulses_test.cpp -o /tmp/steering_pulses_test
/tmp/steering_pulses_test
g++ -std=c++20 -Wall -Wextra -Werror tests/cobs_test.cpp -o /tmp/cobs_test
/tmp/cobs_test
```
