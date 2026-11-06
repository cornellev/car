# Sensor shared memory

The Linux writer receives CHUDS CAN-FD telemetry with the repository's
`chuds::Bus<chuds::SocketCanTransport>` and `catalog_body<T>`. It publishes cached
snapshots every 5 ms (200 Hz) with the existing seqlock. `/sensor_shm`, its packed
100-byte block, and `read_shm.py` are unchanged. One writer must own this block.
Each node changes only its own fields; missing nodes retain their latest values.
There is no stale-data timeout in this legacy layout. Before first reception,
measurements are NaN and node timestamps are zero. Motor RPM and duty cycle remain
NaN with timestamp zero because the catalog has no motor message.

## Build and run (Linux / Pi)

Use a C++23 toolchain with `std::expected` support (for example GCC 13 or newer),
CMake 3.21+, and Python 3. No pigpio daemon or library is needed.

```sh
cmake -S telem/shm -B build/telem-shm -DCMAKE_CXX_COMPILER=g++
cmake --build build/telem-shm -j
./build/telem-shm/write_shm           # can0
./build/telem-shm/write_shm vcan0     # positional interface selection
python3 telem/shm/read_shm.py formatted
```

See [CHUDS hardware, wiring, kernel driver and interface setup](../../cev-lib/chuds/README.md).
The Pi CAN controller still uses SPI through the Linux kernel; the writer no
longer polls sensor boards or controls chip selects. Use the CHUDS setup's
500 kbit/s nominal, 2 Mbit/s data and 80% sample points on both phases.
Opening an unavailable or non-FD interface fails startup.

UART GPS remains on `/dev/serial0`; `SHM_GPS_DEVICE` overrides the device for a
bench setup. Existing GNRMC/GPRMC parsing is preserved: latitude/longitude in
degrees, heading in degrees and speed in m/s. Configure UART baud rate and raw
mode externally as before. Missing GPS leaves NaNs and timestamp zero. The UART
reader now accumulates nonblocking reads and can shut down without a GPS fix.

## Conversions and configuration

| Input | Snapshot units / conversion |
| --- | --- |
| front_aux wheel counts | RPM = counts × 60000 / (edges per revolution × window ms) |
| back_aux wheel counts | Same; rear average drives the existing speed filter |
| front_aux steering ADC | steering angle in degrees = ADC × scale + offset |
| back_aux brake ADC | brake pressure in bar = ADC × scale + offset |
| joulemeter current / voltage | A / V, already converted by firmware; current + optional offset |
| filtered speed | m/s; RPM × pi × 0.58166 m / 60, existing alpha 0.05 filter at 200 Hz |

Configuration uses environment variables:

- `SHM_EDGES_PER_REV` defaults to **32, provisional pending elec confirmation**.
- `SHM_WINDOW_MS` defaults to **100**, matching `kPublishPeriodMs` in both
  auxiliary firmwares and `rpm_take()` (rising edges since previous call).
  This is a nominal firmware window, not CAN arrival spacing; scheduling jitter,
  startup windows and lost frames cannot be reconstructed from the catalog.
- `SHM_STEERING_SCALE`, `SHM_STEERING_OFFSET`: degrees/count and degrees.
- `SHM_BRAKE_SCALE`, `SHM_BRAKE_OFFSET`: bar/count and bar.
- Both ADC coefficients must be provided for each measurement. Defaults are NaN;
  ADC outside 0–4095 also produces NaN. **Degrees and bar are the proposed output
  convention, pending confirmation of calibration and consumer expectations.**
  The repository documents a Piher PST-360 and voltage dividers but does not
  supply installed sensor calibration. Do not use test coefficients on the car.
- `SHM_CURRENT_OFFSET_A` defaults to **0**. The old SPI `-0.45 A` correction is
  removed by default, pending a joulemeter bench comparison with elec. Set an
  explicit correction only after testing; no such validation has been performed.

Numeric overrides must be finite; wheel edges and window must be positive.
`errcount` now counts unsuccessful CAN receives other than an empty queue,
including rejected frames and transport faults, rather than failed SPI polls.
It does not indicate missing nodes or guarantee detection of lost frames.

## Timestamps and validity limitations

`global_ts` is local monotonic publication time in microseconds (64 bits).
Per-node timestamps are local monotonic receive times in microseconds truncated
to 32 bits, wrapping approximately every 71.6 minutes; they are not sensor times.
Compare modulo 2^32. A wrapped value can equal zero, so zero alone is not a
permanent validity guarantee.

Front RPM and `steering.ts` advance together on front_aux reception.
**`steering.ts` now timestamps steering angle only**, not brake pressure.
Brake pressure's receive time is `rpm_back.ts`, which advances with back_aux.
Power has the joulemeter receive timestamp. GPS time is local completed RMC
parse time. Cached values and their timestamps survive other nodes' updates.
The legacy layout cannot represent separate validity flags or detailed freshness.

## vcan integration test

Run on Linux with no other `/sensor_shm` writer or consumers relying on live data.
The test overwrites the production shared-memory block and leaves it allocated.

```sh
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan    # once, if absent
sudo ip link set vcan0 up
python3 telem/shm/test_vcan.py ./build/telem-shm/write_shm
```

The test injects CHUDS CAN-FD frames and reads through the unchanged Python reader.
It checks the 100-byte layout, RPM/ADC/current conversions, speed, motor NaNs,
node timestamp independence, cached values, fragmented UART GPS ingestion through
a pseudo-terminal, and clean shutdown. ADC coefficients
in this test are synthetic, not calibration.

## Hardware verification and next PR

**Pi verification is pending.** On the Pi, follow CHUDS hardware setup, build with
the commands above, run the vcan test, then run on can0 with all three boards.
Compare counted wheel revolutions against RPM; measure steering center/endpoints
and brake reference pressures to establish coefficients and units. Compare
joulemeter readings at zero and known loads against a reference meter to settle
whether any current correction is needed. Disconnect one node and confirm other
nodes continue updating while its cached fields remain fixed. Check UART GPS
fixes and SIGTERM shutdown. Record Pi model, OS/compiler, firmware revisions,
coefficients, reference readings and test results before marking this verified.

**Coordination is pending; no timeline or message definitions have been agreed.**
During the first PR, contact elec / CHUDS maintainers to confirm 32 rising edges
per revolution, timing assumptions, ADC calibration/units, current-offset bench
results, and whether motor RPM/duty cycle remain wanted vehicle telemetry.

Prioritize Vehicle Controller support immediately after this migration. Aleena
Xiao owns the project integrating ZED-F9P GPS, ASM330 IMU and MMC5983MA
magnetometer. Coordinate with Aleena and CHUDS maintainers during this PR to
record the implementation timeline, CAN catalog definitions and earliest bench
integration date. Confirm axes/frame conventions, units/scales, rates, message
fragmentation, sensor vs receive timestamps, clock synchronization, GPS fix
quality and per-sensor validity/staleness semantics. All three sensors must be
exposed, particularly IMU and magnetometer measurements for autonomy.

That PR must version/extend the shared-memory layout and update all readers for
new measurements, timestamps and validity, with tests for partial updates,
dropouts and compatibility. Keep UART GPS until the new GPS interface has been
bench validated; then migrate it with an explicit source/validity policy.
Body commands and STOP handling remain outside this writer's scope.
