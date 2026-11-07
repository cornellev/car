# Centralized Hardware Unification Dictation Scheme (CHUDS)

The car's CAN-FD protocol: a header-only C++23 library that builds unchanged for the RP2040 boards
and for the VCU.

## System

- **VCU (vehicle control unit, the leader):** the car's main computer, a Pi or a Jetson, one on
  the bus at a time. It reaches the bus through its own MCP251863 board, the kernel `mcp251xfd`
  driver, and SocketCAN. It sends `BodyState`, is the only node that sends STOP, and collects
  telemetry.
- **RP2040 boards (followers):** each talks to an MCP251863 over SPI. They publish telemetry and
  drive their outputs from `BodyState`.

## Bus settings

Every node must use the same settings, or frames fail with errors. They are defined once in
`frame.hpp` (`kNominalBitrate`, `kDataBitrate`, `kNominalSamplePoint`, `kDataSamplePoint`), and
`static_assert`s check the RP2040 driver's timing against them.

- **Two speeds.** A CAN-FD frame starts at the nominal rate, **500 kbit/s**, while nodes compete
  for the bus by id. Once one node has won, the data bytes switch to the data rate, **2 Mbit/s**.
- **80% sample point, in both phases.** A node reads each bit 80% of the way through it, after the
  signal has settled from the edge. The remaining 20% is slack that nodes use to stay in step with
  each other.

The RP2040 boards apply these in `Mcp251863Transport`. On the VCU, run this once from the repo root:

```sh
just cev-lib chuds install-can
```

It installs a service that brings `can0` up with these settings whenever it appears, at boot or
when a USB CAN adapter is plugged in. Rerun it after changing `frame.hpp`. It also applies the new
settings right away if `can0` is present.

The service reads the settings from `frame.hpp`, so the VCU can't drift from the boards. It runs
`ip link` with `bitrate`/`sample-point` for the nominal phase and `dbitrate`/`dsample-point` for the
data phase. `fd on` enables CAN-FD frames. `restart-ms 100` restarts the controller 100 ms after it
goes bus-off, instead of leaving it off until someone runs the command again.

If you bring `can0` up by hand, pass the same sample points. The kernel defaults (87.5% and 75%)
make every frame a board sends with the data rate fail at the Pi, while frames the Pi sends still
reach the boards:

```sh
sudo ip link set can0 up type can bitrate 500000 sample-point 0.8 dbitrate 2000000 dsample-point 0.8 fd on restart-ms 100
```

## Bench setup

### Pi CAN board (RPi_CAN_V0.1)

The board's MCP251863 connects to the Pi's SPI0 through its 10-pin J1:

| J1  | Signal       | Pi 5 header pin   |
| --- | ------------ | ----------------- |
| 1   | GND          | 6                 |
| 2   | STBY         | any GND           |
| 3-4 | nINT1, nINT0 | not connected     |
| 5   | nINT         | 22 (GPIO25)       |
| 6   | +3V3         | not connected     |
| 7   | nCS          | 24 (GPIO8, CE0)   |
| 8   | SDO          | 21 (GPIO9, MISO)  |
| 9   | SDI          | 19 (GPIO10, MOSI) |
| 10  | SCK          | 23 (GPIO11, SCLK) |

STBY has no pull-down on the board, so a floating STBY leaves the transceiver in standby.

Power it with 12-48 V on J3 pins 1 (GND) and 2 (+48), or with a USB-A to USB-C cable. Its CC pins
are unconnected, so a USB-C to USB-C cable gives it no power. Use one source, not both: USB and the
buck output share the board's 5 V rail with no diode. With power applied, J1 pin 6 reads 3.3 V.

A Pi's USB ports share one current limit, so RPi_CAN and several boards on the same Pi may knock a
board off USB. If that happens, power RPi_CAN from J3 or a separate USB charger.

The kernel driver probes the board once, when its overlay loads, so power the board first. To load
it without a reboot:

```sh
sudo dtoverlay mcp251xfd spi0-0 interrupt=25 oscillator=40000000
```

To load it at every boot, add `dtoverlay=mcp251xfd,spi0-0,interrupt=25,oscillator=40000000` to
`/boot/firmware/config.txt`.

### Wiring the bus

- Connect CAN_P to CAN_P and CAN_N to CAN_N by name, never by pin number. RPi_CAN's J3 has pin 3
  CAN_N and pin 4 CAN_P, and other boards put them the other way round. A swapped node neither
  hears the bus nor is heard by it.
- Terminate the bus at exactly its two ends. RPi_CAN's termination (R3/R4, 62 ohm each) is always
  fitted, so put it at one end. With only one termination, frames fail at the 2 Mbit/s data rate.
  Every other board's design fits the same termination, but check the board itself, since one dev
  board was built without it. Put one terminated board at the other end, and any board without
  termination in the middle.
- Plugging CAN wires into a live bus can cause one brief error-warning or error-passive state. It
  clears on its own.
- Which boards reach the bus on USB power alone is in the boards table in
  [`elec/README.md`](../../elec/README.md#boards).

### Troubleshooting

- **`Failed to read Oscillator Configuration Register (osc=0x00000000)`** or `osc=0xffffffff` in
  `dmesg`: the Pi got no answer over SPI. Check the board's power and the J1 cable. If the board
  was powered after the Pi probed it, retry with
  `echo spi0.0 | sudo tee /sys/bus/spi/drivers/mcp251xfd/bind`.
- **A flood of `CRC read error` in `dmesg`**: the board lost power while `can0` was up. Run
  `sudo ip link set can0 down`, restore power, unbind and bind as above, then bring `can0` up.
- **Error counters at `tx 128 rx 0`**: the node sent and nobody acknowledged. The other nodes are
  not connected, swapped, or unpowered. When both ends show it, CAN_P and CAN_N are swapped at one
  of them.
- **`can0` goes bus-off as soon as a board joins**: most likely the bus has only one termination.
  Check that the far end board has its resistors.
- **Frames from boards fail at the Pi but frames from the Pi reach the boards**: `can0` was brought
  up with the kernel's default sample points. See [Bus settings](#bus-settings).

## Usage

Include `chuds/chuds.hpp` plus the transport for your platform, then do everything through a
`chuds::Bus`. Start from working code, which is compiled in CI and can't go stale:

- **RP2040 board:** [`template-project/main.cpp`](../../elec/src/template-project/main.cpp) is the
  starting point. [`front_aux/main.cpp`](../../elec/src/front_aux/main.cpp) is a full board that
  sends telemetry and applies `BodyState`.
- **VCU:** [`examples/chuds_demo.cpp`](examples/chuds_demo.cpp) sends a `BodyState` and prints
  what arrives over SocketCAN.

Things the code doesn't say:

- When `recv()` has no message, `rx.error()` says why. `Empty` just means nothing is waiting.
  `chuds::is_fault()` is true only for real problems such as bus-off.
- `catalog_body<T>(m)` reads a catalog body only when `m` has that body's `kClass` and one of its
  addresses, and for a command only when it is an `Update`. `body_as<T>` checks only the body
  size, so read messages with `catalog_body<T>`.
- On the VCU, `bus.open("can0")` connects the transport and `bus.wait(timeout)` blocks until a
  frame arrives. Boards have neither: a board that takes commands polls the chip each loop pass.
- A periodic `send()` may ignore its result, since the next send replaces a lost one. A one-shot
  command such as STOP should check it.
- On a board, `bus.status()` reads the controller's error state: `Active`, `Passive`, or `Off`,
  plus its error counters. `cev::CanHealth` in `elec/src/common` prints it once a second while it
  is not `Active`. On the VCU, `ip -details -statistics link show can0` shows the same.

## Files

The headers are in `include/chuds/`.

| File                      | What it holds                                                 |
| ------------------------- | ------------------------------------------------------------- |
| `chuds.hpp`               | includes everything below except the two transports           |
| `bus.hpp`                 | `Bus<T>`: send, recv, and the controller's status             |
| `catalog.hpp`             | the car's node ids and message bodies, `catalog_body<T>`      |
| `catalog_format.hpp`      | `format_message`, `print_message`                             |
| `message.hpp`             | `Message`, `encode`/`decode`, `make_message<T>`, `body_as<T>` |
| `transport.hpp`           | the `Transport` concept, errors, `is_fault()`, `BusStatus`    |
| `frame.hpp`               | `CanFrame` and the bus settings                               |
| `ids.hpp`                 | `CanId`: class and subaddress                                 |
| `mcp_transport.hpp`       | `Mcp251863Transport`, RP2040 only                             |
| `socketcan_transport.hpp` | `SocketCanTransport`, Linux only                              |

Everything is in namespace `chuds`. The protocol is in [`docs/addressing.md`](docs/addressing.md),
and the rules for body structs are in [`docs/serialization.md`](docs/serialization.md).

## Build

This builds for your computer, not the board, so it needs a C++23 compiler: the Xcode Command Line
Tools on macOS, or g++ on Linux (`elec/setup/setup-linux.sh` installs it). It isn't set up for
Windows. Firmware builds on Windows don't need it.

Run these from `cev-lib/chuds/`, or prefix them with `cev-lib chuds` from the repo root
(`just cev-lib chuds test`):

```sh
just build     # configure + build
just test      # + run the doctest suite
just clean
```

The SocketCAN tests skip when `vcan0` is missing. Set `CHUDS_REQUIRE_CAN=1` to make that a
failure instead, as CI does.

The build goes to `build/host/` at the repo root. The demo needs Linux and a virtual CAN interface.
From the repo root:

```sh
just cev-lib chuds vcan-up
./build/host/cev-lib/chuds/examples/chuds_demo recv vcan0     # one terminal
./build/host/cev-lib/chuds/examples/chuds_demo send vcan0     # another
```

## Nodes

**Provisional until the team ratifies the id table.**

Each node owns one subaddress and publishes its telemetry at `0x2NN`. The VCU broadcasts the car's
body outputs as one `BodyState` bitmask on `0x108`, and each node drives only the outputs it has.
The VCU must resend it on change and well within every second, since boards fall back to safe
states after 1 s without one.

All telemetry bodies start with a `uint32_t ts`: microseconds since the sending
node booted, from its local monotonic clock. Clocks are not synchronized across nodes. The timestamp wraps every 2^32 microseconds
(about 71.6 minutes); compute intervals with unsigned 32-bit subtraction.
Aux wheel speeds are whole RPM, rounded to nearest (saturated at 65535), using 32 rising
pulses per revolution and the actual elapsed sample window. Aux bodies include a zeroed
`uint16_t reserved` after the ADC field to keep the wire representation free of padding.
Telemetry bodies are 12 bytes for each aux node and 20 bytes for the joulemeter.
This changes the wire layout; senders and consumers must use the updated catalog together.

| Node       | Subaddress | Telemetry (`0x2NN`)             | Drives from `BodyState`    |
| ---------- | ---------- | ------------------------------- | -------------------------- |
| front_aux  | `0x10`     | rpm_left, rpm_right, steering   | turn L/R, headlights, horn |
| back_aux   | `0x20`     | rpm_left, rpm_right, brake      | turn L/R, wiper            |
| joulemeter | `0x30`     | voltage, current, power, energy | none                       |

Turn lamps light only while the `kBlinkPhase` bit is set. The VCU toggles it, so every node blinks
in phase. A node that hears no `BodyState` for 1 s falls back to safe states: horn off, wiper
parked, turn signals off, headlights held. A STOP applies the same safe states and latches until
reset.

front_aux, back_aux, and joulemeter also take a `StatusBlink` `Update` on `0x1NN`, their own
subaddress, that sets their status LED's blink half-period in ms. They ignore 0, and the period
returns to the firmware default on reset. Dev boards running `elec/src/devboard` take it too, on
the shared subaddress `0x40`, so one command reaches every one of them.

For bench testing on Linux, `chudsctl` (`build/host/cev-lib/chuds/tools/chudsctl` from the repo
root) sends and prints messages by name. The interface defaults to `can0`, and `-i` picks another:

```sh
./build/host/cev-lib/chuds/tools/chudsctl dump                   # print every message, decoded
./build/host/cev-lib/chuds/tools/chudsctl body headlights left   # hold a BodyState until ctrl-c
./build/host/cev-lib/chuds/tools/chudsctl blink all 100          # every node's status LED half-period, in ms
./build/host/cev-lib/chuds/tools/chudsctl stop                   # STOP, latches until boards reset
```

`body` takes any of `headlights left right hazards horn wiper`. `blink` takes `front_aux`,
`back_aux`, `joulemeter`, `devboard`, or `all`.
