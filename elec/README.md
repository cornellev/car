# elec

This folder holds the firmware for the car's RP2040 boards.

Each board has its own program in its own folder under `src/` (for example `template-project`).
`src/common/` holds code shared by all boards.

The Pico SDK is vendored at the repo root in `vendor/pico-sdk/` as a git submodule.

## Setup

The setup scripts install the general build tools, like CMake, Ninja, and Just. They do not
install the arm compiler or picotool. The first `just build` downloads the versions pinned in
`elec/cmake/pinned_tools.cmake` into `elec/.tools`. That download is 150 to 300 MB, takes
about 1 GB of disk, and needs internet once. Later builds reuse it. The build always
uses these copies, so another `arm-none-eabi-gcc` on your computer can't break it.

### Windows

- Make sure you have `winget` installed already. If not, install it
- Cd to the `elec/` folder:
  ```powershell
  cd C:\path\to\urban-concept\elec
  ```
- Run the setup script:
  ```powershell
  powershell -ExecutionPolicy Bypass -File setup/setup-windows.ps1
  ```
  It installs anything missing (CMake, Ninja, Just, and Python) with `winget`,
  and fetches the pico-sdk submodules. Windows may pop up permission windows
  during installs. Click yes for all of them.
- Close and open a new terminal, because PATH may not be updated yet.

### macOS

- Install [Homebrew](https://brew.sh) if you don't have it
- cd to the `elec/` folder:
  ```sh
  cd /path/to/urban-concept/elec
  ```
- Run the setup script from the `elec/` folder:
  ```sh
  ./setup/setup-macos.sh
  ```
  It installs CMake, Ninja, and Just with Homebrew, and fetches the pico-sdk submodules.

Intel Macs aren't supported, because Arm does not publish an Intel Mac build of this compiler.

### Linux

On Debian 12 or newer, Ubuntu 24.04 or newer, or Arch, on an x86_64 or ARM64 computer:

- cd to the `elec/` folder:
  ```sh
  cd /path/to/urban-concept/elec
  ```
- Run the setup script from the `elec/` folder:
  ```sh
  ./setup/setup-linux.sh
  ```
  It installs CMake, Ninja, Just, Python, and g++ with `apt` or `pacman`, and fetches the pico-sdk
  submodules. It also adds a udev rule so `just flash` works without root.

On other distros, install CMake 3.21 or newer, Ninja, Just 1.33.0 or newer, Python 3, g++, and
libusb 1.0 with your package manager.

### Nix

Use the provided flake. `nix develop` (or `direnv allow`) sets everything up. It sets
`ELEC_SYSTEM_TOOLS`, so the build uses the flake's arm compiler and picotool instead of downloading
its own. The first configure reads it, and the build folder keeps that choice, so later builds from
a shell or editor outside nix still use the flake's tools.

## Building

From the `elec/` folder:

```sh
just build                         # build every project
just build-target template-project # build just one project
just clean                         # delete the build files and start fresh
just doctor                        # check if tools are installed and working
```

(From the repo root instead, add the `elec` prefix, ie `just elec build`.)

The project name is the folder name under `src/`. After a build, the file to flash onto the board is located at:

```
build/firmware/elec/src/<project>/<project>.uf2
```

The `build/` folder is at the repo root. If a build fails right after installing or switching
tools (for example it uses the wrong compiler), run `just clean` and build again. The old
`build/firmware/` folder caches the previous compiler, so a stale cache can break the next build.

## Flashing (putting firmware on a board)

### Via a command

```sh
just flash template-project
```

This builds the project and loads it onto the board. If the board was already running our firmware,
you don't need to put it into BOOTSEL as this command makes the board automatically reboot into
BOOTSEL first.

On Windows, this may need a USB driver for picotool.

If it says "no accessible RP2 devices", try manually putting the board into BOOTSEL mode (see
below) and running it again. If it still fails, use the manual method.

### Manually flashing

To put a board in BOOTSEL mode:

- Unplug it
- Hold the BOOTSEL button
- Plug it back in while still holding the button down

If the board already runs our firmware, you can instead press its reset button twice within 200 ms.

In BOOTSEL mode the board shows up as a USB drive called **`RPI-RP2`**. Just drag the
project's `.uf2` file (see the path above) onto that drive. The board flashes itself and restarts

If a board lights up but never shows up on USB, try another cable. Some USB cables carry only power.

### Flashing a board plugged into a Pi

The Pi doesn't need picotool. Opening the board's serial port at 1200 baud reboots a board running
our firmware into BOOTSEL. Then copy the `.uf2` onto its drive:

```sh
stty -F /dev/ttyACM0 1200
sudo mount /dev/disk/by-label/RPI-RP2 /mnt
sudo cp front_aux.uf2 /mnt/ && sync
sudo umount /mnt
```

With several boards plugged in, pick one by serial number under `/dev/serial/by-id/`.

## Boards

Each folder under `src/` is the firmware for one PCB. Every pin is in its `config.hpp`. What each
node sends and drives is in [`cev-lib/chuds/README.md`](../cev-lib/chuds/README.md#nodes).

- `devboard`: SPI_2_CANFD dev boards, a USB serial to CAN bridge (see below). Status LEDs on GPIO12
  and GPIO17.
- `front_aux`: FAux_V1.1. Status LED on GPIO25.
- `back_aux`: BAux_V1.1. Status LED on GPIO25.
- `joulemeter`: Joulemeter_V1.4. Status LED on GPIO25.
- `template-project`: no PCB, the starting point for a new board.

### CAN connector and power

Every board has one 4-pin CAN connector with GND and +48 on pins 1 and 2. The CAN pair is not in
the same order on every board, so always connect CAN_P to CAN_P and CAN_N to CAN_N by name.

| Board      | Connector | Pin 3 | Pin 4 | CAN works on USB power alone            |
| ---------- | --------- | ----- | ----- | --------------------------------------- |
| dev board  | J2        | CAN_P | CAN_N | yes                                     |
| front_aux  | J5        | CAN_P | CAN_N | no: also needs 12 V on J3 or 48 V on J5 |
| back_aux   | J4        | CAN_N | CAN_P | no: needs 48 V on J4                    |
| joulemeter | J2        | CAN_N | CAN_P | no: needs 48 V on J2                    |
| RPi_CAN    | J3        | CAN_N | CAN_P | yes, with a USB-A to USB-C cable        |

RPi_CAN's power and Pi wiring are in
[`cev-lib/chuds/README.md`](../cev-lib/chuds/README.md#pi-can-board-rpi_can_v01).

### Known board faults

- **FAux_V1.1:** 3.3 V comes only from USB, so on 48 V alone the RP2040 is off. Keep USB plugged
  in, or add a Schottky diode from +5 to VBUS.
- **BAux_V1.1:** the CAN chip's SPI pins are not wired, so CAN needs bodge wires. The lamp +12 pins
  sit on a separate `12V` net that reaches no supply. The wiper servo connector J9 carries +12 on
  pin 3, so feed a 5 V servo from its own supply and leave J9 pin 3 unconnected.
- **Joulemeter_V1.4:** the CAN chip's 5 V supply never reaches the buck output, so CAN needs a
  bodge wire from a `Buck_Out` pad (C27.1, C28.1, C29.1, or D6 pad 2) to C4 pad 1. The current
  scale in `config.hpp` is a placeholder until the shunt value is known.

### devboard firmware

The dev board firmware prints every CAN frame it receives on USB serial as a line in `cansend`
syntax, such as `123##1AABB`, and sends every such line it reads onto the bus. It replies
`bad line` to a line it can't parse and `can tx failed` when a send fails, and prints
`can init failed` every second if the CAN chip didn't start.

To test a dev board from a Pi, with the board on `/dev/ttyACM0` and `can0` up:

```sh
stty -F /dev/ttyACM0 raw -echo
cat /dev/ttyACM0                         # one terminal: frames the board hears
candump can0                             # another: frames the board sends
printf '123##1DEADBEEF\n' > /dev/ttyACM0 # board to can0
cansend can0 124##1CAFE                  # can0 to board
cansend can0 140##100026400              # StatusBlink: every dev board's LEDs at 100 ms
```

A Pico drops its USB output while no program has the port open, so start `cat` before sending.

## Adding a new board

Each subdirectory of `src/` is an independent project with its own
`CMakeLists.txt`. To add one

- Copy `src/template-project` to a new folder. The folder name becomes the target name
- Write the board in `main.cpp`
- In the board's `CMakeLists.txt`, list extra libraries in `add_board(...)` and any source files
  besides `main.cpp` in `target_sources(...)`
- Register the folder in the firmware inventory in `elec/CMakeLists.txt`
- Compile with `just build-target {project-name}`
- Flash with `just flash {project-name}`

For example, a board in `src/steering/` with `main.cpp`, `pid.cpp`, and `encoder.cpp` has this
`src/steering/CMakeLists.txt`:

```cmake
add_board(hardware_pwm)
target_sources(steering PRIVATE pid.cpp encoder.cpp)
```

## Editor

### VS Code

We use the clangd extension instead of Microsoft's C/C++ IntelliSense because it integrates better
with other platforms' tools, and the CMake Tools extension to build, flash, and test from VS Code.
For setup:

- Build once first, because clangd reads `build/firmware/compile_commands.json`. For the chuds
  library's own tests and examples, also run `just cev-lib chuds build` (macOS and Linux only,
  since it needs a C++ compiler for your computer)
- In the CMake side panel, under Project Status, pick the `firmware` or `host` preset. They are the
  same presets `just` uses, and each board has a `flash_<board>` target
- chuds' tests, examples, and tools show errors on Windows, which has no host build. Its SocketCAN
  code shows errors on macOS and Windows, because it uses Linux-only headers.
  `telem/shm` shows errors anywhere but the Pi, because it needs the Pi's pigpio library
- Open the repo root (`urban-concept/`) in VS Code, not `elec/`
- Install the recommended clangd and CMake Tools extensions when VS Code asks
- If VS Code asks whether you trust the folder, click Trust
- If it says clangd was not found on your PATH, click Install
- Use clangd 19 or newer. Older ones show false errors about `std::expected`, and Ubuntu's `clangd`
  package is 18. If you see those errors, run "clangd: Check for language server update" from the
  command palette. On ARM Linux, which that can't download for, install `clangd-19` or newer with
  your package manager

### Neovim

Use clangd 19 or newer and Neovim 0.11 or newer. No plugin is needed. Build once first, as in the
VS Code steps. Then add this to your config. It passes the same `--query-driver` flag VS Code uses:

```lua
vim.lsp.config('clangd', {
  cmd = { 'clangd', '--query-driver=**/arm-none-eabi-*,**/clang++' },
  filetypes = { 'c', 'cpp' },
  root_markers = { '.clangd', '.git' },
})
vim.lsp.enable('clangd')
```

### Other editors

Pass `--query-driver=**/arm-none-eabi-*,**/clang++` to clangd.

Front aux retries failed CAN initialization five seconds after each failed
attempt, printing `can init failed` again on failure and `can init ok` on recovery.
The retry timer does not pause the main loop between attempts. A controller that
initialized but reports error-passive or bus-off is not an initialization failure
and does not trigger these retries.
