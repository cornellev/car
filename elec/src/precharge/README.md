# precharge

Firmware for the precharge/isolation board. Charges the bus through the solid state relay (SSR) and 500 Ω
resistor, closes the big relay (G9KA) once the bus is within 5% of the battery, then
turns the SSR off.

From the repository root, run `just elec build-target precharge`.

## Pins

| GPIO | Purpose | Notes |
| ---- | ------ | ----- |
| 2 | Main relay | PWM to the TIP102 base thru 1k res (current to be amplified) |
| 3 | SSR | Driver gate/base (turn on/off SSR for precharge) |
| 4 | E-stop | Input with pull-down. High = OK, low = pressed or wire broken. Falling edge IRQ |
| 26 (ADC0) | Bus voltage | 1M/51k divider, 100 nF at the pin |
| 27 (ADC1) | Battery voltage | 1M/51k divider, 100 nF at the pin |

Onboard LED at pin 25. All pins, thresholds and timings are in `config.hpp`.

## Running Assumptions

Battery is connected and has voltage between 40-63V; E-stop connects to both E-stop 1 on Isolation board and GPIO 4 and is open circuit when tripped; nothing on the bus draws more than ~6 mA during precharge; 24 V is present whenever the Pico is on.

## State machine

| State | SSR | Main relay | Next |
| ----- | --- | ---------- | ---- |
| `INIT` | off | off | After 500 ms: `WAIT_ESTOP_CLEAR` if the e-stop is pressed, otherwise start* |
| `WAIT_ESTOP_CLEAR` | off | off | E-stop released for 1 s: start* |
| `WAIT_START` | off | off | `g` on USB serial: start*. Only used when `kDevMode` is true |
| `PRECHARGE` | on | off | Bus ≥ 95% of battery: `CLOSE_MAIN` |
| `CLOSE_MAIN` | on | 100% | After 250 ms: `RUN` |
| `RUN` | off | ~54% PWM | |
| `ESTOP` | off | off | Latched until power cycle |
| `FAULT` | off | off | Latched until power cycle |

\*Start checks the battery is 40 to 63 V and the readings are sane, waits for `g` over serial if `kDevMode` is true, arms the outputs, then goes to `PRECHARGE`.

The main relay gets full coil voltage for 250 ms to pull in, then drops to ~54% PWM at
20 kHz to hold, because the G9KA can't sit at full voltage continuously (per datasheet, between 45% and 60%).

Once the outputs are armed, an e-stop in any state goes to `ESTOP`. Before arming, a
pressed e-stop holds the board in `INIT`/`WAIT_ESTOP_CLEAR`, which is the only case
where releasing the e-stop starts anything.

### Faults

The LED blinks the fault number, then pauses

| # | Fault | Trips when |
| - | ----- | ---------- |
| 1 | `WATCHDOG_RESET` | The last boot ended in a watchdog timeout (our loop failed somewhere) |
| 2 | `SENSOR` | bus > battery + 2 V, or the bus rose faster than the RC allows |
| 3 | `BATT_UV` | Battery < 40 V at start, or for 1 s while energized |
| 4 | `BATT_OV` | Battery > 63 V at start, or for 200 ms while energized |
| 5 | `SSR_STUCK_ON` | Bus rose > 1 V during `INIT` with the SSR off |
| 6 | `NO_RISE` | Precharge stalled below 25% of battery (shorted bus or open path) |
| 7 | `STALL` | Bus rose < 30% of the expected RC rise over 500 ms (something is loading the bus) |
| 8 | `TIMEOUT` | Precharge took > 12 s |
| 9 | `MAIN_MISMATCH` | Battery and bus differ by > 1.5 V, 100 ms into `CLOSE_MAIN` |
| 10 | `DROPOUT` | Bus > 3 V below battery for 50 ms in `RUN` |

Onboard LED: off in `INIT`, 1 Hz while waiting, 5 Hz in `PRECHARGE`/`CLOSE_MAIN`,
solid in `RUN`, 10 Hz in `ESTOP`.

## Implementation

| File | |
| ---- | - |
| `config.hpp` | Constants |
| `state_machine.cpp` | `precharge::Controller`; handles logic transitions for behavior of code |
| `main.cpp` | Performs intended behavior in each state: relay outputs, e-stop IRQ, ADC, LED, logging, main loop |
| `test_state_machine.cpp` | Host tests for the state machine. Command at the top of the file |

### Main loop

Every 1 ms, `main()` reads both voltages and the e-stop pin into `precharge::Inputs`, calls
`Controller::step()`, and applies the returned `Outputs` with `set_outputs()`. Then it logs,
updates the LED, and feeds the watchdog (100 ms).

### Output gating

`set_outputs()` won't drive either relay unless `g_armed` is set and `g_locked` isn't,
regardless of what the state machine asks for.

- `g_armed` is set by `arm()` when the state machine requests it, and only if the e-stop
  pin reads high. The pin is checked with interrupts off.
- `g_locked` is set by the e-stop IRQ (once armed) or when the state machine enters
  `ESTOP`/`FAULT`, and never cleared.

`set_outputs()` also runs with interrupts off, so the e-stop IRQ can't land between the
check and the pin write.

The main relay pin is a plain SIO output driven low when off. It's switched to the PWM
function only when on. Pull-in sets the PWM level above wrap (100%), hold sets it to
`kMainHoldDutyPct`.

### E-stop

`estop_isr` is the exclusive `IO_IRQ_BANK0` handler and runs from RAM. It calls
`kill_outputs()`, which sets both relay outputs low (GP2 main relay, GP3 SSR) with direct register writes. The main loop
also reads the pin level every tick, which catches an e-stop that was already pressed at
boot. The e-stop's second contact sits in the main relay's coil circuit, so the relay
opens in hardware either way.

### Startup

`relays_init()` runs, driving both SSR Fet gate and BJT base low. Then, if
`watchdog_enable_caused_reboot()` is true, the state machine starts in `FAULT 1`, so a
crash doesn't lead to an automatic restart. Then the watchdog, USB stdio, ADC and e-stop
IRQ start. `isr_hardfault` kills the outputs and spins until the watchdog resets the chip.

`kDevMode` defaults to true for testing so that flashing or reset never starts a precharge on its
own, set to false when hooking up to car.

### Sensing

Each tick, per channel: switch the ADC input, discard one sample, average 16. The divider
ratios (`kBattDivRatio`, `kBusDivRatio`) need calibrating per board.

### Output Monitoring

Print statement over USB serial every 100 ms with current status. Also done for each state change, fault and
e-stop edge. Each e-stop edge also logs the pin level 1 ms later, which tells a real press
from noise. `PICO_STDIO_USB_STDOUT_TIMEOUT_US` is set to 1 ms in `CMakeLists.txt` so a
terminal that stops reading can't block long enough to trip the watchdog.