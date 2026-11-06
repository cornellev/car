# precharge

Firmware for the precharge / isolation board: the board that connects the car's battery
to the rest of the car's high-voltage wiring, safely, when the car turns on.

This README assumes you have never seen the board. Read **What it does** and **Safety**
before touching anything.

---

## What it does

### The problem

The motor controller has large capacitors (about 3 mF) across its power input. An empty
capacitor looks like a dead short for an instant. Connecting a 60 V battery straight to
it would push hundreds of amps for a few milliseconds: that welds relay contacts, blows
fuses and damages the capacitors.

### The fix: precharge, then connect

1. **Precharge.** Connect the battery to the controller through a 500 ohm resistor. The
   resistor limits the current to at most 120 mA, so the capacitors fill up slowly
   (about 4.5 s).
2. **Connect.** Once the controller side (the **bus**) is within 5% of the battery
   voltage, close the main relay, which connects the battery directly. Almost no
   current surges, because the two sides are already at nearly the same voltage.
3. **Hand over.** Turn the precharge path off. The main relay now carries everything.

The firmware watches both voltages the whole time and shuts everything off if anything
looks wrong.

### Words used in this README

| Term | Meaning |
|---|---|
| **Battery** | The car's main battery, 40-60 V. Measured *before* both relays. |
| **Bus** | The wiring on the motor controller's side of the relays. Measured *after* them. |
| **Precharge path** | Battery -> SSR -> 500 ohm resistor -> bus. |
| **SSR** | Solid-state relay (SSR-25DD). Switches the precharge path. |
| **Main relay** | G9KA-1A DC24, the big relay that connects battery to bus directly. |
| **E-stop** | Emergency stop button. Has two contacts: one cuts the main relay in hardware, one tells the Pico. |
| **Latched** | Stays that way until the board is power-cycled, no matter what. |

---

## The circuit

```
                     F1 fuse            main relay (G9KA)
Battery + ───────────[    ]───┬────────────o/ o──────────────┬─── BUS + ──> motor controller +
                              │                              │              (~3 mF inside)
                              └── SSR ─── 500 ohm 10 W ──────┘
                                  (precharge path)

Battery − ───────────────────────────────────────────────────────────────> motor controller −
```

### How the Pico controls it

| What | How |
|---|---|
| **Main relay coil** | 24 V coil. GP2 drives a TIP102 transistor (through 1 k) that switches the coil's low side. A flyback diode sits across the coil. The e-stop's contact A is in series with the coil, so pressing the e-stop opens the main relay even if the firmware is dead. |
| **Main relay hold PWM** | The coil needs full 24 V for 0.1-3 s to close, but overheats if left at 24 V. The firmware applies 100% for 250 ms, then PWMs at ~54% / 20 kHz (about 12.5 V average) to hold it. |
| **SSR** | The SSR's control input runs off 24 V. GP3 switches it through a ZVN3306A MOSFET (with a required 100 k gate-to-source pull-down) or a TIP102 (through 1 k). |
| **Voltage sensing** | Battery and bus each go through a 1 M / 51 k divider (divides by ~20.6, so 60 V reads 2.91 V) with a 100 nF cap at the ADC pin. |
| **E-stop sense** | 3V3(OUT) -> 1 k -> e-stop contact B (normally closed) -> 10 k -> GP4, with 100 nF to GND at the pin. Released = high, pressed = low. A broken wire also reads low, so it counts as pressed. |

### Pico pins

| Pico pin | GPIO | Use |
|---|---|---|
| 4 | GP2 | Main relay (PWM) -> TIP102 base via 1 k |
| 5 | GP3 | SSR driver |
| 6 | GP4 | E-stop sense (low = pressed) |
| 31 | GP26 / ADC0 | **Bus** voltage divider |
| 32 | GP27 / ADC1 | **Battery** voltage divider |
| 33 | AGND | Ground for the two dividers and their 100 nF caps only |
| 36 | 3V3(OUT) | Feeds the e-stop loop through 1 k |
| - | GP25 | Onboard LED (status) |

### Terminal blocks

| Block | Pos 1 | Pos 2 | Pos 3 |
|---|---|---|---|
| **PWR** | +24V IN | 24V GND | GND_REF* |
| **MAIN** | COIL+ (to G9KA coil +) | MAIN_SW (from e-stop contact A) | NC |
| **SSR** | SSR_IN+ | SSR_IN- | NC |
| **SENSE** (60 V!) | BATT (fused battery tap) | NC | BUS |
| **ESTOP** | OUT (to contact B) | IN (from contact B) | NC |

\*GND_REF ties the board ground to battery −. Use it only if 24V GND isn't already battery −
(it is if the 24 V comes from a non-isolated converter running off the main battery). The
board ground must connect to battery − in exactly one place.

---

## Safety

- **60 V DC is enough to hurt.** Work on the board with the battery disconnected.
- **The bus stays charged after shutdown, for up to ~50 minutes.** The only thing draining
  the 3 mF is the 1 M sense divider. Measure the bus with a multimeter before touching it.
- **Never debug (breakpoints) with the battery connected.** A halted Pico leaves the relays
  exactly as they were, with nothing watching.
- **Scope ground clips.** With the Pico on USB, your laptop's ground is battery −. A scope
  ground clip is earthed, so clipping it anywhere except battery − / board GND shorts that
  point. Clip ground only to GND, or run the laptop on battery.
- **Keep `kDevMode = true`** while testing (it's the default). The board then never starts
  precharging until you send `g`, so flashing or replugging can't make the bus live.
- **Use a current-limited bench supply** instead of the battery until the board has been
  proven.

---

## How the firmware behaves

```
power-up
   │
INIT (0.5 s: readings settle, SSR-stuck check)
   │
   ├─ e-stop pressed? ──> WAIT_ESTOP_CLEAR ── released for 1 s ──┐
   │                                                             │
   ├─ battery outside 40-63 V? ──> FAULT                         │
   │                                                             │
   └─ kDevMode? ──> WAIT_START ── 'g' on USB serial ─────────────┤
                                                                 │
PRECHARGE (SSR on; bus rises over ~4.5 s) <──────────────────────┘
   │  bus >= 95% of battery
CLOSE_MAIN (main relay at full 24 V; SSR still on; at 100 ms checks bus == battery)
   │  250 ms
RUN (main relay held at ~54% PWM; SSR off; keeps checking)
```

**ESTOP and FAULT are latched.** The only way out is a power cycle. Releasing the e-stop
never restarts anything, with one exception: if the e-stop was already pressed at
power-up, releasing it (for 1 s) is what starts the sequence.

**What the e-stop does:** the main relay opens in hardware (contact A). On top of that,
the firmware's interrupt turns both outputs off within microseconds and latches ESTOP.

**What the firmware checks:**

| When | Check | Fault |
|---|---|---|
| Boot | Did the last boot end in a crash/hang? | WATCHDOG_RESET |
| Boot | Did the bus rise while the SSR was off? | SSR_STUCK_ON |
| Before starting | Battery between 40 and 63 V? | BATT_UV / BATT_OV |
| Precharge | Bus rising at least 30% as fast as the RC curve predicts (checked every 0.5 s)? | NO_RISE / STALL |
| Precharge | Bus rising *faster* than physically possible through 500 ohm? | SENSOR |
| Precharge | Done within 12 s? | TIMEOUT |
| Close main | Bus within 1.5 V of battery 100 ms after closing? | MAIN_MISMATCH |
| Run | Bus more than 3 V below battery for 50 ms? | DROPOUT |
| Always (while energized) | ADC maxed out, or bus reading above battery? | SENSOR |
| Always (while energized) | Battery < 40 V for 1 s, or > 63 V for 0.2 s? | BATT_UV / BATT_OV |

A watchdog resets the Pico if the main loop ever stops for 100 ms. Both relays turn off
during the reset, and the next boot latches FAULT 1.

---

## Reading the board

### LED (Pico onboard)

| Pattern | Meaning |
|---|---|
| Off | INIT (first 0.5 s) |
| Slow blink, 1 Hz | Waiting: for the e-stop release, or for `g` |
| Fast blink, 5 Hz | Precharging / closing the main relay |
| Solid | RUN |
| Very fast flicker, 10 Hz | ESTOP |
| N blinks, then 2 s off | FAULT number N (table below) |

### USB serial

Connect the Pico over USB and open its serial port. The baud rate doesn't matter.
- **macOS:** `ls /dev/tty.usbmodem*`, then `screen /dev/tty.usbmodemXXXX` (quit: Ctrl-A, then K).
- **Linux:** `screen /dev/ttyACM0` or `picocom /dev/ttyACM0`.
- **Windows:** find the COM port in Device Manager, open it with PuTTY (Serial).

To send `g`, just type `g` in that window. (From another Linux terminal: `printf g > /dev/ttyACM0`.)

The Pico drops output while no program has the port open, so lines from before you
opened it are lost. The status line every 100 ms always shows the current state:

```
WAIT_START batt=58.02 bus=0.31 estop=ok  (send g to start)
```

Other lines:

```
[5123] WAIT_START -> PRECHARGE          state change, [milliseconds since boot]
[9628] PRECHARGE -> CLOSE_MAIN
  precharge took 4505 ms
[9878] CLOSE_MAIN -> RUN
[15012] E-STOP edge in RUN              e-stop input went low
[15013] e-stop 1 ms later: LOW (real)   still low = real press; HIGH = electrical noise
[15013] RUN -> ESTOP
  FAULT 7 STALL: 34.12 (batt 58.10 V, bus 34.12 V)   fault number, name, the reading that tripped it
  bus load ~48.0 mA                                   only for STALL: current something is drawing
```

### Faults

| N | Fault | What it means | Check |
|---|---|---|---|
| 1 | WATCHDOG_RESET | The firmware hung or crashed on the previous boot | Note what it was doing; report it |
| 2 | SENSOR | A reading is impossible: ADC maxed out, bus above battery, or bus jumped | Divider resistors and solder joints, sense wiring, the 100 nF caps, AGND. Also happens if nothing is connected to BUS (no capacitance) |
| 3 | BATT_UV | Battery below 40 V | Battery charge, BATT_SENSE wiring/fuse. **Expected** on the bench with no supply connected |
| 4 | BATT_OV | Battery above 63 V | Charger, divider ratio calibration |
| 5 | SSR_STUCK_ON | Bus rose at boot with the SSR off | SSR shorted, or its output wired backwards (+ must face the battery) |
| 6 | NO_RISE | Bus barely moved during precharge | Shorted bus, open precharge resistor or fuse, SSR not switching (24 V, driver, wiring) |
| 7 | STALL | Bus started rising, then stopped short of 95% | Something on the bus draws current (the log shows how much). Usually the motor controller waking up |
| 8 | TIMEOUT | Precharge took over 12 s | Bus capacitance much larger than 3 mF, or a small load |
| 9 | MAIN_MISMATCH | Main relay didn't close | 24 V, COIL+ / MAIN_SW wiring, e-stop contact A, TIP102 pinout, coil itself |
| 10 | DROPOUT | Main relay opened while running | E-stop contact A or its connector, 24 V supply dipping, or `kDropoutV` too low for the load |

---

## Build and flash

The board must be in the firmware list in `elec/CMakeLists.txt`:

```cmake
add_subdirectory(src/precharge)
```

From `elec/`:

```sh
just build-target precharge   # build
just flash precharge          # build and flash
```

## Files

| File | Job |
|---|---|
| `config.hpp` | Every pin, threshold and timing. Values marked `TUNE` need checking on the hardware. |
| `state_machine.cpp` | All the decisions. Has no hardware code, so it also runs on a laptop. |
| `main.cpp` | Hardware: relay outputs, e-stop interrupt, voltage readings, LED, USB log, watchdog. |
| `test_state_machine.cpp` | Simulates the battery, bus, relays and e-stop through 26 scenarios. Not built by `just`; the command to run it is at the top of the file. Run it after changing `state_machine.cpp` or `config.hpp`. |

## Settings you may need to change (`config.hpp`)

| Setting | Default | Change it when |
|---|---|---|
| `kDevMode` | true | Set false only once the board has been proven on the car |
| `kBattDivRatio`, `kBusDivRatio` | 20.608 | Calibration: apply a known voltage, then `new = old x (meter reading / logged reading)` |
| `kTauS` | 1.5 s | Set to the logged `precharge took` time / 3 |
| `kBattMinV`, `kBattMaxV` | 40, 63 V | Only temporarily, for bench supplies below 40 V. Put them back after |
| `kMainPwmHz` | 20000 | TIP102 runs warm: 5000 |
| `kMainHoldDutyPct` | 54 | Main relay drops out on bumps: raise (max 60) |
| `kDropoutV` | 3.0 V | Must exceed the BATT-to-BUS voltage drop at full load. Measure it |

## Known limits

- A sensor that drifts slowly to a wrong value can't be detected. Recheck the readings
  against a multimeter from time to time.
- If the 3.3 V or USB power to the Pico dips, it resets. With `kDevMode = false` it then
  restarts the sequence by itself, because a power-up is the start signal.
- The bus stays charged long after shutdown (see **Safety**).
