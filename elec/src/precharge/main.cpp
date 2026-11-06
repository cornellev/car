// Precharge / isolation board: closes the battery onto the car's bus through a precharge
// resistor, then closes the main relay once the bus is charged. All decisions are in
// state_machine.cpp; this file is the hardware around it.
//
// Startup order matters:
//   1. relays off (before anything else)
//   2. note whether a watchdog timeout caused this boot, then start the watchdog
//   3. USB serial, LED, ADC, e-stop interrupt
//   4. every 1 ms: read -> decide -> drive relays -> log -> feed watchdog

#include <cstdint>
#include <cstdio>

#include "common/interval.hpp"
#include "config.hpp"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/structs/io_bank0.h"
#include "hardware/structs/sio.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "state_machine.hpp"

using precharge::Fault;
using precharge::Main;
using precharge::State;

namespace {

// ---------------------------------------------------------------------------
// Relays. The only code that drives kMainRelayPin / kSsrPin.
// Nothing energizes until arm() succeeds. Once g_locked is set (e-stop ISR while armed,
// or a FAULT / ESTOP), nothing energizes again until reset.
// ---------------------------------------------------------------------------
volatile bool g_armed      = false;
volatile bool g_locked     = false;
std::uint16_t g_level_full = 0;
std::uint16_t g_level_hold = 0;

void relays_init() {
    constexpr unsigned kPins[] = {kMainRelayPin, kSsrPin};
    for (const unsigned pin : kPins) {
        gpio_init(pin);  // SIO, output value 0
        gpio_put(pin, false);
        gpio_set_dir(pin, GPIO_OUT);
    }
    // PWM slice ready at level 0; the pin stays on SIO (low) until set_outputs hands it over.
    const std::uint32_t wrap = clock_get_hz(clk_sys) / kMainPwmHz - 1;
    hard_assert(wrap > 100 && wrap < 65535);
    g_level_full         = static_cast<std::uint16_t>(wrap + 1);  // above wrap: always high
    g_level_hold         = static_cast<std::uint16_t>((wrap + 1) * kMainHoldDutyPct / 100);
    const unsigned slice = pwm_gpio_to_slice_num(kMainRelayPin);
    pwm_config cfg       = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, static_cast<std::uint16_t>(wrap));
    pwm_init(slice, &cfg, false);
    pwm_set_gpio_level(kMainRelayPin, 0);
    pwm_set_enabled(slice, true);
}

// Interrupts are off between "is it allowed?" and "drive the pin", so the e-stop ISR
// can't land in between.
void set_outputs(Main main, bool ssr) {
    const std::uint32_t irq = save_and_disable_interrupts();
    if (!g_armed || g_locked) {
        main = Main::Off;
        ssr  = false;
    }
    gpio_put(kSsrPin, ssr);
    if (main == Main::Off) {
        gpio_set_function(kMainRelayPin, GPIO_FUNC_SIO);  // SIO output is 0: pin low
        pwm_set_gpio_level(kMainRelayPin, 0);
    } else {
        pwm_set_gpio_level(kMainRelayPin, main == Main::PullIn ? g_level_full : g_level_hold);
        gpio_set_function(kMainRelayPin, GPIO_FUNC_PWM);
    }
    restore_interrupts(irq);
}

// Arms only if the e-stop pin reads released, checked with interrupts off.
void arm() {
    const std::uint32_t irq = save_and_disable_interrupts();
    // drop any stale falling-edge event from before arming
    io_bank0_hw->intr[kEstopPin / 8] = GPIO_IRQ_EDGE_FALL << (4 * (kEstopPin % 8));
    if (!g_locked && gpio_get(kEstopPin)) g_armed = true;
    restore_interrupts(irq);
}

// From RAM with direct register writes: used by the e-stop ISR and the HardFault handler.
// relays_init() left both pins as SIO outputs at 0, so switching the main pin back to
// SIO is enough to drive it low.
void __not_in_flash_func(kill_outputs)() {
    if (g_armed) g_locked = true;
    io_bank0_hw->io[kMainRelayPin].ctrl = GPIO_FUNC_SIO << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
    sio_hw->gpio_clr                    = (1u << kMainRelayPin) | (1u << kSsrPin);
}

// ---------------------------------------------------------------------------
// E-stop: hardware 10k + 100nF filter in front of the pin and the pad's Schmitt trigger
// (on by default), so the first falling edge is acted on immediately.
// ---------------------------------------------------------------------------
volatile std::uint32_t g_estop_edges = 0;

// The exclusive IO_BANK0 handler, so the vector points straight at this RAM function.
void __not_in_flash_func(estop_isr)() {
    const unsigned reg         = kEstopPin / 8;
    const unsigned shift       = 4 * (kEstopPin % 8);
    const std::uint32_t events = (io_bank0_hw->proc0_irq_ctrl.ints[reg] >> shift) & 0xFu;
    io_bank0_hw->intr[reg]     = events << shift;  // acknowledge
    if (events & GPIO_IRQ_EDGE_FALL) {
        kill_outputs();
        g_estop_edges = g_estop_edges + 1;
    }
}

void estop_init() {
    gpio_init(kEstopPin);
    gpio_pull_down(kEstopPin);  // broken wire reads low = e-stopped
    gpio_set_irq_enabled(kEstopPin, GPIO_IRQ_EDGE_FALL, true);
    irq_set_exclusive_handler(IO_IRQ_BANK0, estop_isr);
    irq_set_priority(IO_IRQ_BANK0, 0);  // highest
    irq_set_enabled(IO_IRQ_BANK0, true);
}

// ---------------------------------------------------------------------------
// Voltage sensing: discard one sample after switching the ADC input (it can still hold
// charge from the other channel), then average.
// ---------------------------------------------------------------------------
float read_volts(unsigned adc_input, float div_ratio) {
    adc_select_input(adc_input);
    static_cast<void>(adc_read());
    std::uint32_t sum = 0;
    for (int i = 0; i < kAdcSamples; ++i) sum += adc_read();
    return static_cast<float>(sum) / kAdcSamples * (kAdcVref / 4096.0f) * div_ratio;
}

// ---------------------------------------------------------------------------
// LED: off INIT, 1 Hz waiting, 5 Hz precharging, solid RUN, 10 Hz ESTOP,
// FAULT = blink the fault number, pause 2 s.
// ---------------------------------------------------------------------------
bool led_on(State s, Fault f, std::uint64_t ms) {
    switch (s) {
        case State::Init: return false;
        case State::WaitEstopClear:
        case State::WaitStart: return ms % 1000 < 500;
        case State::Precharge:
        case State::CloseMain: return ms % 200 < 100;
        case State::Run: return true;
        case State::Estop: return ms % 100 < 50;
        case State::Fault: {
            const std::uint64_t n     = static_cast<std::uint64_t>(f);
            const std::uint64_t phase = ms % (n * 400 + 2000);
            return phase < n * 400 && phase % 400 < 200;
        }
    }
    return false;
}

bool start_requested() {
    bool got = false;
    for (int c = getchar_timeout_us(0); c != PICO_ERROR_TIMEOUT; c = getchar_timeout_us(0)) {
        if (c == 'g' || c == 'G') got = true;
    }
    return got;
}

std::uint64_t now_ms() { return time_us_64() / 1000; }

}  // namespace

// Any crash: relays off, then hang so the watchdog resets the chip. The next boot sees
// the watchdog reset and latches FAULT 1.
extern "C" void __not_in_flash_func(isr_hardfault)() {
    kill_outputs();
    while (true) {
        __asm volatile("nop");
    }
}

int main() {
    relays_init();

    // Only a timeout of the watchdog started below counts. picotool and BOOTSEL reboots
    // (including `just flash`) also go through the watchdog but don't count.
    const bool watchdog_reboot = watchdog_enable_caused_reboot();
    // A debugger halt pauses the watchdog and freezes the relays as they are:
    // never debug with the battery connected.
    watchdog_enable(kWatchdogMs, true);

    stdio_init_all();
    gpio_init(kStatusLed);
    gpio_set_dir(kStatusLed, GPIO_OUT);
    adc_init();
    adc_gpio_init(kBattAdcPin);  // also turns off the pin's pulls and digital input
    adc_gpio_init(kBusAdcPin);
    estop_init();

    precharge::Controller sm{watchdog_reboot, kDevMode, now_ms()};
    cev::Interval report{kReportPeriodMs};
    std::uint32_t seen_edges   = g_estop_edges;
    bool check_edge_level      = false;
    std::uint64_t next_tick_us = time_us_64();

    while (true) {
        const std::uint64_t ms = now_ms();
        const float batt       = read_volts(kBattAdcInput, kBattDivRatio);
        const float bus        = read_volts(kBusAdcInput, kBusDivRatio);
        const precharge::Inputs in{
            .now_ms        = ms,
            .batt_v        = batt,
            .bus_v         = bus,
            .estop_low     = !gpio_get(kEstopPin),
            .estop_latched = g_locked,
            .armed         = g_armed,
            .start_cmd     = start_requested(),
        };

        const precharge::Outputs out = sm.step(in);
        if (sm.state == State::Fault || sm.state == State::Estop) {
            g_locked = true;
            set_outputs(Main::Off, false);
        } else {
            if (out.arm_request) arm();
            set_outputs(out.main, out.ssr);
        }

        // ---- log (USB serial) ----
        if (check_edge_level) {  // 1 ms after an edge: still low = real press
            std::printf("[%llu] e-stop 1 ms later: %s\n", static_cast<unsigned long long>(ms),
                        in.estop_low ? "LOW (real)" : "HIGH (glitch?)");
            check_edge_level = false;
        }
        if (g_estop_edges != seen_edges) {
            seen_edges       = g_estop_edges;
            check_edge_level = true;
            std::printf("[%llu] E-STOP edge in %s\n", static_cast<unsigned long long>(ms),
                        precharge::name(sm.state));
        }
        if (sm.changed) {
            std::printf("[%llu] %s -> %s\n", static_cast<unsigned long long>(ms),
                        precharge::name(sm.prev_state), precharge::name(sm.state));
            if (sm.state == State::CloseMain) {
                std::printf("  precharge took %lu ms\n",
                            static_cast<unsigned long>(sm.precharge_ms));
            }
            if (sm.state == State::Fault) {
                std::printf("  FAULT %d %s: %.2f (batt %.2f V, bus %.2f V)\n",
                            static_cast<int>(sm.fault), precharge::name(sm.fault),
                            static_cast<double>(sm.fault_value), static_cast<double>(batt),
                            static_cast<double>(bus));
                if (sm.fault == Fault::Stall) {
                    std::printf("  bus load ~%.1f mA\n",
                                static_cast<double>((batt - bus) / kPrechargeOhm * 1000.0f));
                }
            }
        }
        if (report.due()) {
            std::printf("%s batt=%.2f bus=%.2f estop=%s%s\n", precharge::name(sm.state),
                        static_cast<double>(batt), static_cast<double>(bus),
                        in.estop_low ? "LOW" : "ok",
                        kDevMode && sm.state == State::WaitStart ? "  (send g to start)" : "");
        }
        gpio_put(kStatusLed, led_on(sm.state, sm.fault, ms));

        watchdog_update();

        next_tick_us += kTickUs;
        const std::int64_t slack = static_cast<std::int64_t>(next_tick_us - time_us_64());
        if (slack > 0) {
            busy_wait_us(static_cast<std::uint64_t>(slack));
        } else if (slack < -5000) {
            next_tick_us = time_us_64();  // fell far behind: resync instead of bursting
        }
    }
}
