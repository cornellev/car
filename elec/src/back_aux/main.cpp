#include <cstdint>
#include <cstdio>
#include <initializer_list>

#include "chuds/chuds.hpp"
#include "chuds/mcp_transport.hpp"
#include "common/can_health.hpp"
#include "common/can_rx.hpp"
#include "common/interval.hpp"
#include "common/rpm.hpp"
#include "common/rx_log.hpp"
#include "common/status_led.hpp"
#include "config.hpp"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"

using namespace chuds::back_aux;

// brake sensor adc channel, derived from its gpio
static_assert(kBrakeAdc >= 26 && kBrakeAdc <= 29);
constexpr unsigned kBrakeChannel = kBrakeAdc - 26;

constexpr std::uint32_t kPublishPeriodMs   = 100;
constexpr std::uint32_t kBlinkHalfPeriodMs = 500;
// the leader must resend body state within this period or outputs fall back to their safe state
constexpr std::uint32_t kCommandTimeoutMs = 1000;
// a loop pass takes microseconds, so this only trips on a real hang, well inside the 1 s timeout
constexpr std::uint32_t kWatchdogMs = 500;

// servo pwm at 50 hz
// the center pulse, and the half-span at full deflection
constexpr std::uint16_t kWiperCenterUs   = 1500;
constexpr std::uint16_t kWiperHalfSpanUs = 500;

namespace {

// wiper sweep state, so a stop parks the wiper instead of snapping it
struct Wiper {
    int angle{kWiperParkDeg};
    int dir{1};
    bool running{};
    bool parking{};
    absolute_time_t dwell_until{};
};

void set_wiper_deg(int deg) {
    const auto us =
        static_cast<std::uint16_t>(kWiperCenterUs + deg * kWiperHalfSpanUs / kWiperMaxDeg);
    pwm_set_gpio_level(kWiperPwm, us);
}

// step one increment toward target, clamped
int step_toward(int from, int to) {
    if (from < to) {
        return from + kWiperStepDeg < to ? from + kWiperStepDeg : to;
    }
    if (from > to) {
        return from - kWiperStepDeg > to ? from - kWiperStepDeg : to;
    }
    return from;
}

// advance the sweep one tick
// runs even while stopped so a park completes
void wiper_tick(Wiper& w) {
    if (w.parking) {
        if (w.angle != kWiperParkDeg) {
            w.angle = step_toward(w.angle, kWiperParkDeg);
            set_wiper_deg(w.angle);
        }
        return;
    }
    if (!w.running) {
        return;
    }
    if (absolute_time_diff_us(get_absolute_time(), w.dwell_until) > 0) {
        return;
    }
    w.angle += w.dir * kWiperStepDeg;
    if (w.angle >= kWiperMaxDeg) {
        w.angle       = kWiperMaxDeg;
        w.dir         = -1;
        w.dwell_until = make_timeout_time_ms(kWiperDwellMs);
    } else if (w.angle <= kWiperMinDeg) {
        w.angle       = kWiperMinDeg;
        w.dir         = 1;
        w.dwell_until = make_timeout_time_ms(kWiperDwellMs);
    }
    set_wiper_deg(w.angle);
}

void init_outputs() {
    for (const unsigned pin : {kTurnLeft, kTurnRight}) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, false);
    }
    // the car's running led, held on
    gpio_init(kRunningLed);
    gpio_set_dir(kRunningLed, GPIO_OUT);
    gpio_put(kRunningLed, true);

    // 1 us tick from clk_sys, 20000 ticks per frame gives 50 hz
    gpio_set_function(kWiperPwm, GPIO_FUNC_PWM);
    const unsigned slice = pwm_gpio_to_slice_num(kWiperPwm);
    pwm_set_clkdiv(slice, static_cast<float>(clock_get_hz(clk_sys)) / 1'000'000.0f);
    pwm_set_wrap(slice, 19999);
    // set the park pulse before enabling, so the servo never sees a stray first frame
    set_wiper_deg(kWiperParkDeg);
    pwm_set_enabled(slice, true);
}

// drive this node's outputs from the body state
void apply_body_state(const chuds::BodyState& s, Wiper& w) {
    const bool lit = s.blink_phase();
    gpio_put(kTurnLeft, lit && s.left_turn());
    gpio_put(kTurnRight, lit && s.right_turn());
    w.running = s.wiper();
    w.parking = !w.running;
}

}  // namespace

// back_aux node: turn signals and wiper from commands, wheel speed and brake as telemetry
int main() {
    stdio_init_all();
    init_outputs();
    cev::rpm_init(kRpmLeft, kRpmRight);
    adc_init();
    adc_gpio_init(kBrakeAdc);

    chuds::Bus<chuds::Mcp251863Transport> bus{chuds::Mcp251863Transport{{
        .sck  = kSpiSck,
        .mosi = kSpiMosi,
        .miso = kSpiMiso,
        .cs   = kMcpCs,
        .stby = kMcpStby,
        .nint = kMcpInt,
    }}};

    if (!bus.ready()) {
        std::printf("can init failed\n");
    }

    // the commanded body outputs currently applied
    chuds::BodyState body{};
    Wiper wiper{};
    // a stop latches until reset
    // todo clear on a ratified resume command
    bool stopped{};
    cev::Interval pub{kPublishPeriodMs};
    cev::Interval wiper_iv{kWiperTickMs};
    cev::StatusLed status{kStatusLed, kBlinkHalfPeriodMs};
    cev::CanHealth can_health;
    cev::RxLog rx_log{kNodeId};
    absolute_time_t command_deadline = make_timeout_time_ms(kCommandTimeoutMs);

    // a hung loop reboots the board instead of freezing its outputs
    watchdog_enable(kWatchdogMs, true);

    while (true) {
        watchdog_update();
        can_health.check(bus);
        for (int i = 0; i < cev::kMaxRxPerPass; ++i) {
            const auto rx = bus.recv();
            if (!rx) {
                break;
            }
            const chuds::Message& m = *rx;
            rx_log.log(m);
            if (m.cls == chuds::MsgClass::Emergency && m.type == chuds::MsgType::Stop) {
                stopped = true;
                body    = chuds::BodyState{};
                apply_body_state(body, wiper);
            } else if (!stopped) {
                if (const auto b = chuds::catalog_body<chuds::BodyState>(m)) {
                    body = *b;
                    apply_body_state(body, wiper);
                    command_deadline = make_timeout_time_ms(kCommandTimeoutMs);
                }
            }
            status.apply_status_blink(m, kNodeId);
        }

        if (time_reached(command_deadline)) {
            body = chuds::BodyState{};
            apply_body_state(body, wiper);
            command_deadline = make_timeout_time_ms(kCommandTimeoutMs);
        }

        if (wiper_iv.due()) {
            wiper_tick(wiper);
        }

        status.tick();

        if (pub.due()) {
            const cev::RpmCounts rpm = cev::rpm_take();
            adc_select_input(kBrakeChannel);
            bus.send(Telemetry{rpm.left, rpm.right, adc_read()});
        }
    }
}
