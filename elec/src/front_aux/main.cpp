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
#include "hardware/watchdog.h"
#include "pico/stdlib.h"

using namespace chuds::front_aux;

// steering sensor adc channel, derived from its gpio
static_assert(kSteeringAdc >= 26 && kSteeringAdc <= 29);
constexpr unsigned kSteeringChannel = kSteeringAdc - 26;

constexpr std::uint32_t kPublishPeriodMs   = 100;
constexpr std::uint32_t kBlinkHalfPeriodMs = 500;
// the leader must resend body state within this period or outputs fall back to their safe state
constexpr std::uint32_t kCommandTimeoutMs = 1000;
// a loop pass takes microseconds, so this only trips on a real hang, well inside the 1 s timeout
constexpr std::uint32_t kWatchdogMs = 500;

namespace {

void init_outputs() {
    for (const unsigned pin : {kTurnLeft, kTurnRight, kHeadlightL, kHeadlightR, kHorn}) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, false);
    }
}

// drive this node's outputs from the commanded body state
void apply_body_state(const chuds::BodyState& s) {
    const bool lit        = s.blink_phase();
    const bool headlights = s.headlights();
    gpio_put(kTurnLeft, lit && s.left_turn());
    gpio_put(kTurnRight, lit && s.right_turn());
    gpio_put(kHeadlightL, headlights);
    gpio_put(kHeadlightR, headlights);
    gpio_put(kHorn, s.horn());
}

}  // namespace

// front_aux node: turn/headlight/horn from commands, wheel speed and steering as telemetry
int main() {
    stdio_init_all();
    init_outputs();
    cev::rpm_init(kRpmLeft, kRpmRight);
    adc_init();
    adc_gpio_init(kSteeringAdc);

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
    // a stop latches until reset
    // todo clear on a ratified resume command
    bool stopped{};
    cev::Interval pub{kPublishPeriodMs};
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
                stopped               = true;
                const bool headlights = body.headlights();
                body                  = chuds::BodyState{};
                body.set_headlights(headlights);
                apply_body_state(body);
            } else if (!stopped) {
                if (const auto b = chuds::catalog_body<chuds::BodyState>(m)) {
                    body = *b;
                    apply_body_state(body);
                    command_deadline = make_timeout_time_ms(kCommandTimeoutMs);
                }
            }
            status.apply_status_blink(m, kNodeId);
        }

        // headlights hold, here and on a stop, since going dark is worse than staying lit
        if (time_reached(command_deadline)) {
            const bool headlights = body.headlights();
            body                  = chuds::BodyState{};
            body.set_headlights(headlights);
            apply_body_state(body);
            command_deadline = make_timeout_time_ms(kCommandTimeoutMs);
        }

        status.tick();

        if (pub.due()) {
            const cev::RpmCounts rpm = cev::rpm_take();
            adc_select_input(kSteeringChannel);
            bus.send(Telemetry{rpm.left, rpm.right, adc_read()});
        }
    }
}
