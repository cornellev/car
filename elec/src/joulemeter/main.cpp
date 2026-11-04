#include <cstdint>
#include <cstdio>

#include "chuds/chuds.hpp"
#include "chuds/mcp_transport.hpp"
#include "common/can_health.hpp"
#include "common/can_rx.hpp"
#include "common/interval.hpp"
#include "common/rx_log.hpp"
#include "common/status_led.hpp"
#include "config.hpp"
#include "hardware/adc.h"
#include "pico/stdlib.h"

using namespace chuds::joulemeter;

constexpr std::uint32_t kBlinkHalfPeriodMs = 500;

namespace {

static_assert(kVoltageAdc >= 26 && kVoltageAdc <= 29);
static_assert(kCurrentAdc >= 26 && kCurrentAdc <= 29);

// adc channel is the gpio offset from the adc base at gpio 26
unsigned channel_of(unsigned gpio) { return gpio - 26; }

// average many samples so noise stays under the current/voltage precision spec
constexpr unsigned kOversample = 16;

float read_raw(unsigned gpio) {
    adc_select_input(channel_of(gpio));
    std::uint32_t sum = 0;
    for (unsigned i = 0; i < kOversample; ++i) {
        sum += adc_read();
    }
    return static_cast<float>(sum) / kOversample;
}

float raw_to_volts(float raw) { return raw / kAdcCountsMax * kAdcVref; }

}  // namespace

// joulemeter node: bus current and voltage, integrated to joules, published as telemetry
int main() {
    stdio_init_all();

    adc_init();
    adc_gpio_init(kVoltageAdc);
    adc_gpio_init(kCurrentAdc);

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

    // energy accumulates over a whole run, so integrate in double to hold resolution
    double joules{};
    // energy at the last report
    double report_joules{};
    // time-weighted sums over the current report period, so a slow loop pass counts for its length
    double period_s{};
    double v_sum{};
    double i_sum{};
    absolute_time_t prev = get_absolute_time();
    cev::Interval report{kReportPeriodMs};
    cev::StatusLed status{kStatusLed, kBlinkHalfPeriodMs};
    cev::CanHealth can_health;
    cev::RxLog rx_log{kNodeId};

    while (true) {
        can_health.check(bus);
        for (int i = 0; i < cev::kMaxRxPerPass; ++i) {
            const auto rx = bus.recv();
            if (!rx) {
                break;
            }
            rx_log.log(*rx);
            status.apply_status_blink(*rx, kNodeId);
        }

        const float v_bus = raw_to_volts(read_raw(kVoltageAdc)) * kVoltageDividerRatio;
        const float amps =
            (raw_to_volts(read_raw(kCurrentAdc)) - kCurrentZeroVolts) / kCurrentVoltsPerAmp;

        // integrate over the real elapsed interval
        const absolute_time_t now = get_absolute_time();
        const double dt = static_cast<double>(absolute_time_diff_us(prev, now)) / 1'000'000.0;
        prev            = now;
        joules += static_cast<double>(v_bus * amps) * dt;
        period_s += dt;
        v_sum += static_cast<double>(v_bus) * dt;
        i_sum += static_cast<double>(amps) * dt;

        status.tick();

        if (report.due()) {
            const Telemetry t{static_cast<float>(v_sum / period_s),
                              static_cast<float>(i_sum / period_s),
                              static_cast<float>((joules - report_joules) / period_s),
                              static_cast<float>(joules)};
            bus.send(t);
            std::printf("v=%.2f V  i=%.2f A  p=%.1f W  e=%.1f J\n", t.voltage, t.current, t.power,
                        joules);
            report_joules = joules;
            period_s      = 0;
            v_sum         = 0;
            i_sum         = 0;
        }
    }
}
