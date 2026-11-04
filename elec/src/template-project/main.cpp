#include <cstdint>
#include <cstdio>

#include "chuds/chuds.hpp"
#include "chuds/mcp_transport.hpp"
#include "common/can_health.hpp"
#include "common/can_rx.hpp"
#include "common/status_led.hpp"
#include "config.hpp"
#include "pico/stdlib.h"

constexpr std::uint32_t kBlinkHalfPeriodMs = 500;

// template node: blinks the LED and drains the bus
int main() {
    stdio_init_all();

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

    cev::StatusLed status{kStatusLed, kBlinkHalfPeriodMs};
    cev::CanHealth can_health;

    while (true) {
        can_health.check(bus);
        for (int i = 0; i < cev::kMaxRxPerPass; ++i) {
            const auto rx = bus.recv();
            if (!rx) {
                break;
            }
            // handle messages addressed to this node here
        }

        status.tick();
    }
}
