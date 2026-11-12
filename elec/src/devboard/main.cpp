#include <array>
#include <cstdint>
#include <cstdio>
#include <optional>

#include "chuds/catalog.hpp"
#include "chuds/frame.hpp"
#include "chuds/ids.hpp"
#include "chuds/mcp_transport.hpp"
#include "chuds/message.hpp"
#include "common/can_health.hpp"
#include "common/can_rx.hpp"
#include "common/interval.hpp"
#include "common/status_led.hpp"
#include "config.hpp"
#include "pico/stdlib.h"

constexpr std::uint32_t kBlinkHalfPeriodMs = 500;
// "7FF##1" plus 64 bytes as hex
constexpr std::size_t kMaxLine = 6 + 2 * chuds::kMaxFdPayload;

namespace {

// value of one hex digit, or -1
int hex_digit(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return -1;
}

// parse a cansend-style fd line, "123##1DEADBEEF", flags digit ignored since the mcp always sends with brs
std::optional<chuds::CanFrame> parse_line(const std::array<char, kMaxLine>& line, std::size_t n) {
    if (n < 6 || line[3] != '#' || line[4] != '#' || hex_digit(line[5]) < 0 || (n - 6) % 2 != 0) {
        return std::nullopt;
    }
    int id = 0;
    for (std::size_t i = 0; i < 3; ++i) {
        const int d = hex_digit(line[i]);
        if (d < 0) {
            return std::nullopt;
        }
        id = id * 16 + d;
    }
    if (id > chuds::kMaxStandardId.raw()) {
        return std::nullopt;
    }
    chuds::CanFrame f{};
    f.id  = chuds::CanId::from_raw(static_cast<std::uint16_t>(id));
    f.len = static_cast<std::uint8_t>((n - 6) / 2);
    for (std::size_t i = 0; i < f.len; ++i) {
        const int hi = hex_digit(line[6 + 2 * i]);
        const int lo = hex_digit(line[7 + 2 * i]);
        if (hi < 0 || lo < 0) {
            return std::nullopt;
        }
        f.data[i] = static_cast<std::uint8_t>(hi * 16 + lo);
    }
    return f;
}

// a serial line being read, kept across loop passes
struct LineReader {
    std::array<char, kMaxLine> line{};
    std::size_t len = 0;
    bool too_long{};
};

// print each waiting bus frame as a line, and act on a StatusBlink to this board
void forward_rx(chuds::Mcp251863Transport& can, cev::StatusLed& status) {
    for (int i = 0; i < cev::kMaxRxPerPass; ++i) {
        const auto rx = can.recv();
        if (!rx) {
            return;
        }
        std::printf("%03X##1", static_cast<unsigned>(rx->id.raw()));
        for (std::size_t j = 0; j < rx->len; ++j) {
            std::printf("%02X", static_cast<unsigned>(rx->data[j]));
        }
        std::printf("\n");
        if (const auto m = chuds::decode(*rx)) {
            status.apply_status_blink(*m, chuds::devboard::kNodeId);
        }
    }
}

// read waiting serial input, and send each complete line onto the bus
void forward_serial(chuds::Mcp251863Transport& can, LineReader& r) {
    for (int c = getchar_timeout_us(0); c != PICO_ERROR_TIMEOUT; c = getchar_timeout_us(0)) {
        if (c == '\r') {
            continue;
        }
        if (c != '\n') {
            if (r.len < r.line.size()) {
                r.line[r.len++] = static_cast<char>(c);
            } else {
                r.too_long = true;
            }
            continue;
        }
        const auto f = r.too_long ? std::nullopt : parse_line(r.line, r.len);
        if (!f) {
            std::printf("bad line\n");
        } else if (!can.send(*f)) {
            std::printf("can tx failed\n");
        }
        r = {};
    }
}

}  // namespace

// serial bridge: every frame from the bus goes out as a line on usb serial, and every line in goes onto the bus
// lines that are not frames are status text for a human
int main() {
    stdio_init_all();

    chuds::Mcp251863Transport can{{
        .sck  = kSpiSck,
        .mosi = kSpiMosi,
        .miso = kSpiMiso,
        .cs   = kMcpCs,
        .stby = kMcpStby,
        .nint = kMcpInt,
    }};

    LineReader reader;
    cev::StatusLed status{kStatusLeds, kBlinkHalfPeriodMs};
    cev::Interval report{1000};
    cev::CanHealth can_health;

    while (true) {
        can_health.check(can);
        forward_rx(can, status);
        forward_serial(can, reader);

        // repeated so a serial console opened after boot still sees it
        if (!can.ready() && report.due()) {
            std::printf("can init failed\n");
        }

        status.tick();
    }
}
