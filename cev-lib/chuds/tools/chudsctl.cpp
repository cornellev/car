#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string_view>
#include <system_error>
#include <thread>
#include <utility>

#include "chuds/chuds.hpp"
#include "chuds/socketcan_transport.hpp"

namespace {

using Bus = chuds::Bus<chuds::SocketCanTransport>;

// how often body is resent, well inside the boards' 1 s timeout
constexpr auto kBodyResend = std::chrono::milliseconds{300};
// half a turn-signal blink period
constexpr auto kBlinkHalf = std::chrono::milliseconds{333};
// the vcu has no node id in the catalog, so its stops carry sender 0
constexpr std::uint8_t kSender = 0x00;

struct NamedValue {
    std::string_view name;
    std::uint8_t value;
};

// body output bits by command-line name
constexpr std::array<NamedValue, 6> kOutputs = {{
    {"headlights", chuds::BodyState::kHeadlights},
    {"left", chuds::BodyState::kLeftTurn},
    {"right", chuds::BodyState::kRightTurn},
    {"hazards", chuds::BodyState::kLeftTurn | chuds::BodyState::kRightTurn},
    {"horn", chuds::BodyState::kHorn},
    {"wiper", chuds::BodyState::kWiper},
}};

// node ids by command-line name
constexpr std::array<NamedValue, 4> kNodes = {{
    {"front_aux", chuds::front_aux::kNodeId},
    {"back_aux", chuds::back_aux::kNodeId},
    {"joulemeter", chuds::joulemeter::kNodeId},
    {"devboard", chuds::devboard::kNodeId},
}};

volatile std::sig_atomic_t g_stop = 0;

void on_sigint(int /*sig*/) { g_stop = 1; }

void usage(const char* prog) {
    std::fprintf(stderr,
                 "usage: %s [-i iface] command\n"
                 "\n"
                 "commands:\n"
                 "  dump                 print every message on the bus, decoded\n"
                 "  body [outputs...]    hold a body state until ctrl-c\n"
                 "                       outputs: headlights left right hazards horn wiper\n"
                 "  blink NODE MS        set a node's status LED blink half-period\n"
                 "                       nodes: front_aux back_aux joulemeter devboard all\n"
                 "  stop [SEVERITY]      send a STOP, 0 (default) is the most severe\n"
                 "                       boards latch it until they reset\n"
                 "\n"
                 "iface defaults to can0\n",
                 prog);
}

// a whole decimal number that fits in T
template <class T>
bool parse(std::string_view s, T& out) {
    const auto r = std::from_chars(s.data(), s.data() + s.size(), out);
    return r.ec == std::errc{} && r.ptr == s.data() + s.size();
}

void report(const char* what, const chuds::TxResult& st) {
    if (st) {
        std::printf("sent %s\n", what);
    } else {
        std::fprintf(stderr, "send %s failed: %d\n", what, std::to_underlying(st.error()));
    }
}

int run_dump(Bus& bus) {
    while (g_stop == 0) {
        if (!bus.wait(std::chrono::milliseconds{200})) {
            continue;
        }
        for (auto r = bus.recv(); r || r.error() != chuds::RxError::Empty; r = bus.recv()) {
            if (r) {
                chuds::print_message(*r);
            } else if (r.error() == chuds::RxError::Error) {
                std::fprintf(stderr, "recv error: %d\n", std::to_underlying(r.error()));
                return 1;
            } else if (chuds::is_fault(r.error())) {
                // overflow and bus-off recover, so report and keep reading
                std::fprintf(stderr, "recv fault: %d\n", std::to_underlying(r.error()));
                break;
            }
        }
    }
    return 0;
}

int run_body(Bus& bus, int argc, char** argv, int first) {
    using chuds::BodyState;
    std::uint8_t bits{};
    for (int i = first; i < argc; ++i) {
        const auto* const it =
            std::ranges::find(kOutputs, std::string_view{argv[i]}, &NamedValue::name);
        if (it == kOutputs.end()) {
            std::fprintf(stderr, "unknown output: %s\n", argv[i]);
            return 2;
        }
        bits |= it->value;
    }

    std::printf("holding body state 0x%02x, ctrl-c to release\n", bits);
    using Clock = std::chrono::steady_clock;
    bool blink_on{};
    auto next_send  = Clock::now();
    auto next_blink = Clock::now() + kBlinkHalf;
    while (g_stop == 0) {
        const auto now = Clock::now();
        if (now >= next_blink) {
            blink_on   = !blink_on;
            next_blink = now + kBlinkHalf;
            next_send  = now;
        }
        if (now >= next_send) {
            const auto out =
                static_cast<std::uint8_t>(blink_on ? bits | BodyState::kBlinkPhase : bits);
            const auto st = bus.send(BodyState{out});
            if (!st) {
                std::fprintf(stderr, "send failed: %d\n", std::to_underlying(st.error()));
            }
            next_send = now + kBodyResend;
        }
        // drain received frames so the socket buffer does not fill
        if (bus.wait(std::chrono::milliseconds{20})) {
            while (bus.recv()) {
            }
        }
    }
    std::printf("released, boards fall back to safe states within 1 s\n");
    return 0;
}

int run_blink(Bus& bus, std::string_view node_name, std::string_view arg) {
    std::span<const NamedValue> nodes = kNodes;
    if (node_name != "all") {
        const auto* const it = std::ranges::find(kNodes, node_name, &NamedValue::name);
        if (it == kNodes.end()) {
            std::fprintf(stderr,
                         "blink node must be front_aux, back_aux, joulemeter, devboard, or all\n");
            return 2;
        }
        nodes = {it, 1};
    }
    std::uint16_t ms{};
    if (!parse(arg, ms) || ms == 0) {
        std::fprintf(stderr, "blink needs a period in ms, 1-65535\n");
        return 2;
    }
    bool ok = true;
    for (const NamedValue& node : nodes) {
        const auto st = bus.send(chuds::StatusBlink{.node = node.value, .half_period_ms = ms});
        report("status blink", st);
        ok = ok && st;
    }
    return ok ? 0 : 1;
}

int run_stop(Bus& bus, std::string_view arg) {
    std::uint8_t severity{};
    if (!arg.empty() && !parse(arg, severity)) {
        std::fprintf(stderr, "stop severity must be 0-255\n");
        return 2;
    }
    const auto stop = chuds::make_stop(severity, kSender, chuds::StopReason::Operator);
    auto st         = bus.send(stop);
    // a full tx queue drains within milliseconds, so retry for up to a second
    for (int tries = 0; !st && st.error() == chuds::TxError::QueueFull && tries < 100; ++tries) {
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
        st = bus.send(stop);
    }
    report("stop", st);
    return st ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    std::string_view ifname = "can0";
    int i                   = 1;
    if (argc > 2 && std::string_view{argv[1]} == "-i") {
        ifname = argv[2];
        i      = 3;
    }
    if (i >= argc) {
        usage(argv[0]);
        return 2;
    }
    const std::string_view cmd = argv[i];
    const std::string_view arg = i + 1 < argc ? argv[i + 1] : "";
    if (cmd != "dump" && cmd != "body" && cmd != "blink" && cmd != "stop") {
        usage(argv[0]);
        return 2;
    }

    Bus bus;
    if (!bus.open(ifname)) {
        std::fprintf(stderr, "failed to open %.*s\n", static_cast<int>(ifname.size()),
                     ifname.data());
        return 1;
    }
    std::signal(SIGINT, on_sigint);

    if (cmd == "dump") {
        return run_dump(bus);
    }
    if (cmd == "body") {
        return run_body(bus, argc, argv, i + 1);
    }
    if (cmd == "blink") {
        return run_blink(bus, arg, i + 2 < argc ? argv[i + 2] : "");
    }
    return run_stop(bus, arg);
}
