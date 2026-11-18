#include <sys/poll.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <string_view>
#include <utility>

#include "chuds/chuds.hpp"
#include "chuds/socketcan_transport.hpp"

namespace {

// how often the body state is resent, well inside the boards' 1 s timeout
constexpr auto kBodyResend = std::chrono::milliseconds{300};
// half a turn-signal blink period
constexpr auto kBlinkHalf = std::chrono::milliseconds{333};

volatile std::sig_atomic_t g_stop = 0;

void on_sigint(int /*sig*/) { g_stop = 1; }

// the key pressed since the last call, or 0
char read_key() {
    pollfd p{.fd = STDIN_FILENO, .events = POLLIN, .revents = 0};
    char c = 0;
    if (::poll(&p, 1, 0) > 0 && ::read(STDIN_FILENO, &c, 1) == 1) {
        return c;
    }
    return 0;
}

void print_state(const chuds::BodyState& s) {
    std::printf("[body] headlights %s  left %s  right %s  horn %s  wiper %s\n",
                s.headlights() ? "on" : "off", s.left_turn() ? "on" : "off",
                s.right_turn() ? "on" : "off", s.horn() ? "on" : "off", s.wiper() ? "on" : "off");
}

// apply one key to the body state, false if the key does nothing
bool apply_key(char key, chuds::BodyState& s) {
    switch (key) {
        case 'h': s.set_headlights(!s.headlights()); return true;
        case 'l': s.set_left_turn(!s.left_turn()); return true;
        case 'r': s.set_right_turn(!s.right_turn()); return true;
        case 'z': {
            // hazards toggle both turn signals together
            const bool on = !(s.left_turn() && s.right_turn());
            s.set_left_turn(on);
            s.set_right_turn(on);
            return true;
        }
        case 'n': s.set_horn(!s.horn()); return true;
        case 'w': s.set_wiper(!s.wiper()); return true;
        default: return false;
    }
}

// print telemetry and drive the body outputs from the keyboard until q or ctrl-c
int run_live(chuds::Bus<chuds::SocketCanTransport>& bus) {
    using chuds::BodyState;
    using Clock = std::chrono::steady_clock;

    // read keys one at a time without echo, and put the terminal back on exit
    termios saved{};
    const bool tty = ::tcgetattr(STDIN_FILENO, &saved) == 0;
    if (tty) {
        termios raw = saved;
        raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
        ::tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }
    std::signal(SIGINT, on_sigint);

    std::printf("keys: h=headlights, l=left, r=right, z=hazards, n=horn, w=wiper, q=quit\n");

    BodyState body{};
    bool blink_on{};
    auto next_send  = Clock::now();
    auto next_blink = Clock::now() + kBlinkHalf;

    while (g_stop == 0) {
        const char key = read_key();
        if (key == 'q') {
            break;
        }
        const auto now = Clock::now();
        if (apply_key(key, body)) {
            print_state(body);
            next_send = now;
        }
        if (now >= next_blink) {
            blink_on = !blink_on;
            next_blink += kBlinkHalf;
            next_send = now;
        }
        if (now >= next_send) {
            body.set_blink_phase(blink_on);
            (void)bus.send(body);
            next_send = now + kBodyResend;
        }

        if (!bus.wait(std::chrono::milliseconds{20})) {
            continue;
        }
        for (auto r = bus.recv(); r || r.error() != chuds::RxError::Empty; r = bus.recv()) {
            if (r) {
                chuds::print_message(*r);
            } else if (chuds::is_fault(r.error())) {
                std::fprintf(stderr, "recv fault: %d\n", std::to_underlying(r.error()));
                break;
            }
        }
    }

    if (tty) {
        ::tcsetattr(STDIN_FILENO, TCSANOW, &saved);
    }
    return 0;
}

// send a body state with headlights on
int run_send(chuds::Bus<chuds::SocketCanTransport>& bus) {
    chuds::BodyState state{};
    state.set_headlights(true);
    const auto st = bus.send(state);
    if (!st) {
        std::fprintf(stderr, "send failed: %d\n", std::to_underlying(st.error()));
        return 1;
    }
    std::printf("sent body state 0x%02x\n", state.raw());
    return 0;
}

// print the first message that arrives
int run_recv(chuds::Bus<chuds::SocketCanTransport>& bus, std::string_view ifname) {
    std::printf("listening on %.*s\n", static_cast<int>(ifname.size()), ifname.data());
    while (bus.wait(std::chrono::seconds{5})) {
        const auto r = bus.recv();
        if (r) {
            chuds::print_message(*r);
            return 0;
        }
        if (chuds::is_fault(r.error())) {
            std::fprintf(stderr, "recv failed: %d\n", std::to_underlying(r.error()));
            return 1;
        }
    }
    std::fprintf(stderr, "timeout, nothing received\n");
    return 1;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string_view mode   = argc > 1 ? argv[1] : "";
    const std::string_view ifname = argc > 2 ? argv[2] : "vcan0";

    if (mode != "send" && mode != "recv" && mode != "run") {
        std::fprintf(stderr, "usage: %s send|recv|run [ifname]\n", argv[0]);
        return 2;
    }

    chuds::Bus<chuds::SocketCanTransport> bus;
    if (!bus.open(ifname)) {
        std::fprintf(stderr, "failed to open %.*s\n", static_cast<int>(ifname.size()),
                     ifname.data());
        return 1;
    }

    if (mode == "run") {
        return run_live(bus);
    }

    return mode == "send" ? run_send(bus) : run_recv(bus, ifname);
}
