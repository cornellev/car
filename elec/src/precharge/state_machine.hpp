#pragma once

// All decisions about the relays. Pure logic, no hardware or SDK: each tick it gets a
// snapshot of the inputs and returns what the relays should do.

#include <cstdint>

namespace precharge {

enum class State : std::uint8_t {
    Init,
    WaitEstopClear,  // boot only: e-stop was pressed at power-up
    WaitStart,       // kDevMode only: waiting for 'g'
    Precharge,
    CloseMain,
    Run,
    Estop,  // latched until power cycle
    Fault,  // latched until power cycle
};

// The number is the LED blink count.
enum class Fault : std::uint8_t {
    None,
    WatchdogReset,  // 1  firmware hung or crashed last boot
    Sensor,         // 2  bus above battery, or bus jumped faster than possible
    BattUv,         // 3
    BattOv,         // 4
    SsrStuckOn,     // 5  bus rose at boot with the SSR off
    NoRise,         // 6  bus not charging: shorted bus or open precharge path
    Stall,          // 7  bus stopped rising: something draws current
    Timeout,        // 8
    MainMismatch,   // 9  main relay did not close
    Dropout,        // 10 main relay opened while running
};

enum class Main : std::uint8_t { Off, PullIn, Hold };

struct Inputs {
    std::uint64_t now_ms;
    float batt_v;
    float bus_v;
    bool estop_low;      // pin level
    bool estop_latched;  // relay layer: e-stop ISR fired while armed
    bool armed;          // relay layer: outputs may energize
    bool start_cmd;      // 'g' received this tick
};

struct Outputs {
    Main main        = Main::Off;
    bool ssr         = false;
    bool arm_request = false;  // ask the relay layer to arm (it re-checks the e-stop pin)
};

// True once `cond` has held continuously for `ms`.
struct Persist {
    bool active         = false;
    std::uint64_t since = 0;
    bool update(bool cond, std::uint64_t now, std::uint32_t ms);
};

struct Controller {
    Controller(bool watchdog_reboot, bool dev_mode, std::uint64_t now_ms);
    Outputs step(const Inputs& in);

    State state                = State::Init;
    State prev_state           = State::Init;
    bool changed               = false;  // a transition happened on the last step
    Fault fault                = Fault::None;
    float fault_value          = 0.0f;  // the reading that tripped it
    std::uint32_t precharge_ms = 0;

   private:
    void go(State s, std::uint64_t now);
    Outputs trip(Fault f, float value, std::uint64_t now);
    Outputs begin(const Inputs& in);

    bool watchdog_reboot_;
    bool dev_mode_;
    bool start_seen_ = false;
    std::uint64_t entry_ms_;
    std::uint64_t last_ms_;
    float leak_ref_bus_   = 0.0f;
    float prev_bus_       = 0.0f;
    std::uint64_t win_ms_ = 0;
    float win_bus_        = 0.0f;
    Persist released_, sensor_, uv_, ov_, dropout_;
};

const char* name(State s);
const char* name(Fault f);

}  // namespace precharge
