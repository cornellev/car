#include "state_machine.hpp"

#include <cmath>

#include "config.hpp"

namespace precharge {

bool Persist::update(bool cond, std::uint64_t now, std::uint32_t ms) {
    if (!cond) {
        active = false;
        return false;
    }
    if (!active) {
        active = true;
        since  = now;
    }
    return now - since >= ms;
}

namespace {
bool plausible(const Inputs& in) {
    return !in.saturated && in.bus_v <= in.batt_v + kBusOverBattMaxV;
}
}  // namespace

Controller::Controller(bool watchdog_reboot, bool dev_mode, std::uint64_t now_ms)
    : watchdog_reboot_{watchdog_reboot}, dev_mode_{dev_mode}, entry_ms_{now_ms}, last_ms_{now_ms} {}

void Controller::go(State s, std::uint64_t now) {
    prev_state = state;
    state      = s;
    changed    = true;
    entry_ms_  = now;
    released_ = sensor_ = uv_ = ov_ = dropout_ = Persist{};
}

Outputs Controller::trip(Fault f, float value, std::uint64_t now) {
    fault       = f;
    fault_value = value;
    go(State::Fault, now);
    return {};
}

// Checks battery and sensors, then waits for 'g' (dev mode) and for the relay layer to
// arm before entering PRECHARGE.
Outputs Controller::begin(const Inputs& in) {
    Outputs out;
    if (!plausible(in)) return trip(Fault::Sensor, in.bus_v, in.now_ms);
    if (in.batt_v < kBattMinV) return trip(Fault::BattUv, in.batt_v, in.now_ms);
    if (in.batt_v > kBattMaxV) return trip(Fault::BattOv, in.batt_v, in.now_ms);
    if (dev_mode_ && !start_seen_) {
        if (state != State::WaitStart) go(State::WaitStart, in.now_ms);
        return out;
    }
    if (!in.armed) {
        out.arm_request = true;
        return out;
    }
    go(State::Precharge, in.now_ms);
    prev_bus_ = win_bus_ = in.bus_v;
    win_ms_              = in.now_ms;
    return out;
}

Outputs Controller::step(const Inputs& in) {
    const std::uint64_t now = in.now_ms;
    const float dt_s        = static_cast<float>(now - last_ms_) / 1000.0f;
    last_ms_                = now;
    changed                 = false;

    if (state == State::Fault || state == State::Estop) return {};

    // E-stop beats everything. Before arming, a pressed e-stop is boot behavior instead.
    if (in.estop_latched || (in.armed && in.estop_low)) {
        go(State::Estop, now);
        return {};
    }

    if (state == State::Precharge || state == State::CloseMain || state == State::Run) {
        if (sensor_.update(!plausible(in), now, kSensorPersistMs))
            return trip(Fault::Sensor, in.bus_v, now);
        if (uv_.update(in.batt_v < kBattMinV, now, kBattUvPersistMs))
            return trip(Fault::BattUv, in.batt_v, now);
        if (ov_.update(in.batt_v > kBattMaxV, now, kBattOvPersistMs))
            return trip(Fault::BattOv, in.batt_v, now);
    }

    const std::uint64_t t = now - entry_ms_;
    Outputs out;

    switch (state) {
        case State::Init:
            if (watchdog_reboot_) return trip(Fault::WatchdogReset, 0.0f, now);
            if (t < kLeakRefMs) {
                leak_ref_bus_ = in.bus_v;
                break;
            }
            if (t < kInitSettleMs) break;
            // SSR has been off the whole time: a rising bus means it's shorted or reversed.
            if (in.bus_v - leak_ref_bus_ > kSsrStuckRiseV) {
                return trip(Fault::SsrStuckOn, in.bus_v - leak_ref_bus_, now);
            }
            if (in.estop_low) {
                go(State::WaitEstopClear, now);
                break;
            }
            return begin(in);

        case State::WaitEstopClear:
            // The only place a release starts anything (boot only).
            if (released_.update(!in.estop_low, now, kEstopReleaseStableMs)) return begin(in);
            break;

        case State::WaitStart:
            if (in.estop_low) {
                start_seen_ = false;
                go(State::WaitEstopClear, now);
                break;
            }
            if (in.start_cmd) start_seen_ = true;
            return begin(in);

        case State::Precharge: {
            out.ssr = true;

            // The bus charges through 500 ohm, so it can't jump. A faster rise means the bus
            // sensor failed to a wrong value: never close the main relay on that reading.
            const float max_rise =
                kRiseSlackV + kRiseMaxFactor * (in.batt_v - prev_bus_) / kTauS * dt_s;
            if (in.bus_v - prev_bus_ > max_rise) return trip(Fault::Sensor, in.bus_v, now);
            prev_bus_ = in.bus_v;

            // Battery must be valid too, or a 0 V battery reading would make "95% of 0" true.
            if (plausible(in) && in.batt_v >= kBattMinV && in.bus_v >= kDoneFrac * in.batt_v) {
                precharge_ms = static_cast<std::uint32_t>(t);
                go(State::CloseMain, now);
                out.main = Main::PullIn;  // SSR stays on until the main relay is confirmed
                break;
            }

            if (now - win_ms_ >= kStallWindowMs) {
                const float gap      = in.batt_v - win_bus_;
                const float w_s      = static_cast<float>(now - win_ms_) / 1000.0f;
                const float expected = gap * (1.0f - std::exp(-w_s / kTauS));
                if (gap > kStallMinGapV && in.bus_v - win_bus_ < kStallFrac * expected) {
                    const Fault f =
                        in.bus_v < kNoRiseFrac * in.batt_v ? Fault::NoRise : Fault::Stall;
                    return trip(f, in.bus_v, now);
                }
                win_ms_  = now;
                win_bus_ = in.bus_v;
            }

            if (t >= kPrechargeTimeoutMs) return trip(Fault::Timeout, in.bus_v, now);
            break;
        }

        case State::CloseMain:
            out.ssr  = true;
            out.main = Main::PullIn;
            if (t >= kVerifyAfterMs &&
                (std::fabs(in.batt_v - in.bus_v) > kVerifyMaxV || in.batt_v < kBattMinV)) {
                return trip(Fault::MainMismatch, in.batt_v - in.bus_v, now);
            }
            if (t >= kPullInMs) {
                go(State::Run, now);
                out.ssr  = false;  // main relay confirmed closed: precharge path off
                out.main = Main::Hold;
            }
            break;

        case State::Run:
            out.main = Main::Hold;
            if (dropout_.update(in.batt_v - in.bus_v > kDropoutV, now, kDropoutPersistMs)) {
                return trip(Fault::Dropout, in.batt_v - in.bus_v, now);
            }
            break;

        case State::Estop:
        case State::Fault: break;
    }

    // Belt and braces: an output is only ever on in the states that own it.
    if (state != State::Precharge && state != State::CloseMain) out.ssr = false;
    if (state != State::CloseMain && state != State::Run) out.main = Main::Off;
    return out;
}

const char* name(State s) {
    switch (s) {
        case State::Init: return "INIT";
        case State::WaitEstopClear: return "WAIT_ESTOP_CLEAR";
        case State::WaitStart: return "WAIT_START";
        case State::Precharge: return "PRECHARGE";
        case State::CloseMain: return "CLOSE_MAIN";
        case State::Run: return "RUN";
        case State::Estop: return "ESTOP";
        case State::Fault: return "FAULT";
    }
    return "?";
}

const char* name(Fault f) {
    switch (f) {
        case Fault::None: return "NONE";
        case Fault::WatchdogReset: return "WATCHDOG_RESET";
        case Fault::Sensor: return "SENSOR";
        case Fault::BattUv: return "BATT_UV";
        case Fault::BattOv: return "BATT_OV";
        case Fault::SsrStuckOn: return "SSR_STUCK_ON";
        case Fault::NoRise: return "NO_RISE";
        case Fault::Stall: return "STALL";
        case Fault::Timeout: return "TIMEOUT";
        case Fault::MainMismatch: return "MAIN_MISMATCH";
        case Fault::Dropout: return "DROPOUT";
    }
    return "?";
}

}  // namespace precharge
