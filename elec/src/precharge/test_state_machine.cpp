// Host tests for the state machine (not part of `just build`). Simulates the battery,
// bus capacitance, precharge resistor, relays, and the relay layer's arm/lock rules.
// From this folder:
//   g++ -std=c++20 -Wall -Wextra -O1 test_state_machine.cpp state_machine.cpp -o /tmp/test_sm && /tmp/test_sm

#include <cmath>
#include <cstdio>
#include <cstring>

#include "config.hpp"
#include "state_machine.hpp"

using namespace precharge;
#define NAN_F std::nanf("")

typedef struct {
    // plant
    float batt_true;
    float bus_true;
    float cap_f;
    float load_a;           // current the controller draws from the bus
    bool  main_works;       // contacts actually close when energized
    bool  ssr_stuck_on;
    float wire_ohm;         // drop between battery tap and bus tap when main closed
    float short_ohm;        // 0 = no short on the bus
    // relay state
    bool  main_closed;
    int   main_close_timer;
    bool  dropout;          // force contacts open while running
    // relay layer (mirrors relays.c rules)
    bool  armed, latched;
    bool  ssr_driven;
    Main main_driven;
    // sensors
    bool  estop_low;
    float batt_sensor_override;   // NAN = none
    float bus_sensor_override;
    bool  saturated;
    bool  start_cmd;
    // bookkeeping
    bool  invariant_failed;
    float bus_at_close;
    bool  closed_main_ever;
} sim_t;

static void sim_defaults(sim_t *s) {
    *s = sim_t{};
    s->batt_true = 58.0f;
    s->cap_f = 3e-3f;
    s->main_works = true;
    s->wire_ohm = 0.01f;
    s->batt_sensor_override = NAN_F;
    s->bus_sensor_override = NAN_F;
    s->bus_at_close = -1.0f;
}

static void plant_step(sim_t *s, float dt) {
    const bool coil = s->main_driven != Main::Off;
    if (coil && s->main_works && !s->dropout) {
        if (++s->main_close_timer >= 20) s->main_closed = true;      // 20 ms operate
    } else {
        s->main_close_timer = 0;
        s->main_closed = false;
    }
    if (s->main_closed) {
        s->bus_true = s->batt_true - s->load_a * s->wire_ohm;
        return;
    }
    const bool ssr = s->ssr_driven || s->ssr_stuck_on;
    float i_in = ssr ? (s->batt_true - s->bus_true) / kPrechargeOhm : 0.0f;
    float load = s->bus_true > 15.0f ? s->load_a : 0.0f;   // controller wakes ~15 V
    if (s->short_ohm > 0) load += s->bus_true / s->short_ohm;
    s->bus_true += (i_in - load) / s->cap_f * dt;
    s->bus_true -= s->bus_true / (1.05e6f * s->cap_f) * dt; // divider bleed
    if (s->bus_true < 0) s->bus_true = 0;
}

using sm_t = Controller;
// Runs for `ms`; `hook` may change the sim at given times.
typedef void (*hook_fn)(sim_t *, sm_t *, uint64_t t);

static void run(sim_t *s, sm_t *sm, uint64_t ms, hook_fn hook) {
    for (uint64_t t = 0; t < ms; t++) {
        if (hook) hook(s, sm, t);
        // e-stop ISR model: falling edge while armed latches
        static bool prev_low = false;
        if (t == 0) prev_low = s->estop_low;
        if (s->estop_low && !prev_low) {
            if (s->armed) s->latched = true;
            s->ssr_driven = false;
            s->main_driven = Main::Off;
        }
        prev_low = s->estop_low;

        Inputs in = {
            .now_ms = t + 5,   // boot time offset
            .batt_v = std::isnan(s->batt_sensor_override) ? s->batt_true : s->batt_sensor_override,
            .bus_v = std::isnan(s->bus_sensor_override) ? s->bus_true : s->bus_sensor_override,
            .saturated = s->saturated,
            .estop_low = s->estop_low,
            .estop_latched = s->latched,
            .armed = s->armed,
            .start_cmd = s->start_cmd,
        };
        s->start_cmd = false;
        Outputs out = sm->step(in);

        // invariants
        if (out.ssr && sm->state != State::Precharge && sm->state != State::CloseMain)
            s->invariant_failed = true;
        if (out.main != Main::Off && sm->state != State::CloseMain && sm->state != State::Run)
            s->invariant_failed = true;
        if ((sm->state == State::Fault || sm->state == State::Estop) && (out.ssr || out.main != Main::Off))
            s->invariant_failed = true;

        // relay layer model (mirrors main.cpp: lock on FAULT/ESTOP, never cleared)
        if (sm->state == State::Fault || sm->state == State::Estop) {
            s->latched = true;
            s->ssr_driven = false;
            s->main_driven = Main::Off;
        } else {
            if (out.arm_request && !s->estop_low && !s->latched) s->armed = true;
            const bool allow = s->armed && !s->latched;
            s->ssr_driven = out.ssr && allow;
            s->main_driven = allow ? out.main : Main::Off;
        }
        if (s->main_driven != Main::Off && !s->closed_main_ever) {
            s->closed_main_ever = true;
            s->bus_at_close = s->bus_true / s->batt_true;
        }
        plant_step(s, 0.001f);
    }
}

static int g_fail = 0, g_pass = 0;

static void check(const char *label, sim_t *s, sm_t *sm, State want_state, Fault want_fault) {
    bool ok = sm->state == want_state && sm->fault == want_fault && !s->invariant_failed;
    if (s->closed_main_ever && s->bus_at_close < 0.94f) ok = false;   // never close uncharged
    printf("%-58s %s  (got %s/%s", label, ok ? "PASS" : "FAIL", name(sm->state), name(sm->fault));
    if (s->closed_main_ever) printf(", bus at close %.1f%%", s->bus_at_close * 100.0f);
    if (sm->precharge_ms) printf(", precharge %u ms", (unsigned)sm->precharge_ms);
    if (sm->state == State::Fault) printf(", value %.2f", sm->fault_value);
    printf(")\n");
    if (ok) g_pass++; else g_fail++;
}

// ---- hooks ----
static void h_start_at_2s(sim_t *s, sm_t *sm, uint64_t t) { (void)sm; if (t == 2000) s->start_cmd = true; }
static void h_release_3s(sim_t *s, sm_t *sm, uint64_t t) { (void)sm; if (t == 3000) s->estop_low = false; }
static void h_blip(sim_t *s, sm_t *sm, uint64_t t) {
    (void)sm;
    if (t == 3000) s->estop_low = false;
    if (t == 3300) s->estop_low = true;
}
static void h_estop_in_state(sim_t *s, sm_t *sm, uint64_t t) {
    static int armed_at = -1;
    if (t == 0) armed_at = -1;
    if (sm->state == State::Precharge && armed_at < 0) armed_at = (int)t;
    if (armed_at >= 0 && (int)t == armed_at + 1000) s->estop_low = true;
    if (armed_at >= 0 && (int)t == armed_at + 3000) s->estop_low = false;   // release: must stay ESTOP
}
static void h_estop_in_run(sim_t *s, sm_t *sm, uint64_t t) {
    (void)t;
    static uint64_t run_at = 0;
    if (sm->state == State::Run && run_at == 0) run_at = t;
    if (run_at && t == run_at + 500) s->estop_low = true;
    if (run_at && t == run_at + 2000) s->estop_low = false;
}
static void h_dropout(sim_t *s, sm_t *sm, uint64_t t) {
    static uint64_t run_at = 0;
    if (t == 0) run_at = 0;
    if (sm->state == State::Run && run_at == 0) run_at = t;
    if (run_at && t == run_at + 1000) s->dropout = true;
}
static void h_bus_sensor_breaks(sim_t *s, sm_t *sm, uint64_t t) {
    (void)t;
    if (sm->state == State::Precharge && s->bus_true > 10.0f) {
        s->bus_sensor_override = 69.0f;
        s->saturated = true;
    }
}
static void h_bus_sensor_jumps_plausible(sim_t *s, sm_t *sm, uint64_t t) {
    (void)t;   // fails to a plausible-but-wrong value (not saturated, below batt + margin)
    if (sm->state == State::Precharge && s->bus_true > 10.0f) s->bus_sensor_override = 59.0f;
}
static void h_batt_sensor_zero(sim_t *s, sm_t *sm, uint64_t t) {
    (void)t;
    if (sm->state == State::Precharge && s->bus_true > 10.0f) s->batt_sensor_override = 0.0f;
}
static void h_sag_short(sim_t *s, sm_t *sm, uint64_t t) {
    static uint64_t run_at = 0;
    if (t == 0) run_at = 0;
    if (sm->state == State::Run && run_at == 0) run_at = t;
    if (run_at && t == run_at + 500) s->batt_true = 37.0f;
    if (run_at && t == run_at + 800) s->batt_true = 45.0f;
}
static void h_sag_long(sim_t *s, sm_t *sm, uint64_t t) {
    static uint64_t run_at = 0;
    if (t == 0) run_at = 0;
    if (sm->state == State::Run && run_at == 0) run_at = t;
    if (run_at && t == run_at + 500) s->batt_true = 37.0f;
}

#define SM(wd, dev) sm = Controller{wd, dev, 5}

int main() {
    sim_t s;
    Controller sm{false, false, 5};

    sim_defaults(&s); SM(false, false);
    run(&s, &sm, 10000, NULL);
    check("1. normal start (auto) -> RUN", &s, &sm, State::Run, Fault::None);
    bool run_outputs_ok = !s.ssr_driven && s.main_driven == Main::Hold;
    printf("   RUN outputs: SSR off, main HOLD: %s\n", run_outputs_ok ? "PASS" : "FAIL");
    run_outputs_ok ? g_pass++ : g_fail++;

    sim_defaults(&s); SM(false, true);
    run(&s, &sm, 1900, NULL);
    check("2a. DEV_MODE waits for 'g'", &s, &sm, State::WaitStart, Fault::None);
    sim_defaults(&s); SM(false, true);
    run(&s, &sm, 10000, h_start_at_2s);
    check("2b. DEV_MODE 'g' at 2 s -> RUN", &s, &sm, State::Run, Fault::None);

    sim_defaults(&s); s.estop_low = true; SM(false, false);
    run(&s, &sm, 3500, h_release_3s);
    check("3a. e-stop held at boot, released 3 s: still waiting at 3.5 s", &s, &sm,
          State::WaitEstopClear, Fault::None);
    sim_defaults(&s); s.estop_low = true; SM(false, false);
    run(&s, &sm, 12000, h_release_3s);
    check("3b. ... then precharges after 1 s stable -> RUN", &s, &sm, State::Run, Fault::None);

    sim_defaults(&s); s.estop_low = true; SM(false, false);
    run(&s, &sm, 8000, h_blip);
    check("4. e-stop released only 300 ms at boot -> no start", &s, &sm, State::WaitEstopClear,
          Fault::None);

    sim_defaults(&s); SM(false, false);
    run(&s, &sm, 8000, h_estop_in_state);
    check("5. e-stop during precharge, then released -> stays ESTOP", &s, &sm, State::Estop,
          Fault::None);

    sim_defaults(&s); SM(false, false);
    run(&s, &sm, 12000, h_estop_in_run);
    check("6. e-stop during RUN, then released -> stays ESTOP", &s, &sm, State::Estop, Fault::None);

    sim_defaults(&s); s.short_ohm = 1.0f; SM(false, false);
    run(&s, &sm, 10000, NULL);
    check("7. bus shorted -> PRECHARGE_NO_RISE", &s, &sm, State::Fault, Fault::NoRise);

    sim_defaults(&s); s.load_a = 0.05f; SM(false, false);
    run(&s, &sm, 15000, NULL);
    check("8. controller draws 50 mA during precharge -> STALL", &s, &sm, State::Fault,
          Fault::Stall);

    sim_defaults(&s); s.load_a = 0.003f; SM(false, false);
    run(&s, &sm, 15000, NULL);
    check("8b. controller draws 3 mA -> still reaches RUN", &s, &sm, State::Run, Fault::None);

    sim_defaults(&s); s.cap_f = 6e-3f; SM(false, false);
    run(&s, &sm, 15000, NULL);
    check("9. bus capacitance 2x the estimate -> still RUN", &s, &sm, State::Run, Fault::None);

    sim_defaults(&s); s.cap_f = 1e-3f; SM(false, false);
    run(&s, &sm, 15000, NULL);
    check("9b. bus capacitance 1/3 of the estimate -> still RUN", &s, &sm, State::Run, Fault::None);

    sim_defaults(&s); s.main_works = false; SM(false, false);
    run(&s, &sm, 10000, NULL);
    check("10. main relay never closes -> MAIN_MISMATCH", &s, &sm, State::Fault,
          Fault::MainMismatch);

    sim_defaults(&s); s.load_a = 2.0f; SM(false, false);
    s.load_a = 0.0f;   // no load during precharge
    run(&s, &sm, 8000, NULL);
    s.load_a = 2.0f;
    run(&s, &sm, 4000, h_dropout);
    check("11. contactor drops out under load in RUN -> DROPOUT", &s, &sm, State::Fault,
          Fault::Dropout);

    sim_defaults(&s); s.batt_true = 38.0f; SM(false, false);
    run(&s, &sm, 3000, NULL);
    check("12. battery 38 V at boot -> BATT_UV", &s, &sm, State::Fault, Fault::BattUv);

    sim_defaults(&s); s.batt_true = 66.0f; SM(false, false);
    run(&s, &sm, 3000, NULL);
    check("12b. battery 66 V at boot -> BATT_OV", &s, &sm, State::Fault, Fault::BattOv);

    sim_defaults(&s); SM(false, false);
    run(&s, &sm, 10000, h_bus_sensor_breaks);
    check("13. bus divider breaks (reads 69 V, saturated) -> SENSOR", &s, &sm, State::Fault,
          Fault::Sensor);

    sim_defaults(&s); SM(false, false);
    run(&s, &sm, 10000, h_batt_sensor_zero);
    check("14. battery sensor drops to 0 V mid-precharge -> no close", &s, &sm, State::Fault,
          Fault::Sensor);

    sim_defaults(&s); SM(true, false);
    run(&s, &sm, 3000, NULL);
    check("15. watchdog reboot -> FAULT WATCHDOG_RESET", &s, &sm, State::Fault,
          Fault::WatchdogReset);

    sim_defaults(&s); s.ssr_stuck_on = true; SM(false, false);
    run(&s, &sm, 3000, NULL);
    check("16. SSR stuck on at boot -> SSR_STUCK_ON", &s, &sm, State::Fault, Fault::SsrStuckOn);

    sim_defaults(&s); s.bus_true = 55.0f; SM(false, false);
    run(&s, &sm, 5000, NULL);
    check("17. bus still charged from last run -> quick precharge, RUN", &s, &sm, State::Run,
          Fault::None);

    sim_defaults(&s); SM(false, false);
    run(&s, &sm, 12000, h_sag_short);
    check("18a. battery sags to 37 V for 300 ms in RUN -> no fault", &s, &sm, State::Run,
          Fault::None);
    sim_defaults(&s); SM(false, false);
    run(&s, &sm, 12000, h_sag_long);
    check("18b. battery stays at 37 V in RUN -> BATT_UV", &s, &sm, State::Fault, Fault::BattUv);

    sim_defaults(&s); SM(false, false);
    run(&s, &sm, 10000, h_bus_sensor_jumps_plausible);
    check("19. bus sensor jumps to a plausible 59 V -> SENSOR, no close", &s, &sm, State::Fault,
          Fault::Sensor);

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
