#pragma once

// Every pin, threshold and timing for the precharge / isolation board.
// No Pico SDK includes here: state_machine.cpp uses this file and is also built on a
// laptop for test_state_machine.cpp.
// Values marked TUNE are estimates to check on the real hardware.

#include <cstdint>

// true: after power-up (or `just flash`), wait for 'g' on USB serial before precharging.
// false: precharge starts at power-up. Keep true until the system is proven on the car.
constexpr bool kDevMode = true;

// ---- pins ----
constexpr unsigned kMainRelayPin = 2;   // PWM -> 1k -> TIP102 base -> G9KA coil
constexpr unsigned kSsrPin       = 3;   // -> SSR driver (ZVN3306A gate + 100k, or 1k -> TIP102)
constexpr unsigned kEstopPin     = 4;   // high = OK, low = pressed or wire broken
constexpr unsigned kStatusLed    = 25;  // Pico onboard LED
constexpr unsigned kBusAdcPin    = 26;  // ADC0
constexpr unsigned kBattAdcPin   = 27;  // ADC1
constexpr unsigned kBusAdcInput  = 0;
constexpr unsigned kBattAdcInput = 1;

// ---- voltage sensing ----
// (R_top + R_bottom) / R_bottom, nominally 1M / 51k. TUNE: measure each finished divider.
constexpr float kBattDivRatio = 20.608f;
constexpr float kBusDivRatio  = 20.608f;
constexpr float kAdcVref      = 3.3f;
constexpr int   kAdcSamples   = 16;       // averaged per channel per tick
constexpr float kAdcSatCounts = 4050.0f;  // average at/above this = sensor fault (max 4095)

// ---- battery ----
constexpr float         kBattMinV        = 40.0f;  // lower temporarily for low-voltage bench tests
constexpr float         kBattMaxV        = 63.0f;
constexpr std::uint32_t kBattUvPersistMs = 1000;   // ride through sag under load
constexpr std::uint32_t kBattOvPersistMs = 200;
// The bus can't be meaningfully above the battery. TUNE: above the regen I*R drop between taps.
constexpr float         kBusOverBattMaxV = 2.0f;
constexpr std::uint32_t kSensorPersistMs = 20;

// ---- startup ----
constexpr std::uint32_t kInitSettleMs        = 500;
constexpr std::uint32_t kLeakRefMs           = 100;  // bus reference for the SSR-stuck check
constexpr float         kSsrStuckRiseV       = 1.0f; // bus rising this much with SSR off = stuck on
constexpr std::uint32_t kEstopReleaseStableMs = 1000; // boot only

// ---- precharge: 500 ohm x ~3 mF ----
constexpr float         kPrechargeOhm   = 500.0f;
constexpr float         kTauS           = 1.5f;   // TUNE: R x C, check the logged precharge time
constexpr float         kDoneFrac       = 0.95f;
constexpr std::uint32_t kStallWindowMs  = 500;
constexpr float         kStallFrac      = 0.30f;  // rise < 30% of the RC-expected rise = stalled
constexpr float         kStallMinGapV   = 5.0f;   // skip the stall check this close to done
constexpr float         kNoRiseFrac     = 0.25f;  // stalled below 25% = short or open path
constexpr std::uint32_t kPrechargeTimeoutMs = 12000;
constexpr float         kRiseMaxFactor  = 4.0f;   // bus faster than RC allows (C down to ~1/4) = sensor fault
constexpr float         kRiseSlackV     = 0.5f;   // ADC noise allowance per tick

// ---- main relay: G9KA-1A DC24 via TIP102 ----
constexpr std::uint32_t kMainPwmHz       = 20000;  // drop to 5000 if the TIP102 runs warm
constexpr std::uint32_t kMainHoldDutyPct = 54;     // ~12.5 V average (coil sees ~23 V when on)
constexpr std::uint32_t kPullInMs        = 250;    // datasheet: full voltage 0.1-3 s
constexpr std::uint32_t kVerifyAfterMs   = 100;    // datasheet operate time 30 ms max
constexpr float         kVerifyMaxV      = 1.5f;

// ---- run ----
// TUNE: must exceed the worst I*R drop between battery and bus taps at full load.
constexpr float         kDropoutV         = 3.0f;
constexpr std::uint32_t kDropoutPersistMs = 50;

// ---- loop ----
constexpr std::uint32_t kTickUs         = 1000;
constexpr std::uint32_t kWatchdogMs     = 100;
constexpr std::uint32_t kReportPeriodMs = 100;
