#pragma once

#include <cstdint>

// steering sensor: 5V ratiometric (Piher PST-360 or similar), 2.2k/3.3k divider (~0.6×), ADC2
constexpr unsigned kSteeringGpio  = 28;
constexpr unsigned kSteeringAdcCh = 2;  // gpio - 26

// brake sensor: same divider circuit, ADC3
constexpr unsigned kBrakeGpio  = 29;
constexpr unsigned kBrakeAdcCh = 3;  // gpio - 26

// divider ratio Vout/Vin = 3.3k / (2.2k + 3.3k)
constexpr float kDividerRatio = 3.3f / (2.2f + 3.3f);

// RPM pulse inputs: 12V through 10k/3.3k divider, PWM slices 1-4 channel B
constexpr unsigned kRpmGpio[4] = {19, 21, 23, 25};

// print period for the serial bring-up loop
constexpr std::uint32_t kPrintPeriodMs = 100;
