#pragma once

namespace cev {

// maximum receive attempts per loop pass, so bus traffic cannot starve periodic work
inline constexpr int kMaxRxPerPass = 8;

}  // namespace cev
