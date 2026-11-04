#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "chuds/ids.hpp"

namespace chuds {

// the maximum CAN-FD data-field size in bytes
inline constexpr std::size_t kMaxFdPayload = 64;

// every node must run the bus at these bitrates and sample points
// sample points are percent of the bit time
inline constexpr std::uint32_t kNominalBitrate     = 500'000;
inline constexpr std::uint32_t kDataBitrate        = 2'000'000;
inline constexpr std::uint32_t kNominalSamplePoint = 80;
inline constexpr std::uint32_t kDataSamplePoint    = 80;

// round a byte count (0..64) up to the next valid CAN-FD data length
[[nodiscard]] constexpr std::uint8_t round_up_fd_length(std::uint8_t n) {
    if (n <= 8) return n;
    if (n <= 12) return 12;
    if (n <= 16) return 16;
    if (n <= 20) return 20;
    if (n <= 24) return 24;
    if (n <= 32) return 32;
    if (n <= 48) return 48;
    return 64;
}

// whether len is exactly a valid CAN-FD data length, needing no rounding
[[nodiscard]] constexpr bool is_valid_fd_len(std::uint8_t len) {
    return len <= kMaxFdPayload && round_up_fd_length(len) == len;
}
static_assert(is_valid_fd_len(kMaxFdPayload));

// a raw CAN-FD frame with 11-bit id and up to 64 bytes of data
// len is the actual data length, a valid CAN-FD size (see is_valid_fd_len)
// purposely kept not specific to chuds, and only contains the raw CAN-FD concepts
struct CanFrame {
    CanId id{};
    std::uint8_t len{};
    std::array<std::uint8_t, kMaxFdPayload> data{};
};

}  // namespace chuds
