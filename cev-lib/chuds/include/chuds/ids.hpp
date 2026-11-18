#pragma once

#include <compare>
#include <cstdint>
#include <utility>

namespace chuds {

// top 3 bits of the id select the message class, low 8 bits are the subaddress
// classes are dense and ordered by priority, so the lower id wins arbitration
enum class MsgClass : std::uint8_t {
    Emergency = 0,
    Command   = 1,
    Telemetry = 2,
};

// an 11-bit standard CAN identifier, 0x000-0x7FF
// exposes only its class, subaddress, and arbitration order
class CanId {
    // class is bits 8-10, subaddress is bits 0-7
    static constexpr int kClassShift               = 8;
    static constexpr std::uint16_t kClassMask      = 0x0700;
    static constexpr std::uint16_t kSubaddressMask = 0x00FF;
    static constexpr std::uint16_t kStandardMax    = 0x07FF;
    static_assert((kClassMask | kSubaddressMask) == kStandardMax);

   public:
    CanId() = default;

    // pack a class and subaddress into an id
    constexpr CanId(MsgClass cls, std::uint8_t subaddress) {
        const int class_bits = std::to_underlying(cls) << kClassShift;

        raw_ = static_cast<std::uint16_t>(class_bits | subaddress);
    }

    // wrap a raw 11-bit value taken off the wire, for transports only
    [[nodiscard]] static constexpr CanId from_raw(std::uint16_t bits) { return CanId(bits); }

    [[nodiscard]] constexpr MsgClass cls() const {
        // apply the mask first, then shift down to the low bits
        // the bits may name a reserved class, which callers reject with is_valid
        return static_cast<MsgClass>((raw_ & kClassMask) >> kClassShift);
    }

    [[nodiscard]] constexpr std::uint8_t subaddress() const {
        return static_cast<std::uint8_t>(raw_ & kSubaddressMask);
    }

    // the raw bits, for transports only
    [[nodiscard]] constexpr std::uint16_t raw() const { return raw_; }

    // within the 11-bit standard range
    [[nodiscard]] constexpr bool is_standard() const { return raw_ <= kStandardMax; }

    // numeric order is arbitration priority, the lower id wins
    [[nodiscard]] constexpr std::strong_ordering operator<=>(const CanId&) const = default;

   private:
    // privated to prevent direct usage, because it is not
    // immediately clear this ctor builds directly from bits
    constexpr explicit CanId(std::uint16_t bits) : raw_(bits) {}

    std::uint16_t raw_{};
};

// largest valid 11-bit standard identifier
inline constexpr CanId kMaxStandardId = CanId::from_raw(0x7FF);

// the last id of a class range
[[nodiscard]] constexpr CanId class_max(MsgClass cls) { return {cls, 0xFF}; }

// a class value is defined only over the dense range Emergency..Telemetry
[[nodiscard]] constexpr bool is_valid(MsgClass cls) { return cls <= MsgClass::Telemetry; }

}  // namespace chuds
