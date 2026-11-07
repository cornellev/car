#include <cstdint>

#include "chuds/ids.hpp"
#include "chuds/message.hpp"

// built once per case by tests/CMakeLists.txt
// test_message.cpp covers the WireBody rules, this covers that both entry points enforce them
namespace {
struct Unmarked {
    std::uint16_t a{};
};

struct Marked {
    static constexpr chuds::CanId kId{chuds::Band::Telemetry, 0x10};
    std::uint16_t a{};
};

#ifdef BAD_MAKE_MESSAGE
using SendBody = Unmarked;
#else
using SendBody = Marked;
#endif

#ifdef BAD_BODY_AS
using ReceiveBody = Unmarked;
#else
using ReceiveBody = Marked;
#endif

[[maybe_unused]] void send_and_receive() {
    static_cast<void>(chuds::make_message(SendBody{}));
    static_cast<void>(chuds::body_as<ReceiveBody>(chuds::Message{}));
}
}  // namespace
