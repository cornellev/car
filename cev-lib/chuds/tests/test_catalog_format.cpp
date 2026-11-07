#include <doctest/doctest.h>

#include <array>
#include <string_view>

#include "chuds/catalog.hpp"
#include "chuds/catalog_format.hpp"
#include "chuds/ids.hpp"
#include "chuds/message.hpp"

using namespace chuds;

TEST_CASE("format_message prints a catalog body with its values") {
    const auto m = make_message(BodyState{BodyState::kHeadlights});
    REQUIRE(m.has_value());
    std::array<char, 96> line{};
    const int n = format_message(*m, line);
    CHECK(std::string_view(line.data()) == "body state 0x04");
    CHECK(n == 15);
}

TEST_CASE("format_message prints joulemeter floats") {
    const joulemeter::Telemetry t{47.91f, 3.2f, 153.3f, 812.4f};
    const auto m = make_message(t);
    REQUIRE(m.has_value());
    std::array<char, 96> line{};
    format_message(*m, line);
    CHECK(std::string_view(line.data()) == "joulemeter 47.91 V 3.20 A 153.3 W 812.4 J");
}

TEST_CASE("format_message prints a status blink command with its target node") {
    const auto m = make_message(StatusBlink{.node = back_aux::kNodeId, .half_period_ms = 250});
    REQUIRE(m.has_value());
    std::array<char, 96> line{};
    format_message(*m, line);
    CHECK(std::string_view(line.data()) == "status blink 250 ms to 0x20");
}

TEST_CASE("format_message falls back to the header for a message outside the catalog") {
    const Message m{
        .cls        = Band::Command,
        .subaddress = 0x42,
        .body       = {0x01, 0x02},
        .body_len   = 2,
    };
    std::array<char, 96> line{};
    format_message(m, line);
    CHECK(std::string_view(line.data()) == "class 1 subaddress 0x42, 2 body bytes");
}

TEST_CASE("format_message truncates to the buffer and reports the full length") {
    const auto m = make_message(BodyState{});
    REQUIRE(m.has_value());
    std::array<char, 5> line{};
    const int n = format_message(*m, line);
    CHECK(std::string_view(line.data()) == "body");
    CHECK(n == 15);
}

TEST_CASE("format_message prints a stop with its severity, sender and reason") {
    const auto m = make_stop(2, 0x12, StopReason::NodeFault);
    std::array<char, 96> line{};
    format_message(m, line);
    CHECK(std::string_view(line.data()) == "stop severity 2 from 0x12 reason 3");
}

TEST_CASE("format_message prints a body state id with the wrong length as a plain header") {
    const Message m{.cls = Band::Command, .subaddress = 0x08, .body_len = 2};
    std::array<char, 96> line{};
    format_message(m, line);
    CHECK(std::string_view(line.data()) == "class 1 subaddress 0x08, 2 body bytes");
}

TEST_CASE("format_message prints both aux boards' telemetry") {
    const auto front = make_message(front_aux::Telemetry{12, 13, 2048});
    REQUIRE(front.has_value());
    std::array<char, 96> line{};
    format_message(*front, line);
    CHECK(std::string_view(line.data()) == "front_aux rpm 12/13 steering 2048");

    const auto back = make_message(back_aux::Telemetry{7, 8, 900});
    REQUIRE(back.has_value());
    format_message(*back, line);
    CHECK(std::string_view(line.data()) == "back_aux rpm 7/8 brake 900");
}

TEST_CASE("format_message does not print a telemetry frame on 0x08 as a body state") {
    const Message m{.cls = Band::Telemetry, .subaddress = 0x08, .body_len = 1};
    std::array<char, 96> line{};
    format_message(m, line);
    CHECK(std::string_view(line.data()) == "class 2 subaddress 0x08, 1 body bytes");
}
