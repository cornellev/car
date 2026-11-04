#include <doctest/doctest.h>

#include <array>
#include <string_view>

#include "chuds/catalog.hpp"
#include "chuds/catalog_format.hpp"
#include "chuds/ids.hpp"
#include "chuds/message.hpp"

using namespace chuds;

TEST_CASE("format_message prints a catalog body with its values") {
    BodyState body{};
    body.set_headlights(true);
    const auto m = make_message(MsgClass::Command, kBodyStateId, MsgType::Update, body);
    REQUIRE(m.has_value());
    std::array<char, 96> line{};
    const int n = format_message(*m, line);
    CHECK(std::string_view(line.data()) == "body state 0x04");
    CHECK(n == 15);
}

TEST_CASE("format_message prints joulemeter floats") {
    const joulemeter::Telemetry t{47.91f, 3.2f, 153.3f, 812.4f};
    const auto m = make_message(MsgClass::Telemetry, joulemeter::kNodeId, MsgType::Update, t);
    REQUIRE(m.has_value());
    std::array<char, 96> line{};
    format_message(*m, line);
    CHECK(std::string_view(line.data()) == "joulemeter 47.91 V 3.20 A 153.3 W 812.4 J");
}

TEST_CASE("format_message prints a status blink command with its target node") {
    const auto m =
        make_message(MsgClass::Command, back_aux::kNodeId, MsgType::Update, StatusBlink{250});
    REQUIRE(m.has_value());
    std::array<char, 96> line{};
    format_message(*m, line);
    CHECK(std::string_view(line.data()) == "status blink 250 ms to 0x20");
}

TEST_CASE("format_message does not print a status blink to a node that does not take it") {
    const auto m = make_message(MsgClass::Command, 0x42, MsgType::Update, StatusBlink{250});
    REQUIRE(m.has_value());
    std::array<char, 96> line{};
    format_message(*m, line);
    CHECK(std::string_view(line.data()) == "class 1 subaddress 0x42 type 0, 2 body bytes");
}

TEST_CASE("format_message falls back to the header for a message outside the catalog") {
    const Message m{
        .cls        = MsgClass::Command,
        .subaddress = 0x42,
        .type       = MsgType::Update,
        .body       = {0x01, 0x02},
        .body_len   = 2,
    };
    std::array<char, 96> line{};
    format_message(m, line);
    CHECK(std::string_view(line.data()) == "class 1 subaddress 0x42 type 0, 2 body bytes");
}

TEST_CASE("format_message truncates to the buffer and reports the full length") {
    const auto m = make_message(MsgClass::Command, kBodyStateId, MsgType::Update, BodyState{});
    REQUIRE(m.has_value());
    std::array<char, 5> line{};
    const int n = format_message(*m, line);
    CHECK(std::string_view(line.data()) == "body");
    CHECK(n == 15);
}

TEST_CASE("format_message prints a stop with its severity and sender") {
    const auto m = make_stop(2, 0x12, "brake fault");
    std::array<char, 96> line{};
    format_message(m, line);
    CHECK(std::string_view(line.data()) == "stop severity 2 from 0x12");
}

TEST_CASE("format_message prints a body state id with a non-Update type as a plain header") {
    const auto m = make_message(MsgClass::Command, kBodyStateId, MsgType::Action, BodyState{});
    REQUIRE(m.has_value());
    std::array<char, 96> line{};
    format_message(*m, line);
    CHECK(std::string_view(line.data()) == "class 1 subaddress 0x08 type 3, 1 body bytes");
}

TEST_CASE("format_message prints both aux boards' telemetry") {
    const auto front = make_message(MsgClass::Telemetry, front_aux::kNodeId, MsgType::Update,
                                    front_aux::Telemetry{12, 13, 2048});
    REQUIRE(front.has_value());
    std::array<char, 96> line{};
    format_message(*front, line);
    CHECK(std::string_view(line.data()) == "front_aux rpm 12/13 steering 2048");

    const auto back = make_message(MsgClass::Telemetry, back_aux::kNodeId, MsgType::Update,
                                   back_aux::Telemetry{7, 8, 900});
    REQUIRE(back.has_value());
    format_message(*back, line);
    CHECK(std::string_view(line.data()) == "back_aux rpm 7/8 brake 900");
}

TEST_CASE("format_message does not print a telemetry frame on 0x08 as a body state") {
    const auto m = make_message(MsgClass::Telemetry, kBodyStateId, MsgType::Update, BodyState{});
    REQUIRE(m.has_value());
    std::array<char, 96> line{};
    format_message(*m, line);
    CHECK(std::string_view(line.data()) == "class 2 subaddress 0x08 type 0, 1 body bytes");
}
