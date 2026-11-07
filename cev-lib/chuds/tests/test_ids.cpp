#include <doctest/doctest.h>

#include "chuds/ids.hpp"

using namespace chuds;

TEST_CASE("CanId packs class into the top bits and sub into the low byte") {
    CHECK(CanId(Band::Emergency, 0x00).raw() == 0x000);
    CHECK(CanId(Band::Command, 0x40).raw() == 0x140);
    CHECK(CanId(Band::Telemetry, 0x10).raw() == 0x210);
}

TEST_CASE("cls and subaddress round-trip the constructor") {
    const CanId id = CanId(Band::Command, 0x7A);
    CHECK(id.cls() == Band::Command);
    CHECK(id.subaddress() == 0x7A);
}

TEST_CASE("class ranges are dense and gapless") {
    CHECK(CanId(Band::Emergency, 0x00).raw() == 0x000);
    CHECK(class_max(Band::Emergency).raw() == 0x0FF);
    CHECK(CanId(Band::Command, 0x00).raw() == 0x100);
    CHECK(class_max(Band::Command).raw() == 0x1FF);
    CHECK(CanId(Band::Telemetry, 0x00).raw() == 0x200);
    CHECK(class_max(Band::Telemetry).raw() == 0x2FF);
    CHECK(CanId(Band::Status, 0x00).raw() == 0x300);
    CHECK(class_max(Band::Status).raw() == 0x3FF);
}

TEST_CASE("priority order: emergency beats command beats telemetry beats status") {
    CHECK(CanId(Band::Emergency, 0x00) < CanId(Band::Command, 0x00));
    CHECK(CanId(Band::Command, 0x00) < CanId(Band::Telemetry, 0x00));
    CHECK(CanId(Band::Telemetry, 0x00) < CanId(Band::Status, 0x00));
}

TEST_CASE("every id stays within the 11-bit standard range") {
    CHECK(CanId(Band::Status, 0xFF) <= kMaxStandardId);
}

TEST_CASE("the top standard id decodes to a reserved class") {
    const CanId top = CanId::from_raw(0x7FF);
    CHECK(top.is_standard());
    CHECK(top.subaddress() == 0xFF);
    CHECK_FALSE(is_valid(top.cls()));
    CHECK_FALSE(CanId::from_raw(0x800).is_standard());
}

TEST_CASE("only the four defined classes are valid") {
    CHECK(is_valid(Band::Emergency));
    CHECK(is_valid(Band::Command));
    CHECK(is_valid(Band::Telemetry));
    CHECK(is_valid(Band::Status));
    CHECK_FALSE(is_valid(static_cast<Band>(4)));
}
