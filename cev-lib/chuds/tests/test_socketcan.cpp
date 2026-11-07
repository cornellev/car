#include <doctest/doctest.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <expected>
#include <string_view>
#include <utility>

#include "chuds/bus.hpp"
#include "chuds/frame.hpp"
#include "chuds/ids.hpp"
#include "chuds/message.hpp"
#include "chuds/socketcan_transport.hpp"  // IWYU pragma: keep
#include "chuds/transport.hpp"

using namespace chuds;

namespace {
// the interface the integration tests use; a vcan is enough
const char* test_if() {
    const char* e = std::getenv("CHUDS_TEST_CAN_IF");
    return (e != nullptr && *e != '\0') ? e : "vcan0";
}

// true when the test should skip for want of an interface
// CHUDS_REQUIRE_CAN turns the skip into a failure
bool skip_without_can(bool opened) {
    if (opened) {
        return false;
    }
    if (std::getenv("CHUDS_REQUIRE_CAN") != nullptr) {
        FAIL("no CAN-FD interface '" << test_if() << "' and CHUDS_REQUIRE_CAN is set");
    }
    MESSAGE("no CAN-FD interface '" << test_if()
                                    << "', skipping. set one up with: just cev-lib chuds vcan-up");
    return true;
}

// wait for a frame or bus event, then receive it
RxResult recv_wait(SocketCanTransport& t) {
    if (!t.wait(std::chrono::seconds{1})) {
        return std::unexpected(RxError::Empty);
    }
    return t.recv();
}

// receive n frames and count how many were foreign
int count_foreign(SocketCanTransport& t, int n) {
    int count = 0;
    for (int i = 0; i < n; ++i) {
        const auto r = recv_wait(t);
        if (!r && r.error() == RxError::ForeignFrame) {
            ++count;
        }
    }
    return count;
}

// a raw CAN socket for injecting frames the driver would never send itself
int open_raw(bool fd) {
    const int s = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        return -1;
    }
    if (fd) {
        int on{1};
        if (::setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof(on)) < 0) {
            ::close(s);
            return -1;
        }
    }
    ifreq ifr{};
    std::strncpy(ifr.ifr_name, test_if(), IFNAMSIZ - 1);
    if (::ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        ::close(s);
        return -1;
    }
    sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(s);
        return -1;
    }
    return s;
}

// wait for a frame on a raw socket, then read it
ssize_t read_raw(int s, canfd_frame& cf) {
    pollfd p{.fd = s, .events = POLLIN, .revents = 0};
    if (::poll(&p, 1, 1000) <= 0) {
        return -1;
    }
    return ::recv(s, &cf, sizeof(cf), MSG_DONTWAIT);
}
}  // namespace

static_assert(Transport<SocketCanTransport>);

TEST_CASE("a default transport is closed and reports errors, not silence") {
    SocketCanTransport t;
    CHECK_FALSE(t.is_open());
    CHECK_FALSE(t.wait(std::chrono::milliseconds{0}));
    CanFrame f{};
    f.id            = CanId::from_raw(0x123);
    f.len           = 2;
    const auto sent = t.send(f);
    REQUIRE_FALSE(sent.has_value());
    CHECK(sent.error() == TxError::Error);
    const auto got = t.recv();
    REQUIRE_FALSE(got.has_value());
    CHECK(got.error() == RxError::Error);
}

TEST_CASE("opening a nonexistent interface fails") {
    SocketCanTransport t;
    CHECK_FALSE(t.open("nosuchcan0"));
    CHECK_FALSE(t.is_open());
}

TEST_CASE("opening an interface name too long for the kernel fails") {
    SocketCanTransport t;
    std::array<char, IFNAMSIZ> name{};
    name.fill('a');
    CHECK_FALSE(t.open(std::string_view(name.data(), name.size())));
    CHECK_FALSE(t.is_open());
}

TEST_CASE("a move hands over the socket and leaves the source closed") {
    SocketCanTransport a;
    if (skip_without_can(a.open(test_if()))) {
        return;
    }
    SocketCanTransport b{std::move(a)};
    // the test checks the moved-from state on purpose
    // NOLINTNEXTLINE(bugprone-use-after-move,clang-analyzer-cplusplus.Move)
    CHECK_FALSE(a.is_open());
    CHECK(b.is_open());

    SocketCanTransport c;
    c = std::move(b);
    // the test checks the moved-from state on purpose
    // NOLINTNEXTLINE(bugprone-use-after-move,clang-analyzer-cplusplus.Move)
    CHECK_FALSE(b.is_open());
    CHECK(c.is_open());
    CHECK(c.recv() == std::unexpected(RxError::Empty));
}

TEST_CASE("a bus forwards open and wait to its socketcan transport") {
    Bus<SocketCanTransport> bus;
    if (skip_without_can(bus.open(test_if()))) {
        return;
    }
    CHECK_FALSE(bus.wait(std::chrono::milliseconds{20}));
}

TEST_CASE("round-trips a chuds message over a CAN-FD interface") {
    SocketCanTransport tx;
    SocketCanTransport rx;
    if (skip_without_can(rx.open(test_if()) && tx.open(test_if()))) {
        return;
    }
    const Message msg{
        .cls        = Band::Telemetry,
        .subaddress = 0x10,
        .body       = {0x01, 0x39, 0x05},
        .body_len   = 3,
    };
    const auto frame = encode(msg);
    REQUIRE(frame.has_value());
    REQUIRE(tx.send(*frame).has_value());

    const auto r = recv_wait(rx);
    REQUIRE(r.has_value());
    const auto out = decode(*r);
    REQUIRE(out.has_value());
    CHECK(out->cls == Band::Telemetry);
    CHECK(out->subaddress == 0x10);
    REQUIRE(out->body_view().size() == 3);
    CHECK(out->body_view()[2] == 0x05);
}

TEST_CASE("recv reports empty on a quiet bus") {
    SocketCanTransport rx;
    if (skip_without_can(rx.open(test_if()))) {
        return;
    }
    CHECK(rx.recv() == std::unexpected(RxError::Empty));
}

TEST_CASE("wait times out on a quiet bus and wakes when a frame arrives") {
    SocketCanTransport tx;
    SocketCanTransport rx;
    if (skip_without_can(rx.open(test_if()) && tx.open(test_if()))) {
        return;
    }
    CHECK_FALSE(rx.wait(std::chrono::milliseconds{20}));

    CanFrame f{};
    f.id  = CanId::from_raw(0x123);
    f.len = 2;
    REQUIRE(tx.send(f).has_value());
    CHECK(rx.wait(std::chrono::seconds{1}));
    CHECK(rx.recv().has_value());
}

TEST_CASE("send rejects a malformed frame before it reaches the wire") {
    SocketCanTransport tx;
    if (skip_without_can(tx.open(test_if()))) {
        return;
    }
    CanFrame bad_id{};
    bad_id.id  = CanId::from_raw(0x800);  // past the 11-bit standard range
    bad_id.len = 2;
    CHECK(tx.send(bad_id) == std::unexpected(TxError::Error));

    CanFrame bad_len{};
    bad_len.id  = CanId::from_raw(0x100);
    bad_len.len = 65;  // past the FD payload
    CHECK(tx.send(bad_len) == std::unexpected(TxError::Error));

    bad_len.len = 9;  // not a CAN-FD size
    CHECK(tx.send(bad_len) == std::unexpected(TxError::Error));
}

TEST_CASE("recv rejects foreign frames instead of forging a standard id") {
    SocketCanTransport rx;
    if (skip_without_can(rx.open(test_if()))) {
        return;
    }
    const int classic = open_raw(false);
    REQUIRE(classic >= 0);
    can_frame cf{};
    cf.can_id  = 0x123;
    cf.can_dlc = 2;
    REQUIRE(::write(classic, &cf, sizeof(cf)) == static_cast<ssize_t>(sizeof(cf)));

    const int ext = open_raw(true);
    REQUIRE(ext >= 0);
    canfd_frame ef{};
    ef.can_id = 0x800 | CAN_EFF_FLAG;  // would alias to 0x000 (STOP) if masked
    ef.len    = 8;
    REQUIRE(::write(ext, &ef, sizeof(ef)) == static_cast<ssize_t>(sizeof(ef)));

    // every foreign frame is rejected, never handed on as a decodable frame
    CHECK(count_foreign(rx, 2) == 2);
    ::close(classic);
    ::close(ext);
}

TEST_CASE("recv rejects remote frames and ids past the standard range") {
    SocketCanTransport rx;
    if (skip_without_can(rx.open(test_if()))) {
        return;
    }
    const int raw = open_raw(true);
    REQUIRE(raw >= 0);
    canfd_frame rtr{};
    rtr.can_id = 0x123 | CAN_RTR_FLAG;
    rtr.len    = 8;
    REQUIRE(::write(raw, &rtr, sizeof(rtr)) == static_cast<ssize_t>(sizeof(rtr)));

    canfd_frame wide{};
    wide.can_id = 0x800;  // no EFF flag, but past the 11-bit range
    wide.len    = 8;
    REQUIRE(::write(raw, &wide, sizeof(wide)) == static_cast<ssize_t>(sizeof(wide)));

    CHECK(count_foreign(rx, 2) == 2);
    ::close(raw);
}

TEST_CASE("send puts the exact id, BRS flag, length, and bytes on the wire") {
    SocketCanTransport tx;
    if (skip_without_can(tx.open(test_if()))) {
        return;
    }
    const int raw = open_raw(true);
    REQUIRE(raw >= 0);

    CanFrame f{};
    f.id  = CanId::from_raw(0x123);
    f.len = 12;
    for (std::uint8_t i = 0; i < f.len; ++i) {
        f.data[i] = static_cast<std::uint8_t>(0xA0 + i);
    }
    REQUIRE(tx.send(f).has_value());

    canfd_frame cf{};
    REQUIRE(read_raw(raw, cf) == static_cast<ssize_t>(sizeof(cf)));
    CHECK(cf.can_id == 0x123);
    CHECK((cf.flags & CANFD_BRS) != 0);
    CHECK(cf.len == 12);
    for (std::uint8_t i = 0; i < f.len; ++i) {
        CHECK(cf.data[i] == f.data[i]);
    }
    ::close(raw);
}

TEST_CASE("a bus-off error frame holds BusOff until a restart") {
    SocketCanTransport t;
    if (skip_without_can(t.open(test_if()))) {
        return;
    }
    const int raw = open_raw(false);
    REQUIRE(raw >= 0);
    can_frame err{};
    err.can_id  = CAN_ERR_FLAG | CAN_ERR_BUSOFF;
    err.can_dlc = CAN_ERR_DLC;
    REQUIRE(::write(raw, &err, sizeof(err)) == static_cast<ssize_t>(sizeof(err)));

    CHECK(recv_wait(t) == std::unexpected(RxError::BusOff));
    // a quiet bus after bus-off is still BusOff, not an empty success
    CHECK(t.recv() == std::unexpected(RxError::BusOff));

    CanFrame f{};
    f.id  = CanId::from_raw(0x123);
    f.len = 2;
    CHECK(t.send(f) == std::unexpected(TxError::BusOff));

    err.can_id = CAN_ERR_FLAG | CAN_ERR_RESTARTED;
    REQUIRE(::write(raw, &err, sizeof(err)) == static_cast<ssize_t>(sizeof(err)));

    RxResult r = std::unexpected(RxError::BusOff);
    while (r == std::unexpected(RxError::BusOff) && t.wait(std::chrono::seconds{1})) {
        r = t.recv();
    }
    CHECK(r == std::unexpected(RxError::Empty));
    ::close(raw);
}

TEST_CASE("any data frame clears bus-off, since a bus-off controller receives nothing") {
    SocketCanTransport t;
    if (skip_without_can(t.open(test_if()))) {
        return;
    }
    const int raw = open_raw(true);
    REQUIRE(raw >= 0);
    canfd_frame err{};
    err.can_id = CAN_ERR_FLAG | CAN_ERR_BUSOFF;
    err.len    = CAN_ERR_DLC;
    REQUIRE(::write(raw, &err, CAN_MTU) == static_cast<ssize_t>(CAN_MTU));
    CHECK(recv_wait(t) == std::unexpected(RxError::BusOff));

    canfd_frame data{};
    data.can_id = 0x123;
    data.len    = 8;
    REQUIRE(::write(raw, &data, sizeof(data)) == static_cast<ssize_t>(sizeof(data)));
    CHECK(recv_wait(t).has_value());

    CanFrame f{};
    f.id  = CanId::from_raw(0x123);
    f.len = 2;
    CHECK(t.send(f).has_value());
    ::close(raw);
}
