#pragma once

#include "chuds/transport.hpp"  // IWYU pragma: keep

#ifdef __linux__

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <climits>
#include <expected>
#include <string_view>

namespace chuds {

// a Transport over a Linux SocketCAN raw CAN-FD socket
// non-blocking: recv reports Empty on a quiet bus, BusOff on a dead one
// bus-off holds from the bus-off error frame until a restart or any received frame
class SocketCanTransport {
   public:
    SocketCanTransport() = default;

    // disallow copy ops
    // allow move ops bc this transport owns the fd
    SocketCanTransport(const SocketCanTransport&)            = delete;
    SocketCanTransport& operator=(const SocketCanTransport&) = delete;

    SocketCanTransport(SocketCanTransport&& other) noexcept
        : fd_(other.fd_), bus_off_(other.bus_off_) {
        other.fd_ = -1;
    }

    SocketCanTransport& operator=(SocketCanTransport&& other) noexcept {
        if (this != &other) {
            close_fd();
            fd_       = other.fd_;
            bus_off_  = other.bus_off_;
            other.fd_ = -1;
        }
        return *this;
    }

    ~SocketCanTransport() { close_fd(); }

    // open and bind a raw CAN-FD socket on ifname, e.g. "vcan0"
    // enables CAN-FD frames and non-blocking i/o
    // returns false on any failure
    [[nodiscard]] bool open(std::string_view ifname) {
        close_fd();

        // ifr_name needs room for its terminator
        if (ifname.size() >= IFNAMSIZ) {
            return false;
        }

        const int fd = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
        if (fd < 0) {
            return false;
        }

        int on{1};
        if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof(on)) < 0) {
            ::close(fd);
            return false;
        }

        // deliver bus-off and restart as error frames so recv tracks the bus state
        can_err_mask_t err_mask = CAN_ERR_BUSOFF | CAN_ERR_RESTARTED;
        if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) < 0) {
            ::close(fd);
            return false;
        }

        ifreq ifr{};
        // ifr is zero-initialized, so ifr_name stays terminated
        std::copy_n(ifname.data(), ifname.size(), ifr.ifr_name);
        if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
            ::close(fd);
            return false;
        }
        const int ifindex = ifr.ifr_ifindex;

        // the sockopt enables FD framing but does not prove the interface carries FD
        // an mtu below CANFD_MTU is classic-only
        if (::ioctl(fd, SIOCGIFMTU, &ifr) < 0 || ifr.ifr_mtu < static_cast<int>(CANFD_MTU)) {
            ::close(fd);
            return false;
        }

        sockaddr_can addr{};
        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifindex;
        // bind takes the generic sockaddr, the kernel reads it as sockaddr_can
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            ::close(fd);
            return false;
        }

        const int flags = ::fcntl(fd, F_GETFL, 0);
        if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
            ::close(fd);
            return false;
        }

        fd_      = fd;
        bus_off_ = false;
        return true;
    }

    [[nodiscard]] bool is_open() const { return fd_ >= 0; }

    // block until a frame or bus event is waiting, so the next recv has something to report
    // false on timeout, error, or a closed transport
    [[nodiscard]] bool wait(std::chrono::milliseconds timeout) const {
        if (fd_ < 0) {
            return false;
        }
        pollfd p{.fd = fd_, .events = POLLIN, .revents = 0};
        // poll takes an int, and a negative timeout would block forever
        const auto ms = std::clamp<std::chrono::milliseconds::rep>(timeout.count(), 0, INT_MAX);
        return ::poll(&p, 1, static_cast<int>(ms)) > 0;
    }

    [[nodiscard]] TxResult send(const CanFrame& f) const {
        if (fd_ < 0) {
            return std::unexpected(TxError::Error);
        }

        // reject a malformed frame before it corrupts memory or aliases an id
        // an out-of-range id would mask down to a valid one on the wire
        if (!f.id.is_standard() || !is_valid_fd_len(f.len)) {
            return std::unexpected(TxError::Error);
        }

        if (bus_off_) {
            return std::unexpected(TxError::BusOff);
        }

        canfd_frame cf{};
        cf.can_id = f.id.raw();
        // frame contract is always CAN-FD with bit-rate switch
        cf.flags = CANFD_BRS;
        cf.len   = f.len;
        std::copy_n(f.data.data(), f.len, cf.data);

        const ssize_t n = ::write(fd_, &cf, sizeof(cf));
        if (n == static_cast<ssize_t>(sizeof(cf))) {
            return {};
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS)) {
            return std::unexpected(TxError::QueueFull);
        }
        return std::unexpected(TxError::Error);
    }

    [[nodiscard]] RxResult recv() {
        if (fd_ < 0) {
            return std::unexpected(RxError::Error);
        }

        canfd_frame cf{};
        const ssize_t n = ::read(fd_, &cf, sizeof(cf));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                if (bus_off_) {
                    return std::unexpected(RxError::BusOff);
                }
                return std::unexpected(RxError::Empty);
            }
            return std::unexpected(RxError::Error);
        }

        // an error frame reports bus state, not data
        // we registered only bus-off and restart
        if ((cf.can_id & CAN_ERR_FLAG) != 0) {
            bus_off_ = (cf.can_id & CAN_ERR_BUSOFF) != 0;
            if (bus_off_) {
                return std::unexpected(RxError::BusOff);
            }
            return std::unexpected(RxError::Empty);
        }

        // a bus-off controller receives nothing, so any frame means the bus is back
        bus_off_ = false;

        // a 16-byte read is a classic CAN frame
        // chuds is CAN-FD only, so reject it
        if (n != static_cast<ssize_t>(sizeof(cf))) {
            return std::unexpected(RxError::ForeignFrame);
        }

        // chuds is standard-id CAN-FD only
        // reject remote and extended frames rather than forging a standard id
        if ((cf.can_id & (CAN_RTR_FLAG | CAN_EFF_FLAG)) != 0) {
            return std::unexpected(RxError::ForeignFrame);
        }
        // a standard frame carries no stray bits above the 11-bit id
        if ((cf.can_id & CAN_EFF_MASK) > CAN_SFF_MASK) {
            return std::unexpected(RxError::ForeignFrame);
        }

        CanFrame out{};
        out.id  = CanId::from_raw(static_cast<std::uint16_t>(cf.can_id & CAN_SFF_MASK));
        out.len = cf.len > kMaxFdPayload ? kMaxFdPayload : cf.len;
        std::copy_n(cf.data, out.len, out.data.data());
        return out;
    }

   private:
    void close_fd() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    int fd_{-1};
    bool bus_off_{false};
};

static_assert(Transport<SocketCanTransport>);

}  // namespace chuds

#endif  // __linux__
