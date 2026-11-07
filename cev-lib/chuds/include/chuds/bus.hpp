#pragma once

#include <chrono>
#include <cstdint>
#include <expected>
#include <string_view>
#include <utility>

#include "chuds/message.hpp"
#include "chuds/transport.hpp"

namespace chuds {

// mirrors RxResult one level up
using RecvResult = std::expected<Message, RxError>;

// sends and receives chuds messages over a transport it owns
template <Transport T>
class Bus {
   public:
    // builds a default transport, such as an unopened SocketCanTransport
    Bus() = default;

    // takes over a transport built for it
    explicit Bus(T transport) : t_(std::move(transport)) {}

    Bus(const Bus&)            = delete;
    Bus& operator=(const Bus&) = delete;
    Bus(Bus&&)                 = delete;
    Bus& operator=(Bus&&)      = delete;
    ~Bus()                     = default;

    // periodic messages may ignore the result, since the next send replaces a lost one
    // one-shot commands such as STOP should check it and retry on QueueFull
    TxResult send(const Message& m) {
        const auto f = encode(m);
        if (!f) {
            return std::unexpected(TxError::Error);
        }
        return t_.send(*f);
    }

    // build a message from a struct body and send it on the id the struct owns
    template <class Body>
    TxResult send(const Body& body) {
        const auto m = make_message(body);
        if (!m) {
            return std::unexpected(TxError::Error);
        }
        return send(*m);
    }

    // the next message, or why there is none
    // Empty means nothing is waiting and is not a fault
    [[nodiscard]] RecvResult recv() {
        const auto r = t_.recv();
        if (!r) {
            return std::unexpected(r.error());
        }
        return decode(*r);
    }

    // open the transport by interface name, only for transports that have open()
    [[nodiscard]] bool open(std::string_view ifname)
        requires requires(T& t, std::string_view name) { t.open(name); }
    {
        return t_.open(ifname);
    }

    // block until a frame is waiting, only for transports that have wait()
    [[nodiscard]] bool wait(std::chrono::milliseconds timeout)
        requires requires(T& t, std::chrono::milliseconds ms) { t.wait(ms); }
    {
        return t_.wait(timeout);
    }

    // whether the transport initialized, only for transports that report it
    [[nodiscard]] bool ready() const
        requires requires(const T& t) { t.ready(); }
    {
        return t_.ready();
    }

    // the controller's error state, only for transports that report it
    [[nodiscard]] BusStatus status()
        requires requires(T& t) { t.status(); }
    {
        return t_.status();
    }

   private:
    T t_{};
};

}  // namespace chuds
