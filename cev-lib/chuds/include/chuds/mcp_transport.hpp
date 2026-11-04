#pragma once

#include <algorithm>
#include <cstdint>
#include <expected>

#include "chuds/transport.hpp"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "mcp251863.hpp"

namespace chuds {

// whether a driver timing preset runs at this bitrate and sample point on the 40 MHz clock
constexpr bool timing_matches(const BitTiming& t, std::uint32_t bitrate,
                              std::uint32_t sample_point) {
    const std::uint32_t tq_per_bit = std::uint32_t{t.tseg1} + t.tseg2 + 3;
    return MCP251863_CLOCK_HZ == bitrate * (std::uint32_t{t.brp} + 1) * tq_per_bit &&
           100 * (std::uint32_t{t.tseg1} + 2) == sample_point * tq_per_bit;
}

// the transport applies these presets through init(), so they must match the chuds bus
static_assert(timing_matches(kBitTiming500K40MHz, kNominalBitrate, kNominalSamplePoint));
static_assert(timing_matches(kBitTiming2M40MHz, kDataBitrate, kDataSamplePoint));

// the rp2040 pins wired to an mcp251863
struct McpPins {
    unsigned sck;
    unsigned mosi;
    unsigned miso;
    unsigned cs;
    unsigned stby;
    unsigned nint;
};

// a Transport over the MCP251863 CAN-FD controller
class Mcp251863Transport {
   public:
    // sets up the spi block and pins, then initializes the chip
    // rp2040 spi pins alternate between spi0 and spi1 in banks of 8
    explicit Mcp251863Transport(const McpPins& pins)
        : spi_(spi_get_instance(pins.sck / 8 % 2)),
          // boards wire only nINT, and the transport never enables the driver's pin interrupts
          mcp_(spi_, pins.cs, pins.stby, pins.nint, pins.nint, pins.nint),
          nint_(pins.nint) {
        spi_init(spi_, MCP251863_BAUD_RATE);
        spi_set_format(spi_, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
        gpio_set_function(pins.sck, GPIO_FUNC_SPI);
        gpio_set_function(pins.mosi, GPIO_FUNC_SPI);
        gpio_set_function(pins.miso, GPIO_FUNC_SPI);

        // nINT is push-pull once the mcp is up, the pull-up holds it idle until then
        gpio_init(nint_);
        gpio_set_dir(nint_, GPIO_IN);
        gpio_pull_up(nint_);

        // init needs the spi and pin setup above
        // filter 0 with an empty mask routes every standard-id frame to the rx fifo
        // NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
        init_ok_ = mcp_.init() == Error::None && mcp_.configureTxFifo(kTxFifo) == Error::None &&
                   mcp_.configureRxFifo(kRxFifo, 0, 0, 0) == Error::None &&
                   mcp_.begin() == Error::None;
    }

    // a move keeps the setup already done, so a Bus can take the transport over
    Mcp251863Transport(Mcp251863Transport&&)                 = default;
    Mcp251863Transport(const Mcp251863Transport&)            = delete;
    Mcp251863Transport& operator=(const Mcp251863Transport&) = delete;
    Mcp251863Transport& operator=(Mcp251863Transport&&)      = delete;
    ~Mcp251863Transport()                                    = default;

    [[nodiscard]] TxResult send(const CanFrame& f) {
        if (!init_ok_) {
            return std::unexpected(TxError::Error);
        }
        if (!f.id.is_standard() || !is_valid_fd_len(f.len)) {
            return std::unexpected(TxError::Error);
        }

        // chuds frames are CAN-FD with bit-rate switch and a standard id
        const Error err = mcp_.start_push_canfd(kTxFifo, f.id.raw(), f.data.data(), f.len, true);
        if (err == Error::TxFifoFull) {
            // a bus-off keeps the tx queue full, so rule it out first
            if (bus_off()) {
                return std::unexpected(TxError::BusOff);
            }
            return std::unexpected(TxError::QueueFull);
        }
        if (err != Error::None || finish([this] { return mcp_.poll_push(); }) != Error::None ||
            mcp_.request_send(kTxFifo) != Error::None || mcp_.clear_send() != Error::None) {
            mcp_.abort();
            return std::unexpected(TxError::Error);
        }
        return {};
    }

    // whether the chip initialized
    [[nodiscard]] bool ready() const { return init_ok_; }

    // the chip's error state, read over spi
    // a chip that failed init is not on the bus, so it reports Off
    [[nodiscard]] BusStatus status() {
        if (!init_ok_) {
            return {.state = BusState::Off};
        }
        const Result<Status> r = mcp_.getStatus();
        if (!r) {
            return {.state = BusState::Off};
        }
        const Status& s = r.value();
        BusState state  = BusState::Active;
        if (s.bus_off) {
            state = BusState::Off;
        } else if (s.tx_error_passive || s.rx_error_passive) {
            state = BusState::Passive;
        }
        return {.state = state, .tx_errors = s.tx_error_count, .rx_errors = s.rx_error_count};
    }

    [[nodiscard]] RxResult recv() {
        if (!init_ok_) {
            return std::unexpected(RxError::Error);
        }
        // nINT high means no enabled interrupt is pending, so skip the spi reads
        // the driver also enables tx and error interrupts, so nINT is usually low anyway
        if (gpio_get(nint_)) {
            return std::unexpected(RxError::Empty);
        }
        const Error err = mcp_.start_pop_canfd(kRxFifo);
        if (err == Error::RxFifoEmpty) {
            // a dead bus is a fault, not a quiet one
            if (bus_off()) {
                return std::unexpected(RxError::BusOff);
            }

            const Result<FifoStatus> fifo = mcp_.getFIFOStatus(kRxFifo);
            if (fifo && fifo.value().rx_overflow) {
                // the flag is sticky and holds nINT low, so clear it once reported
                (void)mcp_.clearRxOverflow(kRxFifo);
                return std::unexpected(RxError::Overflow);
            }

            return std::unexpected(RxError::Empty);
        }
        if (err != Error::None || finish([this] { return mcp_.poll_pop(); }) != Error::None) {
            mcp_.abort();
            return std::unexpected(RxError::Error);
        }
        const Result<CanFdFrame> frame = mcp_.get_read_frame();
        if (!frame) {
            return std::unexpected(RxError::Error);
        }
        const CanFdFrame& cf = frame.value();

        // chuds is standard 11-bit CAN-FD only
        // reject classic, remote, extended, out-of-range
        if (cf.ide || cf.rtr || !cf.fdf || cf.id > kMaxStandardId.raw()) {
            return std::unexpected(RxError::ForeignFrame);
        }

        CanFrame out{};
        out.id  = CanId::from_raw(static_cast<std::uint16_t>(cf.id));
        out.len = cf.len > kMaxFdPayload ? kMaxFdPayload : cf.len;
        std::copy_n(cf.data, out.len, out.data.data());

        return out;
    }

   private:
    // driver fifo numbers, the chip's FIFO1 and FIFO2
    static constexpr std::uint8_t kTxFifo = 0;
    static constexpr std::uint8_t kRxFifo = 1;

    // one frame's dma takes under 100 us, so a millisecond means it stalled
    static constexpr std::uint32_t kDmaTimeoutUs = 1000;

    // waits out the dma transfer that poll reports on
    template <typename Poll>
    [[nodiscard]] Error finish(Poll poll) {
        const absolute_time_t deadline = make_timeout_time_us(kDmaTimeoutUs);
        Error err                      = poll();
        while (err == Error::Busy && !time_reached(deadline)) {
            err = poll();
        }
        return err;
    }

    [[nodiscard]] bool bus_off() {
        const Result<Status> s = mcp_.getStatus();
        return s && s.value().bus_off;
    }

    spi_inst_t* spi_;
    MCP251863 mcp_;
    unsigned nint_;
    bool init_ok_{};
};

// assert that this meets the requirement of a Transport
static_assert(Transport<Mcp251863Transport>);

}  // namespace chuds
