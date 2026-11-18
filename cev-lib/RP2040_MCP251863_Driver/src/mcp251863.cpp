#include "mcp251863.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <utility>

// a macro, since it returns from the function that uses it
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define MCP_TRY(expr)           \
    do {                        \
        const Error e = (expr); \
        if (e != Error::None) { \
            return e;           \
        }                       \
    } while (0)

MCP251863* MCP251863::instance_ = nullptr;

namespace {

std::optional<PayloadSize> canfd_len_to_dlc(size_t len) {
    switch (len) {
        case 0: return PayloadSize::PL_SIZE_MCP_0;
        case 1: return PayloadSize::PL_SIZE_MCP_1;
        case 2: return PayloadSize::PL_SIZE_MCP_2;
        case 3: return PayloadSize::PL_SIZE_MCP_3;
        case 4: return PayloadSize::PL_SIZE_MCP_4;
        case 5: return PayloadSize::PL_SIZE_MCP_5;
        case 6: return PayloadSize::PL_SIZE_MCP_6;
        case 7: return PayloadSize::PL_SIZE_MCP_7;
        case 8: return PayloadSize::PL_SIZE_MCP_8;
        case 12: return PayloadSize::PL_SIZE_MCP_12;
        case 16: return PayloadSize::PL_SIZE_MCP_16;
        case 20: return PayloadSize::PL_SIZE_MCP_20;
        case 24: return PayloadSize::PL_SIZE_MCP_24;
        case 32: return PayloadSize::PL_SIZE_MCP_32;
        case 48: return PayloadSize::PL_SIZE_MCP_48;
        case 64: return PayloadSize::PL_SIZE_MCP_64;
        default: return std::nullopt;
    }
}

uint8_t canfd_dlc_to_len(uint8_t dlc) {
    switch (static_cast<PayloadSize>(dlc & 0x0F)) {
        case PayloadSize::PL_SIZE_MCP_0: return 0;
        case PayloadSize::PL_SIZE_MCP_1: return 1;
        case PayloadSize::PL_SIZE_MCP_2: return 2;
        case PayloadSize::PL_SIZE_MCP_3: return 3;
        case PayloadSize::PL_SIZE_MCP_4: return 4;
        case PayloadSize::PL_SIZE_MCP_5: return 5;
        case PayloadSize::PL_SIZE_MCP_6: return 6;
        case PayloadSize::PL_SIZE_MCP_7: return 7;
        case PayloadSize::PL_SIZE_MCP_8: return 8;
        case PayloadSize::PL_SIZE_MCP_12: return 12;
        case PayloadSize::PL_SIZE_MCP_16: return 16;
        case PayloadSize::PL_SIZE_MCP_20: return 20;
        case PayloadSize::PL_SIZE_MCP_24: return 24;
        case PayloadSize::PL_SIZE_MCP_32: return 32;
        case PayloadSize::PL_SIZE_MCP_48: return 48;
        case PayloadSize::PL_SIZE_MCP_64: return 64;
        default: return 0;
    }
}

uint8_t fifo_plsize_to_len(FifoPayloadSize size) {
    switch (size) {
        case FifoPayloadSize::FIFO_PLSIZE_8: return 8;
        case FifoPayloadSize::FIFO_PLSIZE_12: return 12;
        case FifoPayloadSize::FIFO_PLSIZE_16: return 16;
        case FifoPayloadSize::FIFO_PLSIZE_20: return 20;
        case FifoPayloadSize::FIFO_PLSIZE_24: return 24;
        case FifoPayloadSize::FIFO_PLSIZE_32: return 32;
        case FifoPayloadSize::FIFO_PLSIZE_48: return 48;
        case FifoPayloadSize::FIFO_PLSIZE_64: return 64;
        default: return 0;
    }
}

uint8_t can_dlc_to_len(uint8_t dlc, bool fdf) {
    if (!fdf && ((dlc & 0x0F) > 8)) {
        return 8;
    }
    return canfd_dlc_to_len(dlc);
}

Error create_message_obj(uint8_t* dst, const CanFdFrame& frame, size_t* objectSize);

uint32_t pack_nominal_bit_timing(BitTiming timing) {
    return (((uint32_t)timing.brp & 0xFF) << 24) | (((uint32_t)timing.tseg1 & 0xFF) << 16) |
           (((uint32_t)timing.tseg2 & 0x7F) << 8) | ((uint32_t)timing.sjw & 0x7F);
}

uint32_t pack_data_bit_timing(BitTiming timing) {
    return (((uint32_t)timing.brp & 0xFF) << 24) | (((uint32_t)timing.tseg1 & 0xFF) << 16) |
           (((uint32_t)timing.tseg2 & 0x0F) << 8) | ((uint32_t)timing.sjw & 0x0F);
}

Error validate_bit_timing(const BitTiming& nominal, const BitTiming& data) {
    if (nominal.tseg2 > 0x7F || nominal.sjw > 0x7F) {
        return Error::InvalidBitTiming;
    }
    if (data.tseg1 > 0x1F || data.tseg2 > 0x0F || data.sjw > 0x0F) {
        return Error::InvalidBitTiming;
    }
    return Error::None;
}

uint32_t encode_tdc(bool enable, uint8_t offset) {
    if (!enable) {
        return 0;
    }

    // 10 at bits 17:16 for TDCMOD auto
    // TDCO at bits 14:8
    return (2UL << 16) | ((offset & 0x7F) << 8);
}

InitConfig default_init_config() {
    InitConfig config{};

    config.enablePll         = 0;
    config.sclkDiv2          = 0;
    config.enableTdc         = 1;
    config.rxTimestampEnable = 0;
    // (DTSEG1 + 1) tq, microchip's value for a 40 MHz clock at 2 Mbit/s
    config.tdcOffset = 15;

    // FIFO depth is the number of message slots, 1..32
    // FSIZE register field stores it as 0..31
    // we subtract 1 from fsize at call sites
    config.txFifoDepth = 8;
    config.rxFifoDepth = 8;

    config.txPayloadSize = FifoPayloadSize::FIFO_PLSIZE_64;
    config.rxPayloadSize = FifoPayloadSize::FIFO_PLSIZE_64;

    // Defaults assume a 40 MHz CAN clock: nominal 500 kbit/s, data 2 Mbit/s.
    config.nominalBitTiming = kBitTiming500K40MHz;
    config.dataBitTiming    = kBitTiming2M40MHz;

    return config;
}

uint32_t pack_id_word(const CanFdFrame& frame) {
    if (frame.ide) {
        return ((frame.id >> 18) & 0x7FF) | ((frame.id & 0x3FFFF) << 11);
    }

    const uint32_t sid   = frame.id & 0x7FF;
    const uint32_t sid11 = (frame.sid11 || (frame.id > 0x7FF)) ? ((frame.id >> 11) & 0x01) : 0;
    return sid | (sid11 << 29);
}

uint32_t pack_control_word(const CanFdFrame& frame) {
    return ((uint32_t)frame.dlc & 0x0F) | (frame.ide ? (1UL << 4) : 0) |
           (frame.rtr ? (1UL << 5) : 0) | (frame.brs ? (1UL << 6) : 0) |
           (frame.fdf ? (1UL << 7) : 0) | (frame.esi ? (1UL << 8) : 0);
}

void store_word(uint8_t* dst, uint32_t word) {
    dst[0] = (uint8_t)(word & 0xFF);
    dst[1] = (uint8_t)((word >> 8) & 0xFF);
    dst[2] = (uint8_t)((word >> 16) & 0xFF);
    dst[3] = (uint8_t)((word >> 24) & 0xFF);
}

uint32_t load_word(const uint8_t* src) {
    return ((uint32_t)src[0]) | ((uint32_t)src[1] << 8) | ((uint32_t)src[2] << 16) |
           ((uint32_t)src[3] << 24);
}

Error finalize_frame_dlc(CanFdFrame* frame) {
    if (frame->fdf) {
        auto dlc_opt = canfd_len_to_dlc(frame->len);
        if (!dlc_opt) {
            return Error::InvalidPayloadSize;
        }
        frame->dlc = std::to_underlying(*dlc_opt);
        return Error::None;
    }

    if (frame->len > 8) {
        return Error::InvalidPayloadSize;
    }
    frame->dlc = frame->len;
    frame->brs = false;
    return Error::None;
}

Error validate_tx_frame(const CanFdFrame& frame) {
    if (frame.ide && (frame.id > 0x1FFFFFFF)) {
        return Error::IdOutOfRange;
    }
    if (!frame.ide && (frame.id > 0x7FF)) {
        return Error::IdOutOfRange;
    }
    if (frame.len > 64) {
        return Error::InvalidPayloadSize;
    }
    if (!frame.fdf && frame.len > 8) {
        return Error::InvalidPayloadSize;
    }
    return Error::None;
}

CanFdFrame decode_rx_header(const uint8_t* header, bool timestampEnabled) {
    CanFdFrame frame{};

    const uint32_t word0  = load_word(header);
    const uint32_t word1  = load_word(header + 4);
    frame.dlc             = word1 & 0x0F;
    frame.ide             = (word1 & (1UL << 4)) != 0;
    frame.rtr             = (word1 & (1UL << 5)) != 0;
    frame.brs             = (word1 & (1UL << 6)) != 0;
    frame.fdf             = (word1 & (1UL << 7)) != 0;
    frame.esi             = (word1 & (1UL << 8)) != 0;
    frame.filter_hit      = (word1 >> 11) & 0x1F;
    frame.sid11           = (word0 & (1UL << 29)) != 0;
    frame.len             = can_dlc_to_len(frame.dlc, frame.fdf);
    frame.timestamp_valid = timestampEnabled;
    if (timestampEnabled) {
        frame.timestamp = load_word(header + 8);
    }
    // suspicious
    if (frame.ide) {
        frame.id = ((word0 & 0x7FF) << 18) | ((word0 >> 11) & 0x3FFFF);
    } else {
        frame.id = (word0 & 0x7FF) | ((uint32_t)(frame.sid11 ? 1 : 0) << 11);
    }

    return frame;
}

Error create_message_obj(uint8_t* dst, const CanFdFrame& frame, size_t* objectSize) {
    CanFdFrame txFrame = frame;
    MCP_TRY(validate_tx_frame(txFrame));
    MCP_TRY(finalize_frame_dlc(&txFrame));

    const size_t rawSize = 8 + (size_t)txFrame.len;
    const size_t objSize = (rawSize + 3) & ~static_cast<size_t>(3);  // round up to multiple of 4

    memset(dst, 0, objSize);
    store_word(dst, pack_id_word(txFrame));
    store_word(dst + 4, pack_control_word(txFrame));
    for (uint8_t i = 0; i < txFrame.len; i++) {
        dst[8 + i] = txFrame.data[i];
    }

    if (objectSize != nullptr) {
        *objectSize = objSize;
    }
    return Error::None;
}

}  // namespace

MCP251863::MCP251863(spi_inst_t* ispi, uint iCSPin, uint iSTBYPin, uint iINTPin, uint iINT0Pin,
                     uint iINT1Pin)
    : spi_(ispi),
      chipSelectPin_(iCSPin),
      standbyPin_(iSTBYPin),
      intPin_(iINTPin),
      int0Pin_(iINT0Pin),
      int1Pin_(iINT1Pin) {
    instance_ = this;
}

// Checks if a FIFO was actually configured and in the direction expected
Error MCP251863::checkFifo(uint8_t fifoNum, bool wantTx) const {
    if (fifoNum > 30) {
        return Error::InvalidFifoNum;
    }
    const FifoInfo& info = fifoInfo[fifoNum];
    if (!info.configured || info.isTx != wantTx) {
        return Error::InvalidFifoNum;
    }
    return Error::None;
}

void MCP251863::csSelect() const {
    asm volatile("nop \n nop \n nop");
    gpio_put(chipSelectPin_, false);
    asm volatile("nop \n nop \n nop");
}

void MCP251863::csDeselect() const {
    asm volatile("nop \n nop \n nop");
    gpio_put(chipSelectPin_, true);
    asm volatile("nop \n nop \n nop");
}

Error MCP251863::dmaWriteAddr(uint16_t startAddr, const uint8_t* data, size_t len) {
    if (data == nullptr && len > 0) {
        return Error::NullPointer;
    }
    if (len + 2 > MAX_TRANSFER) {
        return Error::InvalidPayloadSize;
    }

    // form message CCCC-AAAAAAAAAAAA
    tx_buff[0] = (std::to_underlying(Command::CMD_MCP_WRITA) << 4) | (startAddr >> 8);
    tx_buff[1] = (startAddr << 4) >> 4;

    memcpy(&tx_buff[2], data, len);

    transfer_len = len;

    spiTransferDMA(len + 2);

    return Error::None;
}

// blocking SPI
// CRC mode and Safe mode unimplemented
Error MCP251863::writeAddr(uint16_t startAddr, const uint8_t* data, size_t len) {
    if (data == nullptr && len > 0) {
        return Error::NullPointer;
    }
    // form message CCCC-AAAAAAAAAAAA
    uint8_t message[2];

    message[0] = (std::to_underlying(Command::CMD_MCP_WRITA) << 4) | (startAddr >> 8);
    message[1] = (startAddr << 4) >> 4;

    csSelect();
    spi_write_blocking(spi_, message, 2);
    spi_write_blocking(spi_, data, len);
    csDeselect();

    return Error::None;
}

Error MCP251863::dmaReadAddr(uint16_t startAddr, uint8_t* dst, size_t len) {
    if (dst == nullptr) {
        return Error::NullPointer;
    }
    const size_t paddedLen = (len + 3) / 4 * 4 + 2;
    if (paddedLen > MAX_TRANSFER) {
        return Error::InvalidPayloadSize;
    }

    // form message CCCC-AAAAAAAAAAAA
    tx_buff[0] = (std::to_underlying(Command::CMD_MCP_READA) << 4) | (startAddr >> 8);
    tx_buff[1] = (startAddr << 4) >> 4;

    user_rx_buff = dst;

    transfer_len = len;

    spiTransferDMA(paddedLen);

    return Error::None;
}

Error MCP251863::readAddr(uint16_t startAddr, uint8_t* dst, size_t len) {
    if (dst == nullptr) {
        return Error::NullPointer;
    }
    uint8_t message[2] = {0};

    // form message CCCC-AAAAAAAAAAAA
    message[0] = (std::to_underlying(Command::CMD_MCP_READA) << 4) | (startAddr >> 8);
    message[1] = (startAddr << 4) >> 4;

    csSelect();
    spi_write_blocking(spi_, message, 2);
    spi_read_blocking(spi_, 0, dst, len);
    csDeselect();

    return Error::None;
}

// helper functions for reading/writing to 4 byte registers
Error MCP251863::readReg(uint16_t addr, uint32_t* dst) {
    if (dst == nullptr) {
        return Error::NullPointer;
    }
    uint8_t buf[4] = {0};
    MCP_TRY(readAddr(addr, buf, 4));
    *dst = ((uint32_t)buf[0]) | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) |
           ((uint32_t)buf[3] << 24);
    return Error::None;
}

Error MCP251863::writeReg(uint16_t addr, uint32_t value) {
    uint8_t buff[4] = {
        (uint8_t)(value & 0xFF),
        (uint8_t)((value >> 8) & 0xFF),
        (uint8_t)((value >> 16) & 0xFF),
        (uint8_t)((value >> 24) & 0xFF),
    };
    return writeAddr(addr, buff, 4);
}

// R-M-W
Error MCP251863::updateByte(uint16_t addr, uint8_t field, uint8_t value) {
    uint8_t buff = 0;
    MCP_TRY(readAddr(addr, &buff, 1));
    buff = (uint8_t)((buff & (uint8_t)~field) | value);
    return writeAddr(addr, &buff, 1);
}

Error MCP251863::pollRegisterBit(uint16_t addr, uint8_t mask, bool wantSet, int maxIters,
                                 uint32_t delayUs, Error timeoutType) {
    for (int i = 0; i < maxIters; i++) {
        uint8_t buff = 0;
        MCP_TRY(readAddr(addr, &buff, 1));
        // if wantSet is false, then bit = 0 -> no error
        // if wantSet is true, then bit = 1 -> no error
        if (((buff & mask) != 0) == wantSet) {
            return Error::None;
        }
        sleep_us(delayUs);
    }
    stats.poll_timeouts++;
    return timeoutType;
}

Error MCP251863::pollRegisterBit(uint16_t addr, uint8_t mask, bool wantSet, Error timeoutType) {
    // default 100 iterations, 1 ms delay
    return pollRegisterBit(addr, mask, wantSet, 100, 1000, timeoutType);
}

Error MCP251863::readOpMode(uint8_t* opmod) {
    uint8_t buff = 0;
    MCP_TRY(readAddr(std::to_underlying(RegisterAddress::REG_MCP_C1CON) + 2, &buff, 1));
    *opmod = buff >> 5;
    return Error::None;
}

Error MCP251863::waitForOpMode(ControllerMode target, int maxIters, uint32_t delayUs) {
    for (int i = 0; i < maxIters; i++) {
        uint8_t mode = 0;
        MCP_TRY(readOpMode(&mode));
        if (mode == std::to_underlying(target)) {
            return Error::None;
        }
        sleep_us(delayUs);
    }
    stats.poll_timeouts++;
    return Error::ModeChangeTimeout;
}

Error MCP251863::waitForOpMode(ControllerMode target) { return waitForOpMode(target, 100, 1000); }

Error MCP251863::readFifoUserAddress(uint16_t fifoPointAddr, size_t objectSize,
                                     uint16_t* messageAddr) {
    uint16_t ua = 0;
    // the register is little-endian like the rp2040, so its 2 bytes read straight into ua
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    MCP_TRY(readAddr(fifoPointAddr, reinterpret_cast<uint8_t*>(&ua), 2));
    // RAM is 2 KB (0x400 - 0xBFF)
    // Thus buff value should be between 0x00 and 0x800 - objectSize
    if (ua > 0x800 - objectSize) {
        return Error::BadRegisterValue;
    }
    // add offset
    *messageAddr = ua + 0x400;
    return Error::None;
}

Error MCP251863::initDMA() {
    if (dmaInitialized) {
        dma_channel_set_irq0_enabled(dma_rx_chan, false);
        dma_channel_unclaim(dma_tx_chan);
        dma_channel_unclaim(dma_rx_chan);
        dmaInitialized = false;
    }
    const int tx = dma_claim_unused_channel(false);
    dma_tx_chan  = tx;
    dma_tx_cfg   = dma_channel_get_default_config(dma_tx_chan);
    channel_config_set_transfer_data_size(&dma_tx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&dma_tx_cfg, true);
    channel_config_set_write_increment(&dma_tx_cfg, false);
    channel_config_set_dreq(&dma_tx_cfg, spi_get_dreq(spi_, true));
    dma_channel_configure(dma_tx_chan, &dma_tx_cfg, &spi_get_hw(spi_)->dr, tx_buff, 0, false);

    const int rx = dma_claim_unused_channel(false);
    dma_rx_chan  = rx;
    dma_rx_cfg   = dma_channel_get_default_config(dma_rx_chan);
    channel_config_set_transfer_data_size(&dma_rx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&dma_rx_cfg, false);
    channel_config_set_write_increment(&dma_rx_cfg, true);
    channel_config_set_dreq(&dma_rx_cfg, spi_get_dreq(spi_, false));
    dma_channel_configure(dma_rx_chan, &dma_rx_cfg, rx_buff, &spi_get_hw(spi_)->dr, 0, false);

    // DMA raises IRQ line 0 when it finishes
    // call dmaIrqHandler whenever DMA int 0 fires
    dma_channel_set_irq0_enabled(dma_rx_chan,  // RX finishes after TX
                                 true);
    // no one else should use the DMA_IRQ_0 line
    irq_set_exclusive_handler(DMA_IRQ_0, MCP251863::dmaIrqHandler);
    irq_set_enabled(DMA_IRQ_0, true);
    dmaInitialized = true;
    return Error::None;
}

void MCP251863::spiTransferDMA(size_t len) {
    // the irq must reach this object even after the driver was moved
    instance_ = this;

    // abort possible unfinished transactions
    // rp2040-e13: an abort can raise a spurious completion irq, so mask and clear it
    dma_channel_set_irq0_enabled(dma_rx_chan, false);
    dma_channel_abort(dma_tx_chan);
    dma_channel_abort(dma_rx_chan);
    dma_channel_acknowledge_irq0(dma_rx_chan);
    dma_channel_set_irq0_enabled(dma_rx_chan, true);

    // clear overrun and drain RX FIFO
    spi_get_hw(spi_)->icr = SPI_SSPICR_RORIC_BITS;
    while (spi_is_readable(spi_)) {
        (void)spi_get_hw(spi_)->dr;
    }

    dma_channel_set_read_addr(dma_tx_chan, tx_buff, false);
    dma_channel_set_write_addr(dma_rx_chan, rx_buff, false);

    dma_channel_set_trans_count(dma_tx_chan, len, false);
    dma_channel_set_trans_count(dma_rx_chan, len, false);

    gpio_put(chipSelectPin_, false);

    // start both DMA channels
    dma_start_channel_mask((1u << dma_tx_chan) | (1u << dma_rx_chan));
}

void MCP251863::dmaIrqHandler() {
    if (instance_ != nullptr) {
        dma_channel_acknowledge_irq0(instance_->dma_rx_chan);  // clear int flag
        instance_->finishTransfer();
    }
}

void MCP251863::finishTransfer() {
    while (spi_is_busy(spi_)) {
        tight_loop_contents();
    }
    gpio_put(chipSelectPin_, true);

    if (fifo_op_state == FifoOperationState::POPPING) {
        pending_fifo_uinc = true;
        memcpy(user_rx_buff, &rx_buff[2], transfer_len);
        user_frame.valid = true;
        fifo_op_state    = FifoOperationState::POP_DONE;
    }
    if (fifo_op_state == FifoOperationState::PUSHING) {
        pending_fifo_uinc = true;
        fifo_op_state     = FifoOperationState::PUSH_DONE;
    }
}

Error MCP251863::serviceFifoUinc() {
    MCP_TRY(updateByte(user_fifo_addr + 1, 0x00, 0b00000001));  // UINC
    pending_fifo_uinc = false;
    if (fifo_op_state == FifoOperationState::POP_DONE) {
        stats.frames_rx++;
    }
    return Error::None;
}

void MCP251863::abort() {
    if (dmaInitialized) {
        dma_channel_abort(dma_tx_chan);
        dma_channel_abort(dma_rx_chan);
    }
    gpio_put(chipSelectPin_, true);
    fifo_op_state     = FifoOperationState::IDLE;
    pending_fifo_uinc = false;
}

Error MCP251863::init() { return init(default_init_config()); }

Error MCP251863::init(const InitConfig& config) {
    if ((config.txFifoDepth < 1) || (config.txFifoDepth > 32) || (config.rxFifoDepth < 1) ||
        (config.rxFifoDepth > 32)) {
        return Error::InvalidFifoDepth;
    }
    if (fifo_plsize_to_len(config.txPayloadSize) == 0 ||
        fifo_plsize_to_len(config.rxPayloadSize) == 0) {
        return Error::InvalidPayloadSize;
    }
    MCP_TRY(validate_bit_timing(config.nominalBitTiming, config.dataBitTiming));

    // Remember the config so recover() can repeat this exact bring-up
    lastConfig = config;

    gpio_init(chipSelectPin_);
    gpio_put(chipSelectPin_, true);
    gpio_set_dir(chipSelectPin_, GPIO_OUT);
    gpio_init(standbyPin_);
    gpio_put(standbyPin_, std::to_underlying(TransceiverMode::TMODE_MCP_STBY));
    gpio_set_dir(standbyPin_, GPIO_OUT);

    (void)setTransceiverMode(TransceiverMode::TMODE_MCP_STBY);

    writeMode_ = WriteMode::WM_MCP_NORM;
    readMode_  = ReadMode::RM_MCP_NORM;

    abort();
    for (FifoInfo& f : fifoInfo) {
        f = FifoInfo{};
    }
    bus_off_latched = false;
    // bits are cleared manually or when TXREQ is set
    txlarb_latched = false;
    txerr_latched  = false;
    // could also expose: txbp, rxbp, txwarn, rxwarn, ewarn

    const Error err = configureDevice(config);
    if (err != Error::None) {
        (void)setTransceiverMode(TransceiverMode::TMODE_MCP_STBY);
        (void)reset();
        return err;
    }

    return Error::None;
}

Error MCP251863::configureDevice(const InitConfig& config) {
    uint32_t reg = 0;

    MCP_TRY(reset());
    sleep_ms(10);

    MCP_TRY(pollRegisterBit(std::to_underlying(RegisterAddress::REG_MCP_OSC) + 1, 1 << 2, true,
                            Error::OscillatorTimeout));

    MCP_TRY(setControllerMode(ControllerMode::CMODE_MCP_CONF));

    const uint8_t osc = (config.enablePll ? 0x01 : 0x00) | (config.sclkDiv2 ? 0x10 : 0x00);
    MCP_TRY(writeAddr(std::to_underlying(RegisterAddress::REG_MCP_OSC), &osc, 1));
    if (config.enablePll) {
        // PLLRDY
        MCP_TRY(pollRegisterBit(std::to_underlying(RegisterAddress::REG_MCP_OSC) + 1, 0x01, true,
                                Error::PllTimeout));
    }

    if (config.sclkDiv2) {
        // SCLKRDY
        MCP_TRY(pollRegisterBit(std::to_underlying(RegisterAddress::REG_MCP_OSC) + 1, 1 << 4, true,
                                Error::SclkdivTimeout));
    }

    MCP_TRY(setBitTiming(config.nominalBitTiming, config.dataBitTiming));

    MCP_TRY(writeReg(std::to_underlying(RegisterAddress::REG_MCP_C1TDC),
                     encode_tdc(config.enableTdc, config.tdcOffset)));

    rxTimestampsEnabled_ = config.rxTimestampEnable != 0;

    // FifoInterruptFlag txFlags[] = {FIFO_INT_MCP_NFNE, FIFO_INT_MCO_TXAT};
    // FifoInterruptFlag rxFlags[] = {FIFO_INT_MCP_NFNE, FIFO_INT_MCP_OVFL};
    // if (rxTimestampsEnabled_) {
    //     uint16_t rx_fifo_addr =
    //         std::to_underlying(RegisterAddress::REG_MCP_C1FIFOCONx) + 12 * (rxFifoNum_ - 1);
    //     MCP_TRY(updateByte(rx_fifo_addr, 0x00, 1 << 5));  // RXTSEN
    // }

    // clear interrupt flag registers
    reg = 0xFFFFFFFF;
    MCP_TRY(writeReg(std::to_underlying(RegisterAddress::REG_MCP_C1RXIF), reg));
    MCP_TRY(writeReg(std::to_underlying(RegisterAddress::REG_MCP_C1TXIF), reg));
    MCP_TRY(writeReg(std::to_underlying(RegisterAddress::REG_MCP_C1RXOVIF), reg));
    MCP_TRY(writeReg(std::to_underlying(RegisterAddress::REG_MCP_C1TXATIF), reg));
    InterruptEnable interrupts[] = {
        InterruptEnable::INT_EN_MCP_RXIE,    // RX
        InterruptEnable::INT_EN_MCP_TXIE,    // TX
        InterruptEnable::INT_EN_MCP_RXOVIE,  // RX overflow
        InterruptEnable::INT_EN_MCP_TXATIE,  // transmit attempt
        InterruptEnable::INT_EN_MCP_CERRIE,  // CAN bus error
        InterruptEnable::INT_EN_MCP_SERRIE,  // system error
    };
    MCP_TRY(setInterrupts(interrupts, sizeof(interrupts) / sizeof(interrupts[0])));
    MCP_TRY(initDMA());
    return setTransceiverMode(TransceiverMode::TMODE_MCP_NORM);
}

Error MCP251863::begin() {
    MCP_TRY(setControllerMode(ControllerMode::CMODE_MCP_CFD_NORM));
    return waitForOpMode(ControllerMode::CMODE_MCP_CFD_NORM);
}

Error MCP251863::recover() {
    const InitConfig cfg = lastConfig;
    const Error err      = init(cfg);
    if (err == Error::None) {
        stats.recoveries++;
    }
    return err;
}

Error MCP251863::setBitTiming(BitTiming nominalTiming, BitTiming dataTiming) {
    MCP_TRY(validate_bit_timing(nominalTiming, dataTiming));

    MCP_TRY(writeReg(std::to_underlying(RegisterAddress::REG_MCP_C1NBTCFG),
                     pack_nominal_bit_timing(nominalTiming)));

    return writeReg(std::to_underlying(RegisterAddress::REG_MCP_C1DBTCFG),
                    pack_data_bit_timing(dataTiming));
}

Error MCP251863::reset() {
    if (fifo_op_state != FifoOperationState::IDLE) {
        return Error::Busy;
    }

    const Command cmd  = Command::CMD_MCP_RESET;
    uint8_t message[2] = {0};

    message[0] = std::to_underlying(cmd) << 4;

    csSelect();
    spi_write_blocking(spi_, message, 2);
    csDeselect();

    return Error::None;
}

Error MCP251863::configureTxFifo(uint8_t fifoNum) {
    return configureTxFifo(fifoNum, 0);  // don't care about priority
}

Error MCP251863::configureTxFifo(uint8_t fifoNum, uint8_t prioNum) {
    FifoInterruptFlag txFlags[] = {FifoInterruptFlag::FIFO_INT_MCP_NFNE,
                                   FifoInterruptFlag::FIFO_INT_MCP_TXAT};
    MCP_TRY(initGeneralPurposeFifo(fifoNum, FifoMode::FIFO_MODE_MCP_TX, lastConfig.txPayloadSize,
                                   lastConfig.txFifoDepth,
                                   prioNum,  // prioNum
                                   TxRetransmitMode::TXRET_MCP_UNLIM, txFlags, 2));
    fifoInfo[fifoNum].configured = true;
    fifoInfo[fifoNum].isTx       = true;
    return Error::None;
}

Error MCP251863::configureRxFifo(uint8_t fifoNum, uint8_t fltNum, uint16_t canSID,
                                 uint16_t sidMask) {
    FifoInterruptFlag rxFlags[] = {FifoInterruptFlag::FIFO_INT_MCP_NFNE,
                                   FifoInterruptFlag::FIFO_INT_MCP_OVFL};
    MCP_TRY(initGeneralPurposeFifo(fifoNum, FifoMode::FIFO_MODE_MCP_RX, lastConfig.rxPayloadSize,
                                   lastConfig.rxFifoDepth,
                                   0,                                 // prioNum : don't care for RX
                                   TxRetransmitMode::TXRET_MCP_NONE,  // don't care
                                   rxFlags, 2));
    MCP_TRY(initFilter(fltNum, fifoNum, canSID, sidMask));
    fifoInfo[fifoNum].configured = true;
    fifoInfo[fifoNum].isTx       = false;
    fifoInfo[fifoNum].rxFltNum   = fltNum;
    return Error::None;
}

Error MCP251863::configureRxFifo(uint8_t fifoNum, uint16_t canSID) {
    return configureRxFifo(fifoNum, fifoNum, canSID);
}

Error MCP251863::initGeneralPurposeFifo(uint8_t fifoNum, FifoMode fifoMode, FifoPayloadSize plSize,
                                        uint8_t fSize, uint8_t prioNum, TxRetransmitMode retranMode,
                                        const FifoInterruptFlag* intFlagArray, size_t intFlagSize) {
    if (fifoNum > 30) {
        return Error::InvalidFifoNum;
    }
    if (fSize < 1 || fSize > 32) {
        return Error::InvalidFifoDepth;
    }
    if (prioNum > 31) {
        return Error::InvalidArgument;
    }
    const uint8_t len = fifo_plsize_to_len(plSize);
    if (len == 0) {
        return Error::InvalidPayloadSize;
    }
    if (intFlagSize > 0 && intFlagArray == nullptr) {
        return Error::NullPointer;
    }

    uint8_t buff[4];
    const uint16_t addr = std::to_underlying(RegisterAddress::REG_MCP_C1FIFOCONx) + 12 * fifoNum;

    uint8_t intFlags = 0;
    for (size_t i = 0; i < intFlagSize; i++) {
        intFlags |= static_cast<uint8_t>(intFlagArray[i]);
    }

    buff[0] = intFlags | (std::to_underlying(fifoMode) << 7);
    buff[1] = 0b00000000;
    buff[2] = 0b00000000 | (std::to_underlying(retranMode) << 5) | prioNum;
    // FSIZE stores depth-1 (0 = 1 message; 31 = 32 messages); caller passes 1..32
    buff[3] = ((std::to_underlying(plSize) & 0b111) << 5) | ((fSize - 1) & 0x1F);

    MCP_TRY(writeAddr(addr, buff, 4));
    return Error::None;
}

Error MCP251863::disableFifo(uint8_t fifoNum) {
    if (fifoNum > 30) {
        return Error::InvalidFifoNum;
    }
    if (!fifoInfo[fifoNum].configured) {
        return Error::None;
    }
    const uint16_t addr = std::to_underlying(RegisterAddress::REG_MCP_C1FIFOCONx) + 12 * fifoNum;
    // set FRESET bit to reset FIFO
    MCP_TRY(updateByte(addr + 1, 0x00, 0b00000100));
    // disable Filter attached to FIFO if FIFO is RX
    if (!fifoInfo[fifoNum].isTx) {
        const uint8_t flt_num   = fifoInfo[fifoNum].rxFltNum;
        const uint16_t flt_addr = std::to_underlying(RegisterAddress::REG_MCP_C1FLTCONx) + flt_num;
        const uint16_t flt_obj_addr =
            std::to_underlying(RegisterAddress::REG_MCP_C1FLTOBJx) + 8 * flt_num;

        uint8_t buff[4];

        // Disable Filter
        buff[0] = 0x00;
        MCP_TRY(writeAddr(flt_addr, buff, 1));

        // Reset Filter ID
        buff[0] = 0x00;
        buff[1] = 0x00;
        buff[2] = 0x00;
        buff[3] = 0x00;
        MCP_TRY(writeAddr(flt_obj_addr, buff, 4));
    }

    fifoInfo[fifoNum].configured = false;
    return Error::None;
}

Error MCP251863::initTransmitEventFifo(uint8_t fSize, const FifoInterruptFlag* intFlagArray,
                                       size_t intFlagSize) {
    if (fSize < 1 || fSize > 32) {
        return Error::InvalidFifoDepth;
    }
    if (intFlagSize > 0 && intFlagArray == nullptr) {
        return Error::NullPointer;
    }

    uint8_t buff[4];
    const uint16_t addr = std::to_underlying(RegisterAddress::REG_MCP_C1TEFCON);

    uint8_t intFlags = 0;
    for (size_t i = 0; i < intFlagSize; i++) {
        intFlags |= static_cast<uint8_t>(intFlagArray[i]);
    }

    buff[0] = 0b00000000 | intFlags;
    buff[1] = 0b00000000;
    buff[2] = 0b00000000;
    buff[3] = (fSize - 1) & 0x1F;

    return writeAddr(addr, buff, 4);
}

Error MCP251863::initTransmitQueue(FifoPayloadSize plSize, uint8_t fSize, uint8_t prioNum,
                                   TxRetransmitMode retranMode,
                                   const FifoInterruptFlag* intFlagArray, size_t intFlagSize) {
    if (fSize < 1 || fSize > 32) {
        return Error::InvalidFifoDepth;
    }
    if (prioNum > 31) {
        return Error::InvalidArgument;
    }
    if (fifo_plsize_to_len(plSize) == 0) {
        return Error::InvalidPayloadSize;
    }
    if (intFlagSize > 0 && intFlagArray == nullptr) {
        return Error::NullPointer;
    }

    uint8_t buff[4];
    const uint16_t addr = std::to_underlying(RegisterAddress::REG_MCP_C1TXQCON);

    uint8_t intFlags = 0;
    for (size_t i = 0; i < intFlagSize; i++) {
        intFlags |= static_cast<uint8_t>(intFlagArray[i]);
    }

    MCP_TRY(pollRegisterBit(addr + 1, 0x04, false, Error::PollTimeout));

    buff[0] = 0b00000000 | intFlags;
    buff[1] = 0b00000000;
    buff[2] = 0b00000000 | (std::to_underlying(retranMode) << 5) | prioNum;
    buff[3] = ((std::to_underlying(plSize) & 0b111) << 5) | ((fSize - 1) & 0x1F);

    return writeAddr(addr, buff, 4);
}

Error MCP251863::initFilter(uint8_t fltNum, uint8_t fifoNum, uint16_t canSID, uint16_t sidMask) {
    if (fltNum > 31) {
        return Error::InvalidFilterNum;
    }
    if (canSID > 0x7FF || sidMask > 0x7FF) {
        return Error::IdOutOfRange;
    }

    const uint16_t flt_addr = std::to_underlying(RegisterAddress::REG_MCP_C1FLTCONx) + fltNum;
    const uint16_t flt_obj_addr =
        std::to_underlying(RegisterAddress::REG_MCP_C1FLTOBJx) + 8 * fltNum;
    const uint16_t flt_mask_addr =
        std::to_underlying(RegisterAddress::REG_MCP_C1MASKx) + 8 * fltNum;

    uint8_t buff[4];

    // CiFLTOBJm can only be modified while the filter is disabled
    buff[0] = 0x00;
    MCP_TRY(writeAddr(flt_addr, buff, 1));

    // Supports standard ID only
    buff[0] = canSID & 0xFF;
    buff[1] = (canSID >> 8) & 0x0F;
    buff[2] = 0x00;
    buff[3] = 0x00;
    MCP_TRY(writeAddr(flt_obj_addr, buff, 4));

    // Set mask (only the masked ID bits must match)
    buff[0] = sidMask & 0xFF;         // MSID<7:0>
    buff[1] = (sidMask >> 8) & 0x07;  // MSID<10:8>
    buff[2] = 0x00;                   // MEID = don't care
    buff[3] = 0b01000000;             // MIDE = 1; MSID11 = 0 (don't care)
    MCP_TRY(writeAddr(flt_mask_addr, buff, 4));

    // Re-enable
    buff[0] = 0b00000000 | (fifoNum + 1) | (1 << 7);
    return writeAddr(flt_addr, buff, 1);
}

Error MCP251863::start_push_canfd(uint8_t fifoNum, uint32_t id, const uint8_t* data, size_t len,
                                  bool brs, bool extended_id) {
    if (len > MCP251863_MAX_PAYLOAD) {
        return Error::InvalidPayloadSize;
    }
    auto dlc_opt = canfd_len_to_dlc(len);
    if (!dlc_opt) {
        return Error::InvalidPayloadSize;
    }
    if ((len > 0) && (data == nullptr)) {
        return Error::NullPointer;
    }
    CanFdFrame frame{};
    frame.id  = id;
    frame.ide = extended_id;
    frame.fdf = true;
    frame.brs = brs;
    frame.len = len;
    frame.dlc = std::to_underlying(*dlc_opt);
    for (size_t i = 0; i < len; i++) {
        frame.data[i] = data[i];
    }
    return start_push_frame(fifoNum, frame);
}

Error MCP251863::start_push_frame(uint8_t fifoNum, const CanFdFrame& frame) {
    MCP_TRY(checkFifo(fifoNum, true));
    // 8-byte header + up to 64 payload bytes, word padded => at most 72.
    uint8_t message[72];
    size_t objectSize = 0;
    MCP_TRY(create_message_obj(message, frame, &objectSize));
    if (fifo_op_state != FifoOperationState::IDLE) {
        return Error::Busy;
    }
    const uint16_t fifo_stat_addr =
        std::to_underlying(RegisterAddress::REG_MCP_C1FIFOSTAx) + 12 * fifoNum;
    const uint16_t fifo_point_addr =
        std::to_underlying(RegisterAddress::REG_MCP_C1FIFOUAx) + 12 * fifoNum;
    user_fifo_addr = std::to_underlying(RegisterAddress::REG_MCP_C1FIFOCONx) + 12 * fifoNum;
    uint8_t stat   = 0;
    MCP_TRY(readAddr(fifo_stat_addr, &stat, 1));
    if ((stat & 0b00000001) == 0) {
        stats.tx_fifo_full_events++;
        return Error::TxFifoFull;
    }
    uint16_t message_addr = 0;
    MCP_TRY(readFifoUserAddress(fifo_point_addr, objectSize, &message_addr));
    fifo_op_state   = FifoOperationState::PUSHING;
    const Error err = dmaWriteAddr(message_addr, message, objectSize);
    if (err != Error::None) {
        fifo_op_state = FifoOperationState::IDLE;
    }
    return err;
}

Error MCP251863::poll_push() {
    // read once so the dma irq can't change it between the two checks
    const FifoOperationState state = fifo_op_state;
    if (state == FifoOperationState::PUSH_DONE) {
        if (pending_fifo_uinc) {
            MCP_TRY(serviceFifoUinc());
        }
        return Error::None;
    }
    if (state == FifoOperationState::PUSHING) {
        return Error::Busy;
    }
    return Error::NoOperationPending;
}

Error MCP251863::request_send(uint8_t fifoNum) {
    MCP_TRY(checkFifo(fifoNum, true));
    const uint16_t addr = std::to_underlying(RegisterAddress::REG_MCP_C1FIFOCONx) + 12 * fifoNum;
    MCP_TRY(pollRegisterBit(addr + 1, 0b00000001, false, Error::PollTimeout));
    MCP_TRY(updateByte(addr + 1, 0x00, 0b00000010));  // TXREQ
    stats.frames_tx_requested++;
    return Error::None;
}

Error MCP251863::clear_send() {
    if (fifo_op_state == FifoOperationState::PUSHING) {
        return Error::Busy;
    }
    if (pending_fifo_uinc) {
        MCP_TRY(serviceFifoUinc());
    }
    fifo_op_state = FifoOperationState::IDLE;
    return Error::None;
}

Error MCP251863::start_pop_canfd(uint8_t fifoNum) {
    MCP_TRY(checkFifo(fifoNum, false));
    if (fifo_op_state != FifoOperationState::IDLE) {
        return Error::Busy;
    }
    user_fifo_addr = std::to_underlying(RegisterAddress::REG_MCP_C1FIFOCONx) + 12 * fifoNum;
    const uint16_t fifo_stat_addr =
        std::to_underlying(RegisterAddress::REG_MCP_C1FIFOSTAx) + 12 * fifoNum;
    const uint16_t fifo_point_addr =
        std::to_underlying(RegisterAddress::REG_MCP_C1FIFOUAx) + 12 * fifoNum;
    uint8_t buff            = 0;
    uint16_t message_addr   = 0;
    uint8_t header[12]      = {0};
    const size_t headerSize = rxTimestampsEnabled_ ? 12 : 8;
    MCP_TRY(readAddr(fifo_stat_addr, &buff, 1));
    if ((buff & 0b00000001) == 0) {
        return Error::RxFifoEmpty;
    }
    MCP_TRY(readFifoUserAddress(fifo_point_addr, headerSize, &message_addr));
    MCP_TRY(readAddr(message_addr, header, headerSize));
    user_frame = decode_rx_header(header, rxTimestampsEnabled_);
    if (user_frame.len > 0) {
        fifo_op_state   = FifoOperationState::POPPING;
        const Error err = dmaReadAddr(message_addr + headerSize, user_frame.data, user_frame.len);
        if (err != Error::None) {
            fifo_op_state = FifoOperationState::IDLE;
        }
        return err;
    }
    user_frame.valid  = true;
    pending_fifo_uinc = true;
    fifo_op_state     = FifoOperationState::POP_DONE;
    return Error::None;
}

Error MCP251863::poll_pop() {
    // read once so the dma irq can't change it between the two checks
    const FifoOperationState state = fifo_op_state;
    if (state == FifoOperationState::POP_DONE) {
        if (pending_fifo_uinc) {
            MCP_TRY(serviceFifoUinc());
        }
        return Error::None;
    }
    if (state == FifoOperationState::POPPING) {
        return Error::Busy;
    }
    return Error::NoOperationPending;
}

Result<CanFdFrame> MCP251863::get_read_frame() {
    MCP_TRY(poll_pop());
    fifo_op_state = FifoOperationState::IDLE;
    return user_frame;
}

Result<FifoStatus> MCP251863::getFIFOStatus(uint8_t fifoNum) {
    FifoStatus status{};

    const uint16_t fifo_stat_addr =
        std::to_underlying(RegisterAddress::REG_MCP_C1FIFOSTAx) + 12 * fifoNum;
    uint32_t reg = 0;
    MCP_TRY(readReg(fifo_stat_addr, &reg));

    status.fifo_num                = (reg >> 8) & 0x1F;
    status.tx_aborted              = (reg & (1UL << 7)) != 0;
    status.tx_lost_arbitration     = (reg & (1UL << 6)) != 0;
    status.tx_error                = (reg & (1UL << 5)) != 0;
    status.tx_attempts_exhausted   = (reg & (1UL << 4)) != 0;
    status.rx_overflow             = (reg & (1UL << 3)) != 0;
    status.empty_or_full           = (reg & (1UL << 2)) != 0;
    status.half_empty_or_half_full = (reg & (1UL << 1)) != 0;
    status.not_full_or_not_empty   = (reg & 1UL) != 0;

    return status;
}

Error MCP251863::clearRxOverflow(uint8_t fifoNum) {
    MCP_TRY(checkFifo(fifoNum, false));
    // writing zero clears RXOVIF, the read-only flags in this byte ignore the write
    const uint8_t zero = 0;
    return writeAddr(std::to_underlying(RegisterAddress::REG_MCP_C1FIFOSTAx) + 12 * fifoNum, &zero,
                     1);
}

Result<Status> MCP251863::getStatus() {
    Status status{};

    MCP_TRY(readReg(std::to_underlying(RegisterAddress::REG_MCP_C1INT), &status.interrupt_flags));
    MCP_TRY(readReg(std::to_underlying(RegisterAddress::REG_MCP_C1RXIF), &status.rx_if));
    MCP_TRY(readReg(std::to_underlying(RegisterAddress::REG_MCP_C1TXIF), &status.tx_if));
    MCP_TRY(readReg(std::to_underlying(RegisterAddress::REG_MCP_C1RXOVIF), &status.rx_overflow_if));
    MCP_TRY(readReg(std::to_underlying(RegisterAddress::REG_MCP_C1TXATIF), &status.tx_attempt_if));
    MCP_TRY(readReg(std::to_underlying(RegisterAddress::REG_MCP_C1TREC), &status.trec));
    MCP_TRY(readReg(std::to_underlying(RegisterAddress::REG_MCP_C1BDIAGx), &status.bdiag0));
    MCP_TRY(readReg(std::to_underlying(RegisterAddress::REG_MCP_C1BDIAGx) + 4, &status.bdiag1));
    MCP_TRY(readReg(std::to_underlying(RegisterAddress::REG_MCP_CRC), &status.crc));

    status.bus_off          = (status.trec & (1UL << 21)) != 0;
    status.tx_error_passive = (status.trec & (1UL << 20)) != 0;
    status.rx_error_passive = (status.trec & (1UL << 19)) != 0;
    status.tx_error_warning = (status.trec & (1UL << 18)) != 0;
    status.rx_error_warning = (status.trec & (1UL << 17)) != 0;
    status.error_warning    = (status.trec & (1UL << 16)) != 0;
    status.tx_error_count   = (status.trec >> 8) & 0xFF;
    status.rx_error_count   = status.trec & 0xFF;

    return status;
}

Error MCP251863::setControllerMode(ControllerMode contMode) {
    const uint16_t addr = std::to_underlying(RegisterAddress::REG_MCP_C1CON);
    uint8_t current     = 0;
    MCP_TRY(readOpMode(&current));
    if (current != std::to_underlying(ControllerMode::CMODE_MCP_CONF)) {
        MCP_TRY(
            updateByte(addr + 3, 0b00000111, std::to_underlying(ControllerMode::CMODE_MCP_CONF)));
        MCP_TRY(waitForOpMode(ControllerMode::CMODE_MCP_CONF, 1000, 100));
    }
    return updateByte(addr + 3, 0b00000111, std::to_underlying(contMode));
}

Error MCP251863::setTransceiverMode(TransceiverMode mode) const {
    gpio_put(standbyPin_, std::to_underlying(mode));
    return Error::None;
}

Error MCP251863::setInterrupts(const InterruptEnable* intEnArray, size_t intEnSize) {
    if (intEnSize > 32) {
        return Error::InvalidArgument;
    }
    if (intEnSize > 0 && intEnArray == nullptr) {
        return Error::NullPointer;
    }
    uint32_t message = 0;
    for (size_t i = 0; i < intEnSize; i++) {
        message |= static_cast<uint32_t>(intEnArray[i]);
    }
    return writeReg(std::to_underlying(RegisterAddress::REG_MCP_C1INT), message);
}

void MCP251863::gpioIrqHandler(uint gpio, uint32_t /*events*/) {
    if (gpio == instance_->intPin_) {
        instance_->error_flag_pending = true;
    } else if (gpio == instance_->int0Pin_) {
        instance_->tx_flag_pending = true;
    } else if (gpio == instance_->int1Pin_) {
        instance_->rx_flag_pending = true;
    }
}

// system error, CAN bus error
Error MCP251863::enableErrorInterrupts() const {
    gpio_init(intPin_);
    gpio_set_dir(intPin_, GPIO_IN);
    gpio_pull_up(intPin_);
    gpio_set_irq_enabled_with_callback(intPin_, GPIO_IRQ_EDGE_FALL, true,
                                       &MCP251863::gpioIrqHandler);
    return Error::None;
}

// TXAT, Not Full
Error MCP251863::enableTxInterrupts() {
    MCP_TRY(setPinMode(IoPin::IO_MCP_INT0, IoMode::IOMODE_MCP_INT));
    gpio_init(int0Pin_);
    gpio_set_dir(int0Pin_, GPIO_IN);
    gpio_pull_up(int0Pin_);
    gpio_set_irq_enabled_with_callback(int0Pin_, GPIO_IRQ_EDGE_FALL, true,
                                       &MCP251863::gpioIrqHandler);
    return Error::None;
}

// Overflow event, not empty
Error MCP251863::enableRxInterrupts() {
    MCP_TRY(setPinMode(IoPin::IO_MCP_INT1, IoMode::IOMODE_MCP_INT));
    gpio_init(int1Pin_);
    gpio_set_dir(int1Pin_, GPIO_IN);
    gpio_pull_up(int1Pin_);
    gpio_set_irq_enabled_with_callback(int1Pin_, GPIO_IRQ_EDGE_FALL, true,
                                       &MCP251863::gpioIrqHandler);
    return Error::None;
}

Error MCP251863::setPinMode(IoPin pin, IoMode mode) {
    uint8_t buff[4] = {0};
    MCP_TRY(readAddr(std::to_underlying(RegisterAddress::REG_MCP_IOCON), buff, 4));
    if (pin == IoPin::IO_MCP_INT0) {
        switch (mode) {
            case IoMode::IOMODE_MCP_GPIO_IN:
                buff[0] |= 0b00000001;
                buff[3] |= 0b00000001;
                break;
            case IoMode::IOMODE_MCP_GPIO_OUT:
                buff[0] &= 0b11111110;
                buff[3] |= 0b00000001;
                break;
            case IoMode::IOMODE_MCP_INT: buff[3] &= 0b11111110; break;
            default: return Error::InvalidArgument;
        }
    } else if (pin == IoPin::IO_MCP_INT1) {
        switch (mode) {
            case IoMode::IOMODE_MCP_GPIO_IN:
                buff[0] |= 0b00000010;
                buff[3] |= 0b00000010;
                break;
            case IoMode::IOMODE_MCP_GPIO_OUT:
                buff[0] &= 0b11111101;
                buff[3] |= 0b00000010;
                break;
            case IoMode::IOMODE_MCP_INT: buff[3] &= 0b11111101; break;
            default: return Error::InvalidArgument;
        }
    } else {
        return Error::InvalidArgument;
    }
    return writeAddr(std::to_underlying(RegisterAddress::REG_MCP_IOCON), buff, 4);
}

// C1VEC layout
// 31:24 rxcode (third byte)
// 23:16 txcode (second byte)
// 12:8 filhit (first byte, + some offset stuff)
// 7:0 icode (zeroth byte)

Result<int> MCP251863::getTXCode() {
    uint8_t buff = 0;
    MCP_TRY(readAddr(std::to_underlying(RegisterAddress::REG_MCP_C1VEC) + 2, &buff, 1));
    return static_cast<int>(buff & 0x7F);
}

Result<int> MCP251863::getRXCode() {
    uint8_t buff = 0;
    MCP_TRY(readAddr(std::to_underlying(RegisterAddress::REG_MCP_C1VEC) + 3, &buff, 1));
    return static_cast<int>(buff & 0x7F);
}

Result<int> MCP251863::getFLTCode() {
    // its 5 bits so we mask with 0x1F instead of 0x7F
    uint8_t buff = 0;
    MCP_TRY(readAddr(std::to_underlying(RegisterAddress::REG_MCP_C1VEC) + 1, &buff, 1));
    return static_cast<int>(buff & 0x1F);
}

Result<int> MCP251863::getICode() {
    uint8_t buff = 0;
    MCP_TRY(readAddr(std::to_underlying(RegisterAddress::REG_MCP_C1VEC), &buff, 1));
    buff &= 0x7F;
    if (buff > 0b1001010) {  // max valid ICODE value
        return Error::BadRegisterValue;
    }
    return static_cast<int>(buff);
}