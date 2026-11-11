#include "mcp251863.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <utility>

// a macro, since it returns from the function that uses it
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define MCP_TRY(expr) do { if (!(expr)) return 0; } while (0)

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

uint32_t pack_nominal_bit_timing(BitTiming timing) {
    return (((uint32_t)timing.brp & 0xFF) << 24) | (((uint32_t)timing.tseg1 & 0xFF) << 16) |
           (((uint32_t)timing.tseg2 & 0x7F) << 8) | ((uint32_t)timing.sjw & 0x7F);
}

uint32_t pack_data_bit_timing(BitTiming timing) {
    return (((uint32_t)timing.brp & 0xFF) << 24) | (((uint32_t)timing.tseg1 & 0x1F) << 16) |
           (((uint32_t)timing.tseg2 & 0x0F) << 8) | ((uint32_t)timing.sjw & 0x0F);
}

int validate_bit_timing(const BitTiming& nominal, const BitTiming& data) {
    if (nominal.tseg2 > 0x7F || nominal.sjw > 0x7F) { return 0; }
    if (data.tseg1 > 0x1F || data.tseg2 > 0x0F || data.sjw > 0x0F) { return 0; }
    return 1;
}

uint32_t encode_tdc(bool enable, uint8_t offset) {
    if (!enable) { return 0; }
    // 10 at bits 17:16 for TDCMOD auto
    // TDCO at bits 13:8
    return (2UL << 16) | ((offset & 0x3F) << 8);
}

InitConfig default_init_config() {
    InitConfig config{};

    config.enablePll         = 0;
    config.sclkDiv2          = 0;
    config.enableTdc         = 1;
    config.rxTimestampEnable = 0;
    // (DTSEG1 + 1) * (DBRP + 1) tq, microchip's value for a 40 MHz clock at 2 Mbit/s
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
    if (frame.ide) { return ((frame.id >> 18) & 0x7FF) | ((frame.id & 0x3FFFF) << 11); }
    return frame.id & 0x7FF;
}

uint32_t pack_control_word(const CanFdFrame& frame) {
    return ((uint32_t)frame.dlc & 0x0F) | (frame.ide ? (1UL << 4) : 0) | (frame.brs ? (1UL << 6) : 0) |
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

int finalize_frame_dlc(CanFdFrame* frame) {
    auto dlc_opt = canfd_len_to_dlc(frame->len);
    if (!dlc_opt) { return 0; }
    frame->dlc = std::to_underlying(*dlc_opt);
    return 1;
}

int validate_tx_frame(const CanFdFrame& frame) {
    if (!frame.fdf) { return 0; } // CAN not supported
    if (frame.ide && (frame.id > 0x1FFFFFFF)) { return 0; }
    if (!frame.ide && (frame.id > 0x7FF)) { return 0; }
    return 1;
}

CanFdFrame decode_rx_header(const uint8_t* header, bool timestampEnabled) {
    CanFdFrame frame{};
    const uint32_t word0  = load_word(header);
    const uint32_t word1  = load_word(header + 4);
    frame.dlc             = word1 & 0x0F;
    frame.ide             = (word1 & (1UL << 4)) != 0;
    frame.brs             = (word1 & (1UL << 6)) != 0;
    frame.fdf             = (word1 & (1UL << 7)) != 0;
    frame.esi             = (word1 & (1UL << 8)) != 0;
    frame.filter_hit      = (word1 >> 11) & 0x1F;
    frame.len             = canfd_dlc_to_len(frame.dlc);
    frame.timestamp_valid = timestampEnabled;
    if (timestampEnabled) {
        frame.timestamp = load_word(header + 8);
    }
    if (frame.ide) {
        frame.id = ((word0 & 0x7FF) << 18) | ((word0 >> 11) & 0x3FFFF);
    } else {
        frame.id = (word0 & 0x7FF);
    }
    return frame;
}

int create_message_obj(uint8_t* dst, const CanFdFrame& frame, size_t* objectSize) {
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
    *objectSize = objSize;
    return 1;
}

uint16_t mcp_crc16(const uint8_t *data, size_t len, uint16_t crc = 0xFFFF) {
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8; 
        for (int b = 0; b < 8; b++) 
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x8005 : (crc << 1);
    }
    return crc;
}

int crc_length_field(uint16_t addr, size_t len, uint8_t* n) {
    bool isRam = addr >= 0x400 && addr < 0xC00;
    if (isRam && len % 4 != 0) { return 0; }
    size_t count = isRam ? len / 4 : len;
    if (count > 255) { return 0; }
    *n = static_cast<uint8_t>(count);
    return 1;
}

uint16_t fifo_con_addr(uint8_t fifoNum) {
    return std::to_underlying(RegisterAddress::REG_MCP_C1FIFOCONx) + 12 * fifoNum;
}
uint16_t fifo_sta_addr(uint8_t fifoNum) {
    return std::to_underlying(RegisterAddress::REG_MCP_C1FIFOSTAx) + 12 * fifoNum;
}
uint16_t fifo_ua_addr(uint8_t fifoNum) {
    return std::to_underlying(RegisterAddress::REG_MCP_C1FIFOUAx) + 12 * fifoNum;
}

BusErrorState decode_trec(uint32_t trec) {
    BusErrorState e{};
    e.bus_off          = (trec & (1UL << 21)) != 0;  
    e.tx_error_passive = (trec & (1UL << 20)) != 0;  
    e.rx_error_passive = (trec & (1UL << 19)) != 0;  
    e.tx_error_warning = (trec & (1UL << 18)) != 0;  
    e.rx_error_warning = (trec & (1UL << 17)) != 0;  
    e.error_warning    = (trec & (1UL << 16)) != 0; 
    e.tx_error_count   = (trec >> 8) & 0xFF;         
    e.rx_error_count   = trec & 0xFF;             
    return e;
}

}  // namespace

MCP251863::MCP251863(spi_inst_t* ispi, uint iCSPin, uint iSTBYPin, uint iINTPin)
    : spi_(ispi),
      chipSelectPin_(iCSPin),
      standbyPin_(iSTBYPin),
      intPin_(iINTPin) { }

// Checks if a FIFO was actually configured and in the direction expected
int MCP251863::checkFifo(uint8_t fifoNum, bool wantTx) const {
    if (fifoNum > 30) { return 0; }
    const FifoInfo& info = fifoInfo[fifoNum];
    if (!info.configured || info.isTx != wantTx) { return 0; }
    return 1;
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

int MCP251863::writeAddr(uint16_t startAddr, const uint8_t* data, size_t len) {
    MCP_TRY(writeAddrUnchecked(startAddr, data, len));
    if (writeMode_ != WriteMode::WM_MCP_NORM) { return crcCheckPassed(); }
    return 1;
}

int MCP251863::writeAddrUnchecked(uint16_t startAddr, const uint8_t* data, size_t len) {
    if (data == nullptr && len > 0) { return 0; }

    // form message CCCC-AAAAAAAAAAAA
    uint8_t message[3];
    message[1] = startAddr & 0xFF;

    if (writeMode_ == WriteMode::WM_MCP_NORM) {

        message[0] = (std::to_underlying(Command::CMD_MCP_WRITA) << 4) | 
                                        (startAddr >> 8);
        
        csSelect();
        spi_write_blocking(spi_, message, 2);
        spi_write_blocking(spi_, data, len);
        csDeselect();

        return 1;
    }
    else if (writeMode_ == WriteMode::WM_MCP_CRC) {

        message[0] = (std::to_underlying(Command::CMD_MCP_WRACR) << 4) | 
                                        (startAddr >> 8);

        MCP_TRY(crc_length_field(startAddr, len, &message[2]));
        
        uint16_t crc = mcp_crc16(message, 3); 
        crc = mcp_crc16(data, len, crc);
        uint8_t crcBytes[2] = {static_cast<uint8_t>(crc >> 8), static_cast<uint8_t>(crc)};

        csSelect();
        spi_write_blocking(spi_, message, 3);
        spi_write_blocking(spi_, data, len);
        spi_write_blocking(spi_, crcBytes, 2);
        csDeselect(); 

        return 1;
    } else { return 0; } // WRITE_SAFE is unimplemented for now
}

int MCP251863::readAddr(uint16_t startAddr, uint8_t* dst, size_t len) {
    if (dst == nullptr && len > 0) { return 0; }

    uint8_t message[3];
    message[1] = startAddr & 0xFF;

    if (readMode_ == ReadMode::RM_MCP_NORM) {

        message[0] = (std::to_underlying(Command::CMD_MCP_READA) << 4) | 
                                        (startAddr >> 8);

        csSelect();
        spi_write_blocking(spi_, message, 2);
        spi_read_blocking(spi_, 0, dst, len);
        csDeselect();

        return 1;

    } else if (readMode_ == ReadMode::RM_MCP_CRC) {

        message[0] = (std::to_underlying(Command::CMD_MCP_RDACR) << 4) | 
                                        (startAddr >> 8);
    
        MCP_TRY(crc_length_field(startAddr, len, &message[2]));

        uint8_t  crcBytes[2];
        uint16_t headerCrc = mcp_crc16(message, 3);

        for (int i = 0; i < 3; i++) {
            csSelect();
            spi_write_blocking(spi_, message, 3);
            spi_read_blocking(spi_, 0, dst, len); 
            spi_read_blocking(spi_, 0, crcBytes, 2);
            csDeselect();

            uint16_t crc = mcp_crc16(dst, len, headerCrc);
            if (crc == ((crcBytes[0] << 8) | crcBytes[1])) {
                return 1;
            }
        }
        spiCrcErrors_++;
        return 0;

    } else { return 0; }
}

bool MCP251863::crcCheckPassed() {
    uint8_t buff = 0;
    uint16_t crc_addr = std::to_underlying(RegisterAddress::REG_MCP_CRC);
    MCP_TRY(readAddr(crc_addr + 2, &buff, 1));
    if ((buff & 0b00000011) != 0) { // FERRIF, CRCERRIF
        spiCrcErrors_++;
        uint8_t zero = 0;
        (void)writeAddrUnchecked(crc_addr + 2, &zero, 1);
        return false;
    }
    return true;
}

bool MCP251863::eccCheckPassed() {
    uint8_t buff = 0;
    uint16_t ecc_addr = std::to_underlying(RegisterAddress::REG_MCP_ECCSTAT);
    MCP_TRY(readAddr(ecc_addr, &buff, 1));
    if ((buff & 0b00000100) != 0) {  // DEDIF
        eccDoubleErrors_++;
        const uint8_t zero = 0;  
        (void)writeAddr(ecc_addr, &zero, 1);
        return false;
    }
    return true;
}

// helper functions for reading/writing to 4 byte registers
int MCP251863::readReg(uint16_t addr, uint32_t* dst) {
    uint8_t buf[4] = {0};
    MCP_TRY(readAddr(addr, buf, 4));
    *dst = load_word(buf);
    return 1;
}

int MCP251863::writeReg(uint16_t addr, uint32_t value) {
    uint8_t buff[4];
    store_word(buff, value);
    return writeAddr(addr, buff, 4);
}

// R-M-W
int MCP251863::updateByte(uint16_t addr, uint8_t field, uint8_t value) {
    uint8_t buff = 0;
    MCP_TRY(readAddr(addr, &buff, 1));
    buff = (uint8_t)((buff & (uint8_t)~field) | value);
    return writeAddr(addr, &buff, 1);
}

int MCP251863::pollRegisterBit(uint16_t addr, uint8_t mask, bool wantSet) {
    for (int i = 0; i < 100; i++) {
        uint8_t buff = 0;
        MCP_TRY(readAddr(addr, &buff, 1));
        if (((buff & mask) != 0) == wantSet) { return 1; }
        sleep_us(1000);
    }
    return 0;
}

int MCP251863::readOpMode(uint8_t* opmod) {
    uint8_t buff = 0;
    MCP_TRY(readAddr(std::to_underlying(RegisterAddress::REG_MCP_C1CON) + 2, &buff, 1));
    *opmod = buff >> 5;
    return 1;
}

int MCP251863::waitForOpMode(ControllerMode target) {
    for (int i = 0; i < 100; i++) {
        uint8_t mode = 0;
        MCP_TRY(readOpMode(&mode));
        if (mode == std::to_underlying(target)) { return 1; }
        sleep_us(1000);
    }
    return 0;
}

int MCP251863::init() { return init(default_init_config()); }

int MCP251863::init(const InitConfig& config) {
    if ((config.txFifoDepth < 1) || (config.txFifoDepth > 32) || (config.rxFifoDepth < 1) ||
        (config.rxFifoDepth > 32)) {
        return 0;
    }
    if (fifo_plsize_to_len(config.txPayloadSize) == 0 ||
        fifo_plsize_to_len(config.rxPayloadSize) == 0) {
        return 0;
    }
    MCP_TRY(validate_bit_timing(config.nominalBitTiming, config.dataBitTiming));

    // unsupported for now
    if (config.rxTimestampEnable) { return 0; }
    if (config.enablePll || config.sclkDiv2) { return 0; }

    config_ = config;

    gpio_init(chipSelectPin_);
    gpio_put(chipSelectPin_, true);
    gpio_set_dir(chipSelectPin_, GPIO_OUT);
    gpio_init(standbyPin_);
    gpio_put(standbyPin_, std::to_underlying(TransceiverMode::TMODE_MCP_STBY));
    gpio_set_dir(standbyPin_, GPIO_OUT);
    gpio_init(intPin_);
    gpio_set_dir(intPin_, GPIO_IN);
    gpio_pull_up(intPin_);

    writeMode_ = WriteMode::WM_MCP_CRC;
    readMode_  = ReadMode::RM_MCP_CRC;

    if (!configureDevice(config)) {
        (void)reset();
        state_ = DriverState::Uninitialized;
        return 0;
    }

    return 1;
}

int MCP251863::configureDevice(const InitConfig& config) {

    // set ABAT: abort all pending transmissions
    (void)updateByte(std::to_underlying(RegisterAddress::REG_MCP_C1CON) + 3, 1 << 3, 1 << 3);
    (void)setControllerMode(ControllerMode::CMODE_MCP_CONF);

    MCP_TRY(reset());
    sleep_ms(10);

    // OSCRDY
    MCP_TRY(pollRegisterBit(std::to_underlying(RegisterAddress::REG_MCP_OSC) + 1, 1 << 2, true));

    // turn off the TXQ and TEF since we're not using them to save RAM
    MCP_TRY(updateByte(std::to_underlying(RegisterAddress::REG_MCP_C1CON) + 2, 0x18, 0x00));

    MCP_TRY(setBitTiming(config.nominalBitTiming, config.dataBitTiming));

    // read the bit timing back to prove MCP answered / isn't disconnected
    uint32_t nbtcfg = 0;
    MCP_TRY(readReg(std::to_underlying(RegisterAddress::REG_MCP_C1NBTCFG), &nbtcfg));
    if (nbtcfg != pack_nominal_bit_timing(config.nominalBitTiming)) { return 0; }

    MCP_TRY(writeReg(std::to_underlying(RegisterAddress::REG_MCP_C1TDC),
                     encode_tdc(config.enableTdc, config.tdcOffset)));

    uint8_t eccon = 0x01;
    MCP_TRY(writeAddr(std::to_underlying(RegisterAddress::REG_MCP_ECCCON), &eccon, 1)); // enable ECC

    // clear RAM after enabling ECC
    const uint8_t zeros[64] = {0};
    for (uint16_t addr = 0x400; addr < 0xC00; addr += sizeof(zeros)) {
        MCP_TRY(writeAddr(addr, zeros, sizeof(zeros)));
    }

    // INT pin goes low during a general RX and RX overflow event
    const uint32_t interrupts = std::to_underlying(InterruptEnable::INT_EN_MCP_RXIE) |   
                                std::to_underlying(InterruptEnable::INT_EN_MCP_RXOVIE);   
                        
    // sets enables and clears any stale flags
    return writeReg(std::to_underlying(RegisterAddress::REG_MCP_C1INT), interrupts);
}

int MCP251863::begin() {
    if (state_ != DriverState::Configuring) { return 0; }

    setTransceiverMode(TransceiverMode::TMODE_MCP_NORM);
    if (!setControllerMode(ControllerMode::CMODE_MCP_CFD_NORM) ||
        !waitForOpMode(ControllerMode::CMODE_MCP_CFD_NORM)) {
        setTransceiverMode(TransceiverMode::TMODE_MCP_STBY);
        return 0;
    }

    state_ = DriverState::Running;
    return 1;
}

int MCP251863::setBitTiming(BitTiming nominalTiming, BitTiming dataTiming) {
    MCP_TRY(writeReg(std::to_underlying(RegisterAddress::REG_MCP_C1NBTCFG),
                     pack_nominal_bit_timing(nominalTiming)));
    return writeReg(std::to_underlying(RegisterAddress::REG_MCP_C1DBTCFG),
                    pack_data_bit_timing(dataTiming));
}

int MCP251863::reset() {
    for (FifoInfo& f : fifoInfo) { f = FifoInfo{}; }
    state_ = DriverState::Configuring;

    const uint8_t message[2] = {0, 0};

    csSelect();
    spi_write_blocking(spi_, message, 2);
    csDeselect();

    return 1;
}

int MCP251863::fifoRamFits(uint8_t fifoNum, bool isTx) const {
    // each message is an 8-byte header plus the payload
    const size_t txFifoBytes = config_.txFifoDepth * (8 + fifo_plsize_to_len(config_.txPayloadSize));
    const size_t rxFifoBytes = config_.rxFifoDepth * (8 + fifo_plsize_to_len(config_.rxPayloadSize));
    const size_t unusedFifoBytes = 8 + 8;  // reset size: 1 message, 8-byte payload

    size_t total  = 0;  // RAM used by FIFO1 up to FIFO i
    size_t needed = 0;  // RAM used up to the end of the last FIFO in use

    for (uint8_t i = 0; i <= 30; i++) {
        bool inUse = fifoInfo[i].configured;
        bool tx    = fifoInfo[i].isTx;
        if (i == fifoNum) {
            inUse = true;
            tx    = isTx;
        }
        if (!inUse)  { total += unusedFifoBytes; } 
        else if (tx) { total += txFifoBytes; } 
        else         { total += rxFifoBytes; }

        if (inUse) { needed = total; }
    }

    return needed <= 2048; // 2 KB RAM
}

bool MCP251863::filterInUse(uint8_t fltNum, uint8_t exceptFifo) const {
    for (uint8_t i = 0; i <= 30; i++) {
        const FifoInfo& f = fifoInfo[i];
        if (i != exceptFifo && f.configured && !f.isTx &&
            (f.rxFltNum == fltNum || f.rxFltNum2 == fltNum)) {
            return true;
        }
    }
    return false;
}

int MCP251863::releaseFifo(uint8_t fifoNum) {
    const FifoInfo& f = fifoInfo[fifoNum];
    if (f.configured && !f.isTx) {
        MCP_TRY(disableFilter(f.rxFltNum));
        if (f.rxFltNum2 != NoFilter) { MCP_TRY(disableFilter(f.rxFltNum2)); }
    }
    fifoInfo[fifoNum] = FifoInfo{};
    return 1;
}

int MCP251863::configureTxFifo(uint8_t fifoNum, uint8_t prioNum) {
    if (state_ != DriverState::Configuring) { return 0; }
    if (fifoNum > 30 || prioNum > 31) { return 0; }
    if (!fifoRamFits(fifoNum, true)) { return 0; }
    MCP_TRY(releaseFifo(fifoNum));
    MCP_TRY(initGeneralPurposeFifo(fifoNum, FifoMode::FIFO_MODE_MCP_TX, config_.txPayloadSize,
                                   config_.txFifoDepth, prioNum,
                                   TxRetransmitMode::TXRET_MCP_UNLIM, 0)); // no interrupts for TX
    fifoInfo[fifoNum].configured = true;
    fifoInfo[fifoNum].isTx       = true;
    return 1;
}

int MCP251863::configureRxFifo(uint8_t fifoNum, uint8_t fltNum, uint16_t canSID,
                                 uint16_t sidMask) {
    if (state_ != DriverState::Configuring) { return 0; }
    if (fifoNum > 30 || fltNum > 31 || canSID > 0x7FF || sidMask > 0x7FF) { return 0; }
    if (!fifoRamFits(fifoNum, false)) { return 0; }
    if (filterInUse(fltNum, fifoNum)) { return 0; }
    MCP_TRY(releaseFifo(fifoNum));
    // enable Not Empty and Overflow interrupts for this FIFO
    const uint8_t rxFlags = std::to_underlying(FifoInterruptFlag::FIFO_INT_MCP_NFNE) |
                            std::to_underlying(FifoInterruptFlag::FIFO_INT_MCP_OVFL);
    MCP_TRY(initGeneralPurposeFifo(fifoNum, FifoMode::FIFO_MODE_MCP_RX, config_.rxPayloadSize,
                                   config_.rxFifoDepth,
                                   0,                                 // prioNum : don't care for RX
                                   TxRetransmitMode::TXRET_MCP_NONE,  // don't care
                                   rxFlags));
    MCP_TRY(initFilter(fltNum, fifoNum, canSID, sidMask));
    fifoInfo[fifoNum].configured = true;
    fifoInfo[fifoNum].isTx       = false;
    fifoInfo[fifoNum].rxFltNum   = fltNum;
    return 1;
}

int MCP251863::configureRxFifo(uint8_t fifoNum, uint16_t canSID) {
    return configureRxFifo(fifoNum, fifoNum, canSID);
}

int MCP251863::configureRxFifoTwoIds(uint8_t fifoNum, uint16_t canSID1, uint16_t canSID2) {
    if (fifoNum > 30 || canSID2 > 0x7FF) { return 0; }
    const uint8_t fltNum2 = 31 - fifoNum;
    if (filterInUse(fltNum2, fifoNum)) { return 0; }
    MCP_TRY(configureRxFifo(fifoNum, fifoNum, canSID1));
    if (!initFilter(fltNum2, fifoNum, canSID2, 0x7FF)) {
        (void)releaseFifo(fifoNum); 
        return 0;
    }
    fifoInfo[fifoNum].rxFltNum2 = fltNum2;
    return 1;
}

int MCP251863::initGeneralPurposeFifo(uint8_t fifoNum, FifoMode fifoMode, FifoPayloadSize plSize,
                                        uint8_t fSize, uint8_t prioNum, TxRetransmitMode retranMode,
                                        uint8_t intFlags) {
    uint8_t buff[4];
    const uint16_t addr = fifo_con_addr(fifoNum);

    buff[0] = intFlags | (std::to_underlying(fifoMode) << 7);
    buff[1] = 0;
    buff[2] = (std::to_underlying(retranMode) << 5) | prioNum;
    // FSIZE stores depth-1 (0 = 1 message; 31 = 32 messages); caller passes 1..32
    buff[3] = ((std::to_underlying(plSize) & 0b111) << 5) | ((fSize - 1) & 0x1F);

    return writeAddr(addr, buff, 4);
}

int MCP251863::disableFifo(uint8_t fifoNum) {
    if (fifoNum > 30) { return 0; }
    if (!fifoInfo[fifoNum].configured) { return 0; }
    const uint16_t addr = fifo_con_addr(fifoNum);

    const bool wasTx = fifoInfo[fifoNum].isTx;

    MCP_TRY(releaseFifo(fifoNum));

    if (state_ == DriverState::Running) {
        // clear TXREQ to abort in case a TX FIFO still has frames queued
        if (wasTx) {
            MCP_TRY(updateByte(addr + 1, 0b00000010, 0x00));  // TXREQ = 0
            MCP_TRY(pollRegisterBit(addr + 1, 0b00000010, false));
        }
        // set FRESET bit to empty FIFO (FIFO configuration & RAM is still retained)
        MCP_TRY(updateByte(addr + 1, 0x00, 0b00000100));
        MCP_TRY(pollRegisterBit(addr + 1, 0b00000100, false));
    } else {
        const uint8_t zero = 0;
        MCP_TRY(writeAddr(addr + 0, &zero, 1));  // TXEN, RXTSEN, interrupt enables off
        MCP_TRY(writeAddr(addr + 3, &zero, 1));  // PLSIZE = 8 bytes, FSIZE = 1 message: RAM freed
    }
    return 1;
}

int MCP251863::disableFilter(uint8_t fltNum) {
    const uint8_t off = 0x00;  
    return writeAddr(std::to_underlying(RegisterAddress::REG_MCP_C1FLTCONx) + fltNum, &off, 1);
}

int MCP251863::initFilter(uint8_t fltNum, uint8_t fifoNum, uint16_t canSID, uint16_t sidMask) {
    const uint16_t flt_addr = std::to_underlying(RegisterAddress::REG_MCP_C1FLTCONx) + fltNum;
    const uint16_t flt_obj_addr =
        std::to_underlying(RegisterAddress::REG_MCP_C1FLTOBJx) + 8 * fltNum;
    const uint16_t flt_mask_addr =
        std::to_underlying(RegisterAddress::REG_MCP_C1MASKx) + 8 * fltNum;

    uint8_t buff[4];

    MCP_TRY(disableFilter(fltNum));

    // Supports standard ID only
    buff[0] = canSID & 0xFF;
    buff[1] = (canSID >> 8) & 0x07;
    buff[2] = 0x00;
    buff[3] = 0x00;  // EXIDE = 0, so with MIDE set below only standard-ID frames match
    MCP_TRY(writeAddr(flt_obj_addr, buff, 4));

    // Set mask (only the masked ID bits must match)
    buff[0] = sidMask & 0xFF;         // MSID<7:0>
    buff[1] = (sidMask >> 8) & 0x07;  // MSID<10:8>
    buff[2] = 0x00;                   // MEID = don't care
    buff[3] = 0b01000000;             // MIDE = 1; MSID11 = 0 (don't care)
    MCP_TRY(writeAddr(flt_mask_addr, buff, 4));

    // Re-enable
    buff[0] = (fifoNum + 1) | (1 << 7);
    return writeAddr(flt_addr, buff, 1);
}

int MCP251863::readFifoUserAddress(uint8_t fifoNum, size_t slotSize, uint16_t* messageAddr) {
    uint8_t buff[2] = {0};
    MCP_TRY(readAddr(fifo_ua_addr(fifoNum), buff, 2));
    const uint16_t ua = static_cast<uint16_t>(buff[0] | (buff[1] << 8));
    if ((ua & 0x3) != 0 || ua + slotSize > 0x800) { return 0; }
    *messageAddr = ua + 0x400;
    return 1;
}

int MCP251863::send_canfd(uint8_t fifoNum, uint32_t id, const uint8_t* data, size_t len, bool brs,
                          bool* queued) {
    if (queued == nullptr) { return 0; }
    *queued = false;
    if (state_ != DriverState::Running) { return 0; }
    MCP_TRY(checkFifo(fifoNum, true));
    const size_t slotPayload = fifo_plsize_to_len(config_.txPayloadSize);
    if (len > slotPayload) { return 0; }
    if (len > 0 && data == nullptr) { return 0; }

    // double error here means the frame transmitted from earlier was corrupted
    // that frame has already been sent so just update the ecc counter
    (void)eccCheckPassed();

    CanFdFrame frame{};
    frame.id  = id;
    frame.ide = false;
    frame.fdf = true;
    frame.brs = brs;
    frame.len = static_cast<uint8_t>(len);
    if (len > 0) { memcpy(frame.data, data, len); }

    uint8_t message[72];
    size_t objectSize = 0;
    MCP_TRY(create_message_obj(message, frame, &objectSize));

    // check if TX FIFO is full
    uint8_t stat = 0;
    MCP_TRY(readAddr(fifo_sta_addr(fifoNum), &stat, 1));
    if ((stat & 0x01) == 0) { return 1; }  // TFNRFNIF = 0: FIFO full, *queued stays false

    // get message address + write frame to that address
    uint16_t messageAddr = 0;
    MCP_TRY(readFifoUserAddress(fifoNum, 8 + slotPayload, &messageAddr));
    MCP_TRY(writeAddr(messageAddr, message, objectSize));

    // increment TX FIFO pointer
    const uint8_t uincTxreq = 0b00000011;
    MCP_TRY(writeAddr(fifo_con_addr(fifoNum) + 1, &uincTxreq, 1));
    *queued = true; // message was successfully queued
    return 1;
}

int MCP251863::read_frame(uint8_t fifoNum, CanFdFrame* frame, bool* received) {
    if (frame == nullptr || received == nullptr) { return 0; }
    *received = false;
    if (state_ != DriverState::Running) { return 0; }
    MCP_TRY(checkFifo(fifoNum, false));

    // check if RX FIFO is empty
    uint8_t stat = 0;
    MCP_TRY(readAddr(fifo_sta_addr(fifoNum), &stat, 1));
    if ((stat & 0x01) == 0) { return 1; }  // TFNRFNIF = 0: FIFO empty, *received stays false

    // get message's address
    const size_t slotPayload = fifo_plsize_to_len(config_.rxPayloadSize);
    uint16_t messageAddr = 0;
    MCP_TRY(readFifoUserAddress(fifoNum, 8 + slotPayload, &messageAddr));

    // read and decode RX message's header
    uint8_t header[8];
    MCP_TRY(readAddr(messageAddr, header, 8));
    CanFdFrame rx = decode_rx_header(header, false);

    const uint8_t uinc = 0b00000001;
    if (rx.len > slotPayload) {
        MCP_TRY(writeAddr(fifo_con_addr(fifoNum) + 1, &uinc, 1));
        return 0;
    }

    // round up read length to multiple of four (words)
    const size_t readLen = (rx.len + 3) & ~size_t{3};
    if (readLen > 0) { MCP_TRY(readAddr(messageAddr + 8, rx.data, readLen)); }

    // double error means the frame just read is corrupted
    const bool eccOk = eccCheckPassed();

    MCP_TRY(writeAddr(fifo_con_addr(fifoNum) + 1, &uinc, 1));
    if (!eccOk) { return 0; }  // drop the frame
    *frame    = rx;
    *received = true;
    return 1;
}

int MCP251863::getFIFOStatus(uint8_t fifoNum, FifoStatus* status) {
    if (status == nullptr || fifoNum > 30) { return 0; }
    uint32_t reg = 0;
    MCP_TRY(readReg(fifo_sta_addr(fifoNum), &reg));

    FifoStatus s{};
    s.fifo_index              = (reg >> 8) & 0x1F;
    s.tx_aborted              = (reg & (1UL << 7)) != 0;
    s.tx_lost_arbitration     = (reg & (1UL << 6)) != 0;
    s.tx_error                = (reg & (1UL << 5)) != 0;
    s.tx_attempts_exhausted   = (reg & (1UL << 4)) != 0;
    s.rx_overflow             = (reg & (1UL << 3)) != 0;
    s.empty_or_full           = (reg & (1UL << 2)) != 0;
    s.half_empty_or_half_full = (reg & (1UL << 1)) != 0;
    s.not_full_or_not_empty   = (reg & 1UL) != 0;
    *status = s;
    return 1;
}

int MCP251863::clearRxOverflow(uint8_t fifoNum) {
    MCP_TRY(checkFifo(fifoNum, false));
    const uint8_t zero = 0;
    return writeAddr(fifo_sta_addr(fifoNum), &zero, 1);
}

// reads only C1TREC for bus-off or error-passive checks (avoid reading 9 reg all the time)
int MCP251863::getBusErrors(BusErrorState* errors) {
    if (errors == nullptr) { return 0; }
    uint32_t trec = 0;
    MCP_TRY(readReg(std::to_underlying(RegisterAddress::REG_MCP_C1TREC), &trec));
    *errors = decode_trec(trec);
    return 1;
} 

int MCP251863::getStatus(Status* status) {
    if (status == nullptr) { return 0; }
    Status s{};
    uint32_t trec = 0;
    MCP_TRY(readReg(std::to_underlying(RegisterAddress::REG_MCP_C1INT), &s.interrupt_flags));
    MCP_TRY(readReg(std::to_underlying(RegisterAddress::REG_MCP_C1RXIF), &s.rx_if));
    MCP_TRY(readReg(std::to_underlying(RegisterAddress::REG_MCP_C1TXIF), &s.tx_if));
    MCP_TRY(readReg(std::to_underlying(RegisterAddress::REG_MCP_C1RXOVIF), &s.rx_overflow_if));
    MCP_TRY(readReg(std::to_underlying(RegisterAddress::REG_MCP_C1TXATIF), &s.tx_attempt_if));
    MCP_TRY(readReg(std::to_underlying(RegisterAddress::REG_MCP_C1TREC), &trec));
    MCP_TRY(readReg(std::to_underlying(RegisterAddress::REG_MCP_C1BDIAGx), &s.bdiag0));
    MCP_TRY(readReg(std::to_underlying(RegisterAddress::REG_MCP_C1BDIAGx) + 4, &s.bdiag1));
    MCP_TRY(readReg(std::to_underlying(RegisterAddress::REG_MCP_CRC), &s.crc));
    s.errors = decode_trec(trec);
    s.ecc_double_errors = eccDoubleErrors_;
    s.spi_crc_errors    = spiCrcErrors_;
    *status = s;
    return 1;
}

int MCP251863::setControllerMode(ControllerMode contMode) {
    const uint16_t addr = std::to_underlying(RegisterAddress::REG_MCP_C1CON);
    uint8_t current = 0;
    MCP_TRY(readOpMode(&current));
    if (current != std::to_underlying(ControllerMode::CMODE_MCP_CONF)) {
        MCP_TRY(
            updateByte(addr + 3, 0b00000111, std::to_underlying(ControllerMode::CMODE_MCP_CONF)));
        MCP_TRY(waitForOpMode(ControllerMode::CMODE_MCP_CONF));
    }
    return updateByte(addr + 3, 0b00000111, std::to_underlying(contMode));
}

void MCP251863::setTransceiverMode(TransceiverMode mode) const {
    gpio_put(standbyPin_, std::to_underlying(mode));
}

bool MCP251863::interruptPending() const { return !gpio_get(intPin_); }

int MCP251863::pendingRxFifos(uint32_t* rxMask, uint32_t* overflowMask) {
    uint8_t buf[12];
    MCP_TRY(readAddr(std::to_underlying(RegisterAddress::REG_MCP_C1RXIF), buf, 12));
    // shift right by 1 b/c FIFO 0 in hardware is the TXQ FIFO
    *rxMask       = load_word(buf) >> 1;      // C1RXIF
    *overflowMask = load_word(buf + 8) >> 1;  // C1RXOVIF
    return 1;
}

int MCP251863::setPinMode(IoPin pin, IoMode mode) {
    if (std::to_underlying(pin) > 1) { return 0; }
    const uint8_t bit = static_cast<uint8_t>(1U << std::to_underlying(pin));
    const auto clear  = static_cast<uint8_t>(~bit);

    uint8_t buff[4] = {0};
    MCP_TRY(readAddr(std::to_underlying(RegisterAddress::REG_MCP_IOCON), buff, 4));
    switch (mode) {
        case IoMode::IOMODE_MCP_GPIO_IN:
            buff[0] |= bit;
            buff[3] |= bit;
            break;
        case IoMode::IOMODE_MCP_GPIO_OUT:
            buff[0] &= clear;
            buff[3] |= bit;
            break;
        case IoMode::IOMODE_MCP_INT: buff[3] &= clear; break;
        default: return 0;
    }
    return writeAddr(std::to_underlying(RegisterAddress::REG_MCP_IOCON), buff, 4);
}

// ---------------- UNUSED -------------------------

// Error MCP251863::initTransmitEventFifo(uint8_t fSize, const FifoInterruptFlag* intFlagArray,
//                                        size_t intFlagSize) {
//     if (fSize < 1 || fSize > 32) {
//         return Error::InvalidFifoDepth;
//     }
//     if (intFlagSize > 0 && intFlagArray == nullptr) {
//         return Error::NullPointer;
//     }

//     uint8_t buff[4];
//     const uint16_t addr = std::to_underlying(RegisterAddress::REG_MCP_C1TEFCON);

//     uint8_t intFlags = 0;
//     for (size_t i = 0; i < intFlagSize; i++) {
//         intFlags |= static_cast<uint8_t>(intFlagArray[i]);
//     }

//     buff[0] = 0b00000000 | intFlags;
//     buff[1] = 0b00000000;
//     buff[2] = 0b00000000;
//     buff[3] = (fSize - 1) & 0x1F;

//     return writeAddr(addr, buff, 4);
// }

// Error MCP251863::initTransmitQueue(FifoPayloadSize plSize, uint8_t fSize, uint8_t prioNum,
//                                    TxRetransmitMode retranMode,
//                                    const FifoInterruptFlag* intFlagArray, size_t intFlagSize) {
//     if (fSize < 1 || fSize > 32) {
//         return Error::InvalidFifoDepth;
//     }
//     if (prioNum > 31) {
//         return Error::InvalidArgument;
//     }
//     if (fifo_plsize_to_len(plSize) == 0) {
//         return Error::InvalidPayloadSize;
//     }
//     if (intFlagSize > 0 && intFlagArray == nullptr) {
//         return Error::NullPointer;
//     }

//     uint8_t buff[4];
//     const uint16_t addr = std::to_underlying(RegisterAddress::REG_MCP_C1TXQCON);

//     uint8_t intFlags = 0;
//     for (size_t i = 0; i < intFlagSize; i++) {
//         intFlags |= static_cast<uint8_t>(intFlagArray[i]);
//     }

//     MCP_TRY(pollRegisterBit(addr + 1, 0x04, false, Error::PollTimeout));

//     buff[0] = 0b00000000 | intFlags;
//     buff[1] = 0b00000000;
//     buff[2] = 0b00000000 | (std::to_underlying(retranMode) << 5) | prioNum;
//     buff[3] = ((std::to_underlying(plSize) & 0b111) << 5) | ((fSize - 1) & 0x1F);

//     return writeAddr(addr, buff, 4);
// }

// C1VEC layout
// 31:24 rxcode (third byte)
// 23:16 txcode (second byte)
// 12:8 filhit (first byte, + some offset stuff)
// 7:0 icode (zeroth byte)

// Result<int> MCP251863::getTXCode() {
//     uint8_t buff = 0;
//     MCP_TRY(readAddr(std::to_underlying(RegisterAddress::REG_MCP_C1VEC) + 2, &buff, 1));
//     return static_cast<int>(buff & 0x7F);
// }

// Result<int> MCP251863::getRXCode() {
//     uint8_t buff = 0;
//     MCP_TRY(readAddr(std::to_underlying(RegisterAddress::REG_MCP_C1VEC) + 3, &buff, 1));
//     return static_cast<int>(buff & 0x7F);
// }

// Result<int> MCP251863::getFLTCode() {
//     // its 5 bits so we mask with 0x1F instead of 0x7F
//     uint8_t buff = 0;
//     MCP_TRY(readAddr(std::to_underlying(RegisterAddress::REG_MCP_C1VEC) + 1, &buff, 1));
//     return static_cast<int>(buff & 0x1F);
// }

// Result<int> MCP251863::getICode() {
//     uint8_t buff = 0;
//     MCP_TRY(readAddr(std::to_underlying(RegisterAddress::REG_MCP_C1VEC), &buff, 1));
//     buff &= 0x7F;
//     if (buff > 0b1001010) {  // max valid ICODE value
//         return Error::BadRegisterValue;
//     }
//     return static_cast<int>(buff);
// }