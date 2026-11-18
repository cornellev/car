#ifndef MCP251863_H
#define MCP251863_H

#include <stdlib.h>
#include <string.h>

#include <cstdio>
#include <optional>
#include <type_traits>

#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

// Constants and magic numbers
const uint MCP251863_BAUD_RATE = 16000000;
const uint MAX_TRANSFER        = 80;  // 2 cmd + 8 header + 4 opt. rx timestamp + 64 payload + 2 crc
const uint MCP251863_MAX_PAYLOAD = 64;

const uint MCP251863_CLOCK_HZ = 40000000;

// each category starts on its own 0x10 block, and the rest count up from there
// NOLINTNEXTLINE(readability-enum-initial-value)
enum class Error : uint8_t {
    None = 0x00,

    // Programmer errors
    InvalidFifoNum = 0x10,
    InvalidFifoDepth,
    InvalidFilterNum,
    InvalidPayloadSize,
    IdOutOfRange,
    InvalidBitTiming,
    InvalidArgument,
    NullPointer,
    NoOperationPending,

    // Transient / retryable
    Busy = 0x20,
    TxFifoFull,
    RxFifoEmpty,

    // Hardware / bus faults
    SpiTimeout = 0x30,
    PollTimeout,
    ModeChangeTimeout,
    OscillatorTimeout,
    PllTimeout,
    SclkdivTimeout,
    BusOff,
    RxOverflow,
    TxAttemptsExhausted,
    BadRegisterValue,
};

enum class ErrorCategory : uint8_t {
    None       = 0x00,
    Programmer = 0x10,
    Transient  = 0x20,
    Hardware   = 0x30,
};

constexpr ErrorCategory categoryOf(Error e) {
    return static_cast<ErrorCategory>(static_cast<uint8_t>(e) & 0xF0);
}
constexpr bool isProgrammerError(Error e) { return categoryOf(e) == ErrorCategory::Programmer; }
constexpr bool isTransient(Error e) { return categoryOf(e) == ErrorCategory::Transient; }
constexpr bool isHardwareFault(Error e) { return categoryOf(e) == ErrorCategory::Hardware; }

enum class RecoveryAction : uint8_t {
    None,
    FixCaller,
    RetryLater,
    Initialize,
    WaitForBus,
    Reinitialize,
    CheckHardware
};

constexpr RecoveryAction recoveryFor(Error e) {
    switch (e) {
        case Error::None: return RecoveryAction::None;

        case Error::Busy:
        case Error::TxFifoFull:
        case Error::RxFifoEmpty:
        case Error::RxOverflow:
        case Error::TxAttemptsExhausted: return RecoveryAction::RetryLater;

        case Error::SpiTimeout:
        case Error::PollTimeout:
        case Error::ModeChangeTimeout:
        case Error::BadRegisterValue: return RecoveryAction::Reinitialize;

        case Error::BusOff: return RecoveryAction::WaitForBus;

        case Error::OscillatorTimeout:
        case Error::PllTimeout:
        case Error::SclkdivTimeout: return RecoveryAction::CheckHardware;

        default: return RecoveryAction::FixCaller;  // for remaining programming errors
    }
}

constexpr const char* errorToString(Error e) {
    switch (e) {
        case Error::None: return "None";
        case Error::InvalidFifoNum: return "InvalidFifoNum";
        case Error::InvalidFifoDepth: return "InvalidFifoDepth";
        case Error::InvalidFilterNum: return "InvalidFilterNum";
        case Error::InvalidPayloadSize: return "InvalidPayloadSize";
        case Error::IdOutOfRange: return "IdOutOfRange";
        case Error::InvalidBitTiming: return "InvalidBitTiming";
        case Error::InvalidArgument: return "InvalidArgument";
        case Error::NullPointer: return "NullPointer";
        case Error::NoOperationPending: return "NoOperationPending";
        case Error::Busy: return "Busy";
        case Error::TxFifoFull: return "TxFifoFull";
        case Error::RxFifoEmpty: return "RxFifoEmpty";
        case Error::SpiTimeout: return "SpiTimeout";
        case Error::PollTimeout: return "PollTimeout";
        case Error::ModeChangeTimeout: return "ModeChangeTimeout";
        case Error::OscillatorTimeout: return "OscillatorTimeout";
        case Error::PllTimeout: return "PllTimeout";
        case Error::SclkdivTimeout: return "SclkdivTimeout";
        case Error::BusOff: return "BusOff";
        case Error::RxOverflow: return "RxOverflow";
        case Error::TxAttemptsExhausted: return "TxAttemptsExhausted";
        case Error::BadRegisterValue: return "BadRegisterValue";
    }
    return "Unknown";
}

// for functions that can also return non-Error types
template <typename T>
class Result {
   public:
    Result(const T& value) : value_(value), error_(Error::None) {}
    Result(Error error) : value_(T()), error_(error) {}

    [[nodiscard]] bool ok() const { return error_ == Error::None; }
    explicit operator bool() const { return ok(); }
    [[nodiscard]] Error error() const { return error_; }
    [[nodiscard]] const T& value() const { return value_; }
    [[nodiscard]] T& value() { return value_; }

   private:
    T value_;
    Error error_;
};

// Enums for various device modes/configs

// write mode
enum class WriteMode : uint8_t { WM_MCP_NORM = 0, WM_MCP_CRC = 1, WM_MCP_SAFE = 2 };

// read mode
enum class ReadMode : uint8_t { RM_MCP_NORM = 0, RM_MCP_CRC = 1 };

// command type
enum class Command : uint8_t {
    CMD_MCP_RESET = 0b0000,  // reset
    CMD_MCP_READA = 0b0011,  // read address
    CMD_MCP_WRITA = 0b0010,  // write address
    CMD_MCP_RDACR = 0b1011,  // read address crc
    CMD_MCP_WRACR = 0b1010,  // write address crc
    CMD_MCP_WRASF = 0b1100   // write address safe
};
// controller mode
enum class ControllerMode : uint8_t {
    CMODE_MCP_CONF      = 0b100,  // config
    CMODE_MCP_CFD_NORM  = 0b000,  // canfd normal
    CMODE_MCP_C2_NORM   = 0b110,  // can2 normal
    CMODE_MCP_SLP       = 0b001,  // sleep
    CMODE_MCP_LIST_ONLY = 0b011,  // listen only
    CMODE_MCP_RESTR_OP  = 0b111,  // restricted operation
    CMODE_MCP_INT_LOOP  = 0b010,  // internal loopback
    CMODE_MCP_EXT_LOOP  = 0b101   // external loopback
};
enum class TransceiverMode : uint8_t { TMODE_MCP_STBY = 1, TMODE_MCP_NORM = 0 };

enum class TxRetransmitMode : uint8_t {
    TXRET_MCP_NONE  = 0b00,
    TXRET_MCP_THREE = 0b01,
    TXRET_MCP_UNLIM = 0b10
};

enum class FifoInterruptFlag : uint8_t {
    FIFO_INT_MCP_NFNE  = 0b00000001,  // fifo not full (TX), fifo not empty (RX)
    FIFO_INT_MCP_HFHE_ = 0b00000010,  // fifo half full(TX), fifo half empty (RX)
    FIFO_INT_MCP_FFEE  = 0b00000100,  // fifo full (TX), fifo empty (RX),
    FIFO_INT_MCP_OVFL  = 0b00001000,  // fifo overflow (RX),
    FIFO_INT_MCP_TXAT  = 0b00010000,  // transmits exhausted
};

enum class RegisterAddress : uint16_t {
    REG_MCP_OSC        = 0xE00,
    REG_MCP_IOCON      = 0xE04,
    REG_MCP_CRC        = 0xE08,
    REG_MCP_ECCCON     = 0xE0C,
    REG_MCP_ECCSTAT    = 0xE10,
    REG_MCP_DEVID      = 0xE14,
    REG_MCP_C1CON      = 0x0,
    REG_MCP_C1NBTCFG   = 0x4,
    REG_MCP_C1DBTCFG   = 0x8,
    REG_MCP_C1TDC      = 0xC,
    REG_MCP_C1TBC      = 0x10,
    REG_MCP_C1TSCON    = 0x14,
    REG_MCP_C1VEC      = 0x18,
    REG_MCP_C1INT      = 0x1C,
    REG_MCP_C1RXIF     = 0x20,
    REG_MCP_C1TXIF     = 0x24,
    REG_MCP_C1RXOVIF   = 0x28,
    REG_MCP_C1TXATIF   = 0x2C,
    REG_MCP_C1TXREQ    = 0x30,
    REG_MCP_C1TREC     = 0x34,
    REG_MCP_C1BDIAGx   = 0x38,  // 4 addresses between each
    REG_MCP_C1TEFCON   = 0x40,
    REG_MCP_C1TEFSTA   = 0x44,
    REG_MCP_C1TEFUA    = 0x48,
    REG_MCP_C1TXQCON   = 0x50,
    REG_MCP_C1TXQSTA   = 0x54,
    REG_MCP_C1TXQUA    = 0x58,
    REG_MCP_C1FIFOCONx = 0x5C,   // 12 addresses between each
    REG_MCP_C1FIFOSTAx = 0x60,   // 12 addresses between each
    REG_MCP_C1FIFOUAx  = 0x64,   // 12 addresses between each
    REG_MCP_C1FLTCONx  = 0x1D0,  // 4 addresses between each
    REG_MCP_C1FLTOBJx  = 0x1F0,  // 8 addresses between each
    REG_MCP_C1MASKx    = 0x1F4   // 8 addresses between each
};

enum class FifoMode : uint8_t { FIFO_MODE_MCP_TX = 1, FIFO_MODE_MCP_RX = 0 };

enum class PayloadSize : uint8_t {
    PL_SIZE_MCP_0  = 0b0000,
    PL_SIZE_MCP_1  = 0b0001,
    PL_SIZE_MCP_2  = 0b0010,
    PL_SIZE_MCP_3  = 0b0011,
    PL_SIZE_MCP_4  = 0b0100,
    PL_SIZE_MCP_5  = 0b0101,
    PL_SIZE_MCP_6  = 0b0110,
    PL_SIZE_MCP_7  = 0b0111,
    PL_SIZE_MCP_8  = 0b1000,
    PL_SIZE_MCP_12 = 0b1001,
    PL_SIZE_MCP_16 = 0b1010,
    PL_SIZE_MCP_20 = 0b1011,
    PL_SIZE_MCP_24 = 0b1100,
    PL_SIZE_MCP_32 = 0b1101,
    PL_SIZE_MCP_48 = 0b1110,
    PL_SIZE_MCP_64 = 0b1111
};

enum class FifoPayloadSize : uint8_t {
    FIFO_PLSIZE_8  = 0b000,
    FIFO_PLSIZE_12 = 0b001,
    FIFO_PLSIZE_16 = 0b010,
    FIFO_PLSIZE_20 = 0b011,
    FIFO_PLSIZE_24 = 0b100,
    FIFO_PLSIZE_32 = 0b101,
    FIFO_PLSIZE_48 = 0b110,
    FIFO_PLSIZE_64 = 0b111
};

enum class InterruptEnable : uint32_t {
    INT_EN_MCP_TXIF     = 0b00000000000000000000000000000001,
    INT_EN_MCP_RXIF     = 0b00000000000000000000000000000010,
    INT_EN_MCP_TBCIF    = 0b00000000000000000000000000000100,
    INT_EN_MCP_MODIF    = 0b00000000000000000000000000001000,
    INT_EN_MCP_TEFIF    = 0b00000000000000000000000000010000,
    INT_EN_MCP_ECCIF    = 0b00000000000000000000000100000000,
    INT_EN_MCP_SPICRCIF = 0b00000000000000000000001000000000,
    INT_EN_MCP_TXATIF   = 0b00000000000000000000010000000000,
    INT_EN_MCP_RXOVIF   = 0b00000000000000000000100000000000,
    INT_EN_MCP_SERRIF   = 0b00000000000000000001000000000000,
    INT_EN_MCP_CERRIF   = 0b00000000000000000010000000000000,
    INT_EN_MCP_WAKIF    = 0b00000000000000000100000000000000,
    INT_EN_MCP_IVMIF    = 0b00000000000000001000000000000000,
    INT_EN_MCP_TXIE     = 0b00000000000000010000000000000000,
    INT_EN_MCP_RXIE     = 0b00000000000000100000000000000000,
    INT_EN_MCP_TBCIE    = 0b00000000000001000000000000000000,
    INT_EN_MCP_MODIE    = 0b00000000000010000000000000000000,
    INT_EN_MCP_TEFIE    = 0b00000000000100000000000000000000,
    INT_EN_MCP_ECCIE    = 0b00000001000000000000000000000000,
    INT_EN_MCP_SPICRCIE = 0b00000010000000000000000000000000,
    INT_EN_MCP_TXATIE   = 0b00000100000000000000000000000000,
    INT_EN_MCP_RXOVIE   = 0b00001000000000000000000000000000,
    INT_EN_MCP_SERRIE   = 0b00010000000000000000000000000000,
    INT_EN_MCP_CERRIE   = 0b00100000000000000000000000000000,
    INT_EN_MCP_WAKIE    = 0b01000000000000000000000000000000,
    INT_EN_MCP_IVMIE    = 0b10000000000000000000000000000000
};

enum class IoPin : uint8_t { IO_MCP_INT0 = 0, IO_MCP_INT1 = 1 };

enum class IoMode : uint8_t { IOMODE_MCP_INT = 0, IOMODE_MCP_GPIO_OUT = 1, IOMODE_MCP_GPIO_IN = 2 };

enum class MessageType : uint8_t {
    CAN_BASE_MCP    = 0,
    CAN_FD_BASE_MCP = 1,
    CAN_EXT         = 2,
    CAN_FD_EXT      = 3
};

enum class FifoOperationState : uint8_t { IDLE, PUSHING, POPPING, POP_DONE, PUSH_DONE };

// Structs

struct BitTiming {
    uint8_t brp;    // Baud rate prescaler (0 = divide by 1)
    uint8_t tseg1;  // Time segment 1 (propagation + phase1)
    uint8_t tseg2;  // Time segment 2 (phase2)
    uint8_t sjw;    // Synchronization jump width
};

struct FifoInfo {
    bool configured = false;
    bool isTx       = false;
    uint8_t rxFltNum{};  // only for RX FIFOs
};

struct CanFdFrame {
    uint32_t id;           // CAN ID (11-bit for std, 29-bit for ext)
    uint8_t dlc;           // Data length code (register value)
    uint8_t len;           // Actual payload length in bytes
    bool ide;              // Extended ID flag
    bool fdf;              // FD frame flag
    bool brs;              // Bit rate switch flag
    bool rtr;              // Remote transmission request
    bool esi;              // Error status indicator
    bool sid11;            // SID11 field for CAN-FD base frames
    bool valid;            // Frame contains valid data
    uint8_t filter_hit;    // Filter index that matched (RX only)
    uint32_t sequence;     // Sequence number (TX only)
    bool timestamp_valid;  // Timestamp field is populated
    uint32_t timestamp;    // Received timestamp (RX only, if enabled)
    uint8_t data[64];      // Payload bytes
};

struct InitConfig {
    int enablePll;          // Enable PLL (multiply oscillator by 10)
    int sclkDiv2;           // Divide SCLK by 2
    int enableTdc;          // Enable Transmitter Delay Compensation
    int rxTimestampEnable;  // Capture timestamp in received frames
    uint8_t tdcOffset;      // TDC offset (SSP position in TQs)
    uint8_t txFifoDepth;    // TX FIFO depth (1-32 messages)
    uint8_t rxFifoDepth;    // RX FIFO depth (1-32 messages)
    FifoPayloadSize txPayloadSize;
    FifoPayloadSize rxPayloadSize;
    BitTiming nominalBitTiming;
    BitTiming dataBitTiming;
};

struct Status {
    uint32_t interrupt_flags;
    uint32_t rx_if;
    uint32_t tx_if;
    uint32_t rx_overflow_if;
    uint32_t tx_attempt_if;
    uint32_t trec;    // CiTREC - transmit/receive error count reg
    uint32_t bdiag0;  // CiBDIAG0 - bus diagnostic register 0
    uint32_t bdiag1;  // CiBDIAG1 - bus diagnostic register 1
    uint32_t crc;
    bool bus_off;
    bool tx_error_passive;
    bool rx_error_passive;
    bool tx_error_warning;
    bool rx_error_warning;
    bool error_warning;
    uint8_t tx_error_count;
    uint8_t rx_error_count;
    // bool spi_crc_format_error;
    // bool spi_crc_error;
};

struct FifoStatus {
    uint8_t fifo_num;
    bool tx_aborted;
    bool tx_lost_arbitration;
    bool tx_error;
    bool tx_attempts_exhausted;
    bool rx_overflow;
    bool empty_or_full;
    bool half_empty_or_half_full;
    bool not_full_or_not_empty;
};

struct DriverStats {
    // Exact : driver updates counters directly
    uint32_t frames_tx_requested;
    uint32_t frames_rx;
    uint32_t tx_fifo_full_events;
    uint32_t dma_timeouts;
    uint32_t poll_timeouts;
    uint32_t recoveries;
    // Sampled : driver polls for these
    uint32_t tx_arbitration_losses;
    uint32_t tx_attempts_exhausted;
    uint32_t tx_errors;
    uint32_t rx_overflows;
    uint32_t bus_off_events;
};

// Bit timing presets for a 40 MHz CAN clock.
// Bit time = (BRP+1) * (TSEG1+TSEG2+3) / Fsys
//   500 K: (0+1)*(62+15+3)/40e6 = 80/40e6  = 2 us  → 500 Kbit/s, sample point ~79%
//     2 M: (0+1)*(14+ 3+3)/40e6 = 20/40e6  = 500 ns → 2 Mbit/s,  sample point ~75%
inline constexpr BitTiming kBitTiming500K40MHz = {0, 62, 15, 15};
inline constexpr BitTiming kBitTiming2M40MHz   = {0, 14, 3, 3};

// Main class
class MCP251863 {
   private:
    static MCP251863* instance_;
    static void dmaIrqHandler();
    static void gpioIrqHandler(uint gpio, uint32_t events);

    bool dmaInitialized = false;

    int dma_tx_chan{};
    int dma_rx_chan{};

    dma_channel_config dma_tx_cfg{};
    dma_channel_config dma_rx_cfg{};

    uint8_t tx_buff[MAX_TRANSFER]{};
    uint8_t rx_buff[MAX_TRANSFER]{};
    uint8_t* user_rx_buff   = nullptr;
    uint16_t user_fifo_addr = 0;
    CanFdFrame user_frame{};

    size_t transfer_len{};

    volatile FifoOperationState fifo_op_state = FifoOperationState::IDLE;
    volatile bool pending_fifo_uinc           = false;
    volatile bool error_flag_pending          = false;
    volatile bool rx_flag_pending             = false;
    volatile bool tx_flag_pending             = false;

    spi_inst_t* spi_;
    uint chipSelectPin_;
    uint standbyPin_;
    uint intPin_, int0Pin_, int1Pin_;
    WriteMode writeMode_      = WriteMode::WM_MCP_NORM;
    ReadMode readMode_        = ReadMode::RM_MCP_NORM;
    bool rxTimestampsEnabled_ = false;

    // store config info so it can be retrieved
    // during recoveries
    InitConfig lastConfig{};

    DriverStats stats{};

    FifoInfo fifoInfo[32];

    bool bus_off_latched = false;
    bool txlarb_latched  = false;
    bool txerr_latched   = false;

    // DMA-related functions
    Error initDMA();
    void spiTransferDMA(size_t len);
    void finishTransfer();

    void csSelect() const;
    void csDeselect() const;

    // Asynchronous SPI read/write transactions w/ DMA
    Error dmaReadAddr(uint16_t startAddr, uint8_t* dst, size_t len);
    Error dmaWriteAddr(uint16_t startAddr, const uint8_t* data, size_t len);

    // Helper functions
    Error readReg(uint16_t addr, uint32_t* dst);
    Error writeReg(uint16_t addr, uint32_t value);
    Error waitForOpMode(ControllerMode target);
    Error waitForOpMode(ControllerMode target, int maxIters, uint32_t delayUs);
    Error pollRegisterBit(uint16_t addr, uint8_t mask, bool wantSet, Error timeoutType);
    Error pollRegisterBit(uint16_t addr, uint8_t mask, bool wantSet, int maxIters, uint32_t delayUs,
                          Error timeoutType);
    Error updateByte(uint16_t addr, uint8_t field, uint8_t value);
    [[nodiscard]] Error checkFifo(uint8_t fifoNum, bool wantTx) const;
    Error readFifoUserAddress(uint16_t fifoPointAddr, size_t objectSize, uint16_t* messageAddr);
    Error serviceFifoUinc();

    Error configureDevice(const InitConfig& config);

    Error initGeneralPurposeFifo(uint8_t fifoNum, FifoMode fifoMode, FifoPayloadSize plSize,
                                 uint8_t fSize, uint8_t prioNum, TxRetransmitMode retranMode,
                                 const FifoInterruptFlag* intFlagArray, size_t intFlagSize);

    Error initTransmitEventFifo(uint8_t fSize, const FifoInterruptFlag* intFlagArray,
                                size_t intFlagSize);
    Error initTransmitQueue(FifoPayloadSize plSize, uint8_t fSize, uint8_t prioNum,
                            TxRetransmitMode retranMode, const FifoInterruptFlag* intFlagArray,
                            size_t intFlagSize);

    // route one standard-ID filter to a FIFO and enable it
    Error initFilter(uint8_t fltNum, uint8_t fifoNum, uint16_t canSID, uint16_t sidMask);

    Error setControllerMode(ControllerMode mode);
    [[nodiscard]] Error setTransceiverMode(TransceiverMode mode) const;
    Error setInterrupts(const InterruptEnable* intEnArray, size_t intEnSize);

   public:
    Error readOpMode(uint8_t* opmod);
    // Blocking SPI read/write transactions
    Error readAddr(uint16_t startAddr, uint8_t* dst, size_t len);
    Error writeAddr(uint16_t startAddr, const uint8_t* data, size_t len);
    // make a driver bound to SPI and the pins
    MCP251863(spi_inst_t* ispi, uint iCSPin, uint iSTBYPin, uint iINTPin, uint iINT0Pin,
              uint iINT1Pin);

    // initialize the controller with defaults
    Error init();
    // initialize controller with a caller-provided timing
    Error init(const InitConfig& config);

    // optionally set timing of registers
    // (already initialized to default values by init())
    Error setBitTiming(BitTiming nominalTiming, BitTiming dataTiming);

    [[nodiscard]] Error enableErrorInterrupts() const;
    Error enableTxInterrupts();
    Error enableRxInterrupts();

    [[nodiscard]] bool errorInterruptPending() const { return error_flag_pending; }
    [[nodiscard]] bool rxInterruptPending() const { return rx_flag_pending; }
    [[nodiscard]] bool txInterruptPending() const { return tx_flag_pending; }

    void clearErrorInterruptPending() { error_flag_pending = false; }
    void clearRxInterruptPending() { rx_flag_pending = false; }
    void clearTxInterruptPending() { tx_flag_pending = false; }

    // init() does NOT configure any FIFOs. Must call these functions
    // to configure one more general-purpose FIFO after init()
    // fifoNum & fltNum range: 0 - 30
    Error configureTxFifo(uint8_t fifoNum);
    Error configureTxFifo(uint8_t fifoNum, uint8_t prioNum);
    // priority number range: 0 - 30
    // higher number = higher priority to send msgs onto bus
    Error configureRxFifo(uint8_t fifoNum, uint8_t fltNum, uint16_t canSID,
                          uint16_t sidMask = 0x7FF);
    // suscribes the RX FIFO to an ID
    // if 2 RX FIFOs suscribe to the same ID, the FIFO with
    // the lower fitNum has higher priority (msg would be passed
    // into *that* FIFO)
    Error configureRxFifo(uint8_t fifoNum, uint16_t canSID);
    // fltNum = fifoNum
    Error disableFifo(uint8_t fifoNum);

    // after init() is called, device is still in configuration mode
    // so user can freely configure FIFOs. After configuring FIFOs,
    // user must call begin() to enter normal mode
    Error begin();

    // can try reinitializing the MCP after a timeout error
    Error recover();
    // abort a DMA transaction
    void abort();

    // spi reset - resets all registers to default values
    Error reset();

    // kickstart DMA: push CAN FD frame to a TX FIFO
    Error start_push_canfd(uint8_t fifoNum, uint32_t id, const uint8_t* data, size_t len, bool brs,
                           bool extended_id = false);

    // alternative
    Error start_push_frame(uint8_t fifoNum, const CanFdFrame& frame);

    // returns whether DMA "push" transaction finished
    Error poll_push();

    // tell TX FIFO to send all its msgs over to the bus
    // should wait until poll_push returns Error::None before requesting
    Error request_send(uint8_t fifoNum);

    // user must call this before calling start_push_canfd/frame again
    // will prob remove this later, for now it's necessary
    Error clear_send();

    // kickstart DMA: pop and decode CAN FD frame from a RX FIFO
    Error start_pop_canfd(uint8_t fifoNum);

    // returns whether DMA "pop" transaction finished
    Error poll_pop();

    // retrieve popped frame once poll_read() returns Error::None
    Result<CanFdFrame> get_read_frame();

    // configure INT0/INT1 as interrupt pins or GPIOs
    Error setPinMode(IoPin pin, IoMode mode);

    // read controller-wide interrupt, error, and SPI CRC status registers
    Result<Status> getStatus();

    // read status flags for one FIFO
    Result<FifoStatus> getFIFOStatus(uint8_t fifoNum);

    Error clearRxOverflow(uint8_t fifoNum);

    // read driver stats; counts accumulate over time
    // could be useful for analyzing long-term driver behavior?
    [[nodiscard]] DriverStats getStats() const { return stats; }
    // reset driver stats
    void resetStats() { stats = DriverStats{}; }

    Result<int> getTXCode();
    Result<int> getRXCode();
    Result<int> getFLTCode();
    Result<int> getICode();
};

#endif
