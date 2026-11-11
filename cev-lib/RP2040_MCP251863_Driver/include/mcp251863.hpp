#ifndef MCP251863_H
#define MCP251863_H

#include <stdlib.h>
#include <string.h>

#include <cstdio>
#include <optional>
#include <type_traits>

#include "hardware/spi.h"
#include "pico/stdlib.h"

// Constants and magic numbers
const uint MCP251863_BAUD_RATE = 16000000;
const uint MCP251863_MAX_PAYLOAD = 64;
const uint MCP251863_CLOCK_HZ = 40000000;

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

enum class MessageType : uint8_t { CAN_BASE_MCP = 0, CAN_FD_BASE_MCP = 1, CAN_EXT = 2, CAN_FD_EXT = 3 };

//   Uninitialized: before init(), or after it failed; FIFO setup and begin() are refused
//   Configuring:   init() succeeded (or reset() ran), chip in config mode, FIFO setup allowed
//   Running:       begin() succeeded, chip on the bus, FIFO setup refused
enum class DriverState : uint8_t { Uninitialized, Configuring, Running };

// Structs

struct BitTiming {
    uint8_t brp;    // Baud rate prescaler (0 = divide by 1)
    uint8_t tseg1;  // Time segment 1 (propagation + phase1)
    uint8_t tseg2;  // Time segment 2 (phase2)
    uint8_t sjw;    // Synchronization jump width
};

constexpr uint8_t NoFilter = 0xFF;

struct FifoInfo {
    bool configured = false;
    bool isTx       = false;
    uint8_t rxFltNum{};            // only for RX FIFOs
    uint8_t rxFltNum2 = NoFilter;  // second filter
};

struct CanFdFrame {
    uint32_t id;           // CAN ID (11-bit for std, 29-bit for ext)
    uint8_t dlc;           // Data length code (register value)
    uint8_t len;           // Actual payload length in bytes
    bool ide;              // Extended ID flag
    bool fdf;              // FD frame flag
    bool brs;              // Bit rate switch flag
    bool esi;              // Error status indicator
    uint8_t filter_hit;    // Filter index that matched (RX only)
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

struct BusErrorState {
    bool bus_off;
    bool tx_error_passive;
    bool rx_error_passive;
    bool tx_error_warning;
    bool rx_error_warning;
    bool error_warning;
    uint8_t tx_error_count;
    uint8_t rx_error_count;
};

struct Status {
    uint32_t interrupt_flags;
    uint32_t rx_if;
    uint32_t tx_if;
    uint32_t rx_overflow_if;
    uint32_t tx_attempt_if;
    BusErrorState errors;
    uint32_t bdiag0;  // CiBDIAG0 - bus diagnostic register 0
    uint32_t bdiag1;  // CiBDIAG1 - bus diagnostic register 1
    uint32_t crc;
    uint32_t spi_crc_errors;      
    uint32_t ecc_double_errors;  
};

struct FifoStatus {
    uint8_t fifo_index;
    bool tx_aborted;
    bool tx_lost_arbitration;
    bool tx_error;
    bool tx_attempts_exhausted;
    bool rx_overflow;
    bool empty_or_full;
    bool half_empty_or_half_full;
    bool not_full_or_not_empty;
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
    spi_inst_t* spi_;
    uint chipSelectPin_;
    uint standbyPin_;
    uint intPin_; // int0Pin_, int1Pin_ unused
    WriteMode writeMode_ = WriteMode::WM_MCP_NORM;
    ReadMode readMode_   = ReadMode::RM_MCP_NORM;

    InitConfig config_{};

    DriverState state_ = DriverState::Uninitialized;

    uint32_t spiCrcErrors_    = 0; // counted by crcCheckPassed, reported in Status
    uint32_t eccDoubleErrors_ = 0; // counted by eccCheckPassed, reported in Status

    FifoInfo fifoInfo[31];

    // spi reset - resets all registers to default values
    int reset();

    void csSelect() const;
    void csDeselect() const;

    // Helper functions
    int readReg(uint16_t addr, uint32_t* dst);
    int writeReg(uint16_t addr, uint32_t value);
    int readOpMode(uint8_t* opmod);
    int waitForOpMode(ControllerMode target);
    int pollRegisterBit(uint16_t addr, uint8_t mask, bool wantSet);
    int updateByte(uint16_t addr, uint8_t field, uint8_t value);
    int checkFifo(uint8_t fifoNum, bool wantTx) const;
    int readFifoUserAddress(uint8_t fifoNum, size_t slotSize, uint16_t* messageAddr);
    
    int configureDevice(const InitConfig& config);

    int initGeneralPurposeFifo(uint8_t fifoNum, FifoMode fifoMode, FifoPayloadSize plSize,
                                 uint8_t fSize, uint8_t prioNum, TxRetransmitMode retranMode,
                                 uint8_t intFlags);

    // checks whether configuring fifoNum still fits the 2 KB message RAM.
    int fifoRamFits(uint8_t fifoNum, bool isTx) const;

    // route one standard-ID filter to a FIFO and enable it
    int initFilter(uint8_t fltNum, uint8_t fifoNum, uint16_t canSID, uint16_t sidMask);

    // turn off one filter
    int disableFilter(uint8_t fltNum);

    // true if an RX FIFO other than exceptFifo already routes filter fltNum
    bool filterInUse(uint8_t fltNum, uint8_t exceptFifo) const;

    // turn off the FIFO's filters if it is RX, then clear the FIFO's fifoInfo entry.
    int releaseFifo(uint8_t fifoNum);

    int setControllerMode(ControllerMode mode);
    void setTransceiverMode(TransceiverMode mode) const;

    // blocking SPI read/write transactions
    // during runtime, they effectively only fail when there is a CRC mismatch
    int readAddr(uint16_t startAddr, uint8_t* dst, size_t len);
    int writeAddr(uint16_t startAddr, const uint8_t* data, size_t len);
    // writeAddr without the CRC flag check
    int writeAddrUnchecked(uint16_t startAddr, const uint8_t* data, size_t len);
    // reads and clears CRCERRIF/FERRIF, counts crc errors; false if either was set
    bool crcCheckPassed();
    // reads and clears DEDIF/SECIF, counts double errors; false if a double error was seen
    bool eccCheckPassed();

    // write the nominal and data bit timing; only valid in config mode
    int setBitTiming(BitTiming nominalTiming, BitTiming dataTiming);

   public:
    // make a driver bound to SPI and the pins
    MCP251863(spi_inst_t* ispi, uint iCSPin, uint iSTBYPin, uint iINTPin);

    // initialize the controller with defaults
    int init();
    // initialize controller with a caller-provided timing
    int init(const InitConfig& config);

    // true while nINT is low: an RX frame is waiting or an RX overflow has occurred
    bool interruptPending() const;

    // which RX FIFOs have a frame waiting (C1RXIF)
    // plus which RX FIFOs have overflowed (C1RXOVIF)
    // bit n = FIFO n
    int pendingRxFifos(uint32_t* rxMask, uint32_t* overflowMask);

    // init() does NOT configure any FIFOs. Must call these functions
    // to configure one more general-purpose FIFO after init()
    // fifoNum range: 0 - 30, fltNum range: 0 - 31
    // higher number = higher priority to send msgs onto bus
    int configureTxFifo(uint8_t fifoNum, uint8_t prioNum = 0);
    int configureRxFifo(uint8_t fifoNum, uint8_t fltNum, uint16_t canSID,
                          uint16_t sidMask = 0x7FF);
    // suscribes the RX FIFO to an ID
    // if 2 RX FIFOs suscribe to the same ID, the FIFO with
    // the lower fitNum has higher priority (msg would be passed
    // into *that* FIFO)
    int configureRxFifo(uint8_t fifoNum, uint16_t canSID);
    // map 2 discrete IDs to an RX FIFO, using filters fifoNum and 31 - fifoNum
    int configureRxFifoTwoIds(uint8_t fifoNum, uint16_t canSID1, uint16_t canSID2);

    // turns off the FIFO's filter and marks it unusable in the driver
    // before begin(), it shrinks the FIFO to 1 message to free up RAM.
    // After begin(), it just empties the FIFO (FRESET)
    int disableFifo(uint8_t fifoNum);

    // after init() is called, device is still in configuration mode
    // so user can freely configure FIFOs. After configuring FIFOs,
    // user must call begin() to enter normal mode
    int begin();

    // queue a CAN FD frame on a TX FIFO and request it be sent
    // returns 0 when DriverState != Running, invalid FIFO, SPI error, or the frame is 
    // longer than the FIFO's payload size. Returns 1 otherwise
    // if FIFO was full, *queued is false and nothing is written to FIFO
    int send_canfd(uint8_t fifoNum, uint32_t id, const uint8_t* data, size_t len, bool brs,
                   bool* queued);

    // pop the oldest frame from an RX FIFO
    // returns 0 under same conditions as send_canfd
    // if FIFO was empty, *received is false and nothing is put into *frame
    int read_frame(uint8_t fifoNum, CanFdFrame* frame, bool* received);

    // configure INT0/INT1 as interrupt pins or GPIOs
    int setPinMode(IoPin pin, IoMode mode);

    // read controller-wide interrupt, error, and SPI CRC status registers (9 SPI reads)
    int getStatus(Status* status);

    // read only the bus error state (1 SPI read)
    int getBusErrors(BusErrorState* errors);

    // read status flags for one FIFO
    int getFIFOStatus(uint8_t fifoNum, FifoStatus* status);

    int clearRxOverflow(uint8_t fifoNum);
};

#endif
