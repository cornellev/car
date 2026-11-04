#include <cstdint>
#include <cstdio>

#include "hardware/spi.h"
#include "mcp251863.hpp"
#include "pico/stdlib.h"

static constexpr uint SPI_SCK  = 22;
static constexpr uint SPI_TX   = 23;
static constexpr uint SPI_RX   = 20;
static constexpr uint CS_PIN   = 21;
static constexpr uint STBY_PIN = 24;
static constexpr uint INT_PIN  = 25;
static constexpr uint INT0_PIN = 26;
static constexpr uint INT1_PIN = 27;

int main() {
    stdio_init_all();
    sleep_ms(10000);
    printf("TX example starting\n");

    spi_init(spi0, MCP251863_BAUD_RATE);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI_TX, GPIO_FUNC_SPI);
    gpio_set_function(SPI_RX, GPIO_FUNC_SPI);

    MCP251863 mcp(spi0, CS_PIN, STBY_PIN, INT_PIN, INT0_PIN, INT1_PIN);

    if (mcp.init() != Error::None) {
        printf("init failed\n");
        return 1;
    }
    printf("init ok\n");

    mcp.configureTxFifo(0);
    mcp.configureTxFifo(1);
    mcp.configureTxFifo(2);  // make them all have the same priority

    mcp.begin();

    uint8_t data0[8]   = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    uint8_t data1[8]   = {0x00, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E};
    uint8_t data2[8]   = {0x00, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
    const uint32_t id0 = 0x001;
    const uint32_t id1 = 0x002;
    const uint32_t id2 = 0x003;
    int count          = 0;

    printf("Make it to here.\n");

    for (;;) {
        data0[0] = (uint8_t)(count & 0xFF);
        data1[0] = (uint8_t)(count & 0xFF);
        data2[0] = (uint8_t)(count & 0xFF);

        // Queue FIFO 0 msg & request send
        while (mcp.start_push_canfd(0, id0, data0, sizeof(data0), true) != Error::None) {
            sleep_ms(100);
        }
        while (mcp.poll_push() == Error::Busy) {
            sleep_ms(100);
        }
        mcp.clear_send();
        if (mcp.request_send(0) == Error::None) {
            printf("sent FIFO0 frame %d id = 0x%03X data[0]=%02X\n", count, id0, data0[0]);
        }

        // Queue FIFO 1 msg & request send
        while (mcp.start_push_canfd(1, id1, data1, sizeof(data1), true) != Error::None) {
            sleep_ms(100);
        }
        while (mcp.poll_push() == Error::Busy) {
            sleep_ms(100);
        }
        mcp.clear_send();
        if (mcp.request_send(1) == Error::None) {
            printf("sent FIFO1 frame %d id = 0x%03X data[0]=%02X\n", count, id1, data1[0]);
        }

        // Queue FIFO 2 msg & request send
        while (mcp.start_push_canfd(2, id2, data2, sizeof(data2), true) != Error::None) {
            sleep_ms(100);
        }
        while (mcp.poll_push() == Error::Busy) {
            sleep_ms(100);
        }
        mcp.clear_send();
        if (mcp.request_send(2) == Error::None) {
            printf("sent FIFO2 frame %d id = 0x%03X data[0]=%02X\n", count, id2, data2[0]);
        }

        // Polling information
        if (count % 10 == 0) {
            const Status s = mcp.getStatus().value();
            printf("bus_off=%d tx_err=%d tx_count=%d\n", s.bus_off, s.tx_error_passive,
                   s.tx_error_count);
        }

        count++;
        sleep_ms(500);
    }

    return 0;
}
