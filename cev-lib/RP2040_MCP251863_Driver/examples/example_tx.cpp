#include <stdio.h>

#include "hardware/spi.h"
#include "mcp251863.hpp"
#include "pico/stdlib.h"

static constexpr uint SPI_SCK  = 22;
static constexpr uint SPI_TX   = 23;
static constexpr uint SPI_RX   = 20;
static constexpr uint CS_PIN   = 21;
static constexpr uint STBY_PIN = 24;
static constexpr uint INT_PIN  = 25;

int main() {
    stdio_init_all();
    sleep_ms(5000);
    printf("TX example starting\n");

    spi_init(spi0, MCP251863_BAUD_RATE);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI_TX, GPIO_FUNC_SPI);
    gpio_set_function(SPI_RX, GPIO_FUNC_SPI);

    MCP251863 mcp(spi0, CS_PIN, STBY_PIN, INT_PIN);

    if (!mcp.init()) {
        printf("init failed\n");
        return 1;
    }
    printf("init ok\n");

    if (!mcp.configureTxFifo(0)) {
        printf("configureTxFifo failed\n");
        return 1;
    }
    if (!mcp.begin()) {
        printf("begin failed\n");
        return 1;
    }

    uint8_t data0[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    uint8_t data1[8] = {0x00, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E};
    uint8_t data2[8] = {0x00, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
    uint32_t id0     = 0x001;
    uint32_t id1     = 0x002;
    uint32_t id2     = 0x003;
    int count        = 0;

    uint8_t* data = nullptr;
    uint32_t id   = 0;
    size_t len = sizeof(data0);

    for (;;) {
        if (count % 3 == 0) {
            data0[0] = (uint8_t)(count & 0xFF);
            data     = data0;
            id       = id0;
        } else if (count % 3 == 1) {
            data1[0] = (uint8_t)(count & 0xFF);
            data     = data1;
            id       = id1;
        } else {
            data2[0] = (uint8_t)(count & 0xFF);
            data     = data2;
            id       = id2;
        }

        bool queued = false;
        if (!mcp.send_canfd(0, id, data, len, true, &queued)) {
            printf("Send failed.\n");
        } else if (!queued) {  
            printf("TX FIFO was full.\n");
        }

        // Polling information
        if (count % 10 == 0) {
            BusErrorState e{};
            if (mcp.getBusErrors(&e)) {
                printf("bus_off=%d tx_passive=%d tx_count=%d\n", e.bus_off, e.tx_error_passive,
                       e.tx_error_count);
            } else {
                printf("getBusErrors failed\n");
            }
        }

        count++;
        sleep_ms(500);
    }
    return 0;
}
