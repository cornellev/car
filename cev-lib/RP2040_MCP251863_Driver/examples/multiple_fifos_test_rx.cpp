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

namespace {

void print_frame(const CanFdFrame& frame) {
    printf("id=0x%03X  len=%d  fdf=%d  brs=%d  ide=%d  esi=%d  filter=%d  data=", frame.id,
           frame.len, frame.fdf, frame.brs, frame.ide, frame.esi, frame.filter_hit);
    for (int i = 0; i < frame.len; i++) {
        printf("%02X ", frame.data[i]);
    }
    printf("\n");
}

}  // namespace

int main() {
    stdio_init_all();
    sleep_ms(7000);
    printf("RX example starting\n");

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
    printf("init ok -- waiting for frames\n");

    while (mcp.configureRxFifo(0, 0x001) != Error::None) {
        sleep_ms(100);
    }
    while (mcp.configureRxFifo(1, 0x002) != Error::None) {
        sleep_ms(100);
    }
    while (mcp.configureRxFifo(2, 0x003) != Error::None) {
        sleep_ms(100);
    }

    mcp.begin();

    int received0 = 0;
    int received1 = 0;
    int received2 = 0;

    for (;;) {
        // Read frame from RX FIFO 0
        while (mcp.start_pop_canfd(0) != Error::None) {
            sleep_ms(100);
        }
        while (mcp.poll_pop() == Error::Busy) {
            // do something else!
            tight_loop_contents();
        }
        const CanFdFrame frame0 = mcp.get_read_frame().value();
        // Read frame from RX FIFO 1
        while (mcp.start_pop_canfd(1) != Error::None) {
            sleep_ms(100);
        }
        while (mcp.poll_pop() == Error::Busy) {
            // do something else!
            tight_loop_contents();
        }
        const CanFdFrame frame1 = mcp.get_read_frame().value();
        // Read frame from RX FIFO 2
        while (mcp.start_pop_canfd(2) != Error::None) {
            sleep_ms(100);
        }
        while (mcp.poll_pop() == Error::Busy) {
            // do something else!
            tight_loop_contents();
        }
        const CanFdFrame frame2 = mcp.get_read_frame().value();
        if (frame0.valid) {
            received0++;
            printf("FIFO 0: [%d] ", received0);
            print_frame(frame0);
        }
        if (frame1.valid) {
            received1++;
            printf("FIFO 1: [%d] ", received1);
            print_frame(frame1);
        }
        if (frame2.valid) {
            received2++;
            printf("FIFO 2: [%d] ", received2);
            print_frame(frame2);
        }
        // print status every 10 frames so you can spot errors accumulating
        if (received1 % 10 == 0) {
            const Status s = mcp.getStatus().value();
            printf("status: bus_off=%d  rx_err=%d  rx_overflow=0x%08X\n", s.bus_off,
                   s.rx_error_count, s.rx_overflow_if);
        }

        sleep_us(100);  // yield briefly, not blocking
    }

    return 0;
}