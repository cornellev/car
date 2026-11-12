
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
    sleep_ms(5000);
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

    while (mcp.configureRxFifo(0, 0x123) != Error::None) {
        sleep_ms(100);
    }

    mcp.enableRxInterrupts();

    mcp.begin();

    int received = 0;

    for (;;) {
        if (mcp.rxInterruptPending()) {
            mcp.clearRxInterruptPending();

            const Error err = mcp.start_pop_canfd(0);
            if (err == Error::None) {
                while (mcp.poll_pop() == Error::Busy) {
                    tight_loop_contents();
                }
                const CanFdFrame frame = mcp.get_read_frame().value();
                if (frame.valid) {
                    received++;
                    printf("Interrupt fired: [%d] ", received);
                    print_frame(frame);
                }
            }
        }

        sleep_us(100);  // yield briefly, not blocking
    }

    return 0;
}