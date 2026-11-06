#include <cstdint>
#include <cstdio>

#include "config.hpp"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

// configure one RPM gpio as a PWM B-input edge counter
static void rpm_init_one(unsigned gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    const unsigned slice = pwm_gpio_to_slice_num(gpio);
    pwm_config cfg       = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING);
    pwm_config_set_clkdiv(&cfg, 1.f);
    pwm_init(slice, &cfg, false);
    pwm_set_wrap(slice, 0xffff);
    pwm_set_counter(slice, 0);
    pwm_set_enabled(slice, true);
}

// read and reset one RPM counter
static std::uint16_t rpm_take(unsigned gpio) {
    const unsigned slice  = pwm_gpio_to_slice_num(gpio);
    const std::uint16_t n = pwm_get_counter(slice);
    pwm_set_counter(slice, 0);
    return n;
}

// bring-up: print raw ADC counts, converted voltages, and RPM edge counts over USB serial
int main() {
    stdio_init_all();

    adc_init();
    adc_gpio_init(kSteeringGpio);
    adc_gpio_init(kBrakeGpio);

    for (const unsigned gpio : kRpmGpio) {
        rpm_init_one(gpio);
    }

    // wait for a serial monitor to connect before printing anything
    while (!stdio_usb_connected()) {
        sleep_ms(10);
    }

    // column header
    std::printf("\n%8s  %4s  %5s  %5s    %4s  %5s  %5s   %5s %5s %5s %5s\n",
                "time_ms",
                "s_raw", "s_vpin", "s_vsen",
                "b_raw", "b_vpin", "b_vsen",
                "rpm1", "rpm2", "rpm3", "rpm4");

    std::uint32_t next_ms = kPrintPeriodMs;

    while (true) {
        const std::uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now < next_ms) {
            continue;
        }
        next_ms += kPrintPeriodMs;

        // snapshot all four RPM counters before spending time on ADC reads
        const std::uint16_t rpm[4] = {
            rpm_take(kRpmGpio[0]),
            rpm_take(kRpmGpio[1]),
            rpm_take(kRpmGpio[2]),
            rpm_take(kRpmGpio[3]),
        };

        adc_select_input(kSteeringAdcCh);
        const std::uint16_t s_raw = adc_read();
        const float s_vpin        = s_raw * 3.3f / 4095.f;
        const float s_vsens       = s_vpin / kDividerRatio;

        adc_select_input(kBrakeAdcCh);
        const std::uint16_t b_raw = adc_read();
        const float b_vpin        = b_raw * 3.3f / 4095.f;
        const float b_vsens       = b_vpin / kDividerRatio;

        std::printf("%8lu  %4u  %5.3f  %5.3f    %4u  %5.3f  %5.3f   %5u %5u %5u %5u\n",
                    static_cast<unsigned long>(now),
                    s_raw, s_vpin, s_vsens,
                    b_raw, b_vpin, b_vsens,
                    rpm[0], rpm[1], rpm[2], rpm[3]);
    }
}
