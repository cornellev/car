#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>

#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#include "cobs.hpp"
#include "common/interval.hpp"
#include "common/rpm.hpp"
#include "steering_motion.hpp"
#include "steering_homing.hpp"
#include "steering_pulses.hpp"

namespace {

// ---- pin map ----

struct DrivetrainMotor {
    uint pwm;
    uint fwd;
    uint back;
};

struct SteeringMotor {
    uint pulse;
    uint dir;
    uint enable;
};

constexpr std::array drivetrain{
    DrivetrainMotor{.pwm = 6, .fwd = 8, .back = 9},
    DrivetrainMotor{.pwm = 10, .fwd = 12, .back = 13},
};

constexpr SteeringMotor steering{.pulse = 14, .dir = 15, .enable = 16};

constexpr uint ONBOARD_LED_PIN = 25;

// wheel-speed sensors, counted by pio (see common/rpm.pio)
constexpr uint RPM_LEFT_PIN = 23;
constexpr uint RPM_RIGHT_PIN = 24;
constexpr float WHEEL_EDGES_PER_REV = 68.0f;  // magnets per wheel, from the old Merceves_Base firmware
constexpr uint32_t TELEMETRY_PERIOD_MS = 100;
// rps is averaged over the last few periods: each edge in the window is worth
// 1 / (68 * 0.5 s) ~= 0.029 rps, so a longer window reads smoother but lags more
constexpr size_t RPS_WINDOW_PERIODS = 5;

constexpr float DRIVETRAIN_PWM_FREQ_HZ = 2'000.0f;
constexpr float STEPPER_MAX_FREQ_HZ = 17'500.0f;
constexpr int32_t STEERING_HALF_RANGE_STEPS = 800;  // pulses from center to either limit
constexpr float STEPPER_ACCELERATION = 40'000.0f;  // steps/s^2; applies to acceleration and braking
constexpr uint STEPPER_PULSE_US = 3;  // DM542T: pulse high and low must each last at least 2.5 us
constexpr uint STEPPER_DIR_SETUP_US = 5;     // DM542T: dir must lead the pulse edge by at least 5 us
constexpr uint STEPPER_ENABLE_SETUP_US = 5;  // DM542T: enable must lead dir by at least 5 us
// the DM542T ignores pulses for a while after it's enabled, longer than its 5 us setup time.
// not measured: if the first move after startup or a timeout still comes up short, raise it
constexpr uint32_t STEPPER_ENABLE_SETTLE_MS = 200;

constexpr uint STEERING_PWM_CLKDIV = SYS_CLK_HZ / 1'000'000;  // 1 count = 1 us, as SteeringPulses expects
constexpr uint16_t STEERING_IDLE_US = 1'000;        // while holding, re-plan every 1 ms
constexpr uint16_t STEERING_MAX_PERIOD_US = 65'000;  // longest step interval the 16-bit counter allows

static_assert(STEERING_HALF_RANGE_STEPS > 0);
static_assert(STEPPER_ACCELERATION > 0.0f);
static_assert(STEPPER_MAX_FREQ_HZ > 0.0f);
static_assert(STEPPER_MAX_FREQ_HZ * (2 * STEPPER_PULSE_US) < 1'000'000.0f);
static_assert(SYS_CLK_HZ % 1'000'000 == 0 && STEERING_PWM_CLKDIV <= 255, "steering PWM needs a whole-us clock");
// dir is set at the wrap that starts a pulse's period, so the shortest period minus the pulse
// is the least setup time it gets. enable goes on a whole settle time before the first pulse
static_assert(1'000'000.0f / STEPPER_MAX_FREQ_HZ - STEPPER_PULSE_US >= STEPPER_DIR_SETUP_US);
static_assert(STEPPER_ENABLE_SETTLE_MS * 1000 >= STEPPER_ENABLE_SETUP_US);

constexpr uint32_t DRIVETRAIN_WRAP = static_cast<uint32_t>(SYS_CLK_HZ / DRIVETRAIN_PWM_FREQ_HZ) - 1;
static_assert(DRIVETRAIN_WRAP <= std::numeric_limits<uint16_t>::max(),
              "drivetrain PWM frequency too low for clkdiv = 1");

// shared between the main loop and the steering interrupt; see SteeringIrqBlock
SteeringMotion steering_motion(STEERING_HALF_RANGE_STEPS, STEPPER_MAX_FREQ_HZ, STEPPER_ACCELERATION);
SteeringPulses steering_pulses(steering_motion, STEPPER_PULSE_US, STEERING_IDLE_US, STEERING_MAX_PERIOD_US);

// keeps the steering interrupt out while the main loop changes state it shares. don't nest
class SteeringIrqBlock {
public:
    SteeringIrqBlock() { irq_set_enabled(PWM_DEFAULT_IRQ_NUM(), false); }
    ~SteeringIrqBlock() { irq_set_enabled(PWM_DEFAULT_IRQ_NUM(), true); }
    SteeringIrqBlock(const SteeringIrqBlock&) = delete;
    SteeringIrqBlock& operator=(const SteeringIrqBlock&) = delete;
};

bool steering_pwm_running() {
    return pwm_hw->slice[pwm_gpio_to_slice_num(steering.pulse)].csr & PWM_CH0_CSR_EN_BITS;
}

// runs at every wrap of the steering PWM slice, right as that period's pulse (if any) ends.
// it has until the next wrap (at least 1 / STEPPER_MAX_FREQ_HZ) to load the period after
void on_steering_wrap() {
    const uint slice = pwm_gpio_to_slice_num(steering.pulse);
    pwm_clear_irq(slice);

    const SteeringPulses::Wrap wrap = steering_pulses.on_wrap();
    if (wrap.direction_now != 0) {
        gpio_put(steering.dir, wrap.direction_now < 0);
    }
    pwm_set_wrap(slice, wrap.next.top);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(steering.pulse), wrap.next.level);

    if (steering_pulses.drained()) {
        pwm_set_enabled(slice, false);
        gpio_put(steering.enable, 0);
    }
}

// pulse comes from the PWM, one period per step (see SteeringPulses); dir and enable are plain outputs
void init_motor(const SteeringMotor& motor) {
    for (const uint pin : {motor.enable, motor.dir}) {
        gpio_init(pin);
        gpio_put(pin, 0);
        gpio_set_dir(pin, GPIO_OUT);
    }

    const uint slice = pwm_gpio_to_slice_num(motor.pulse);
    hard_assert(slice < NUM_PWM_SLICES);
    const SteeringPulses::Period idle = steering_pulses.idle();
    const bool on_a = pwm_gpio_to_channel(motor.pulse) == PWM_CHAN_A;
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&cfg, STEERING_PWM_CLKDIV);
    pwm_config_set_wrap(&cfg, idle.top);
    pwm_config_set_output_polarity(&cfg, on_a, !on_a);  // high once the counter reaches level, so pulses end at the wrap
    pwm_init(slice, &cfg, false);
    pwm_set_gpio_level(motor.pulse, idle.level);
    gpio_set_function(motor.pulse, GPIO_FUNC_PWM);

    pwm_clear_irq(slice);
    pwm_set_irq_enabled(slice, true);
    irq_set_exclusive_handler(PWM_DEFAULT_IRQ_NUM(), on_steering_wrap);
    irq_set_priority(PWM_DEFAULT_IRQ_NUM(), PICO_HIGHEST_IRQ_PRIORITY);  // ahead of usb, so it never misses a wrap
    irq_set_enabled(PWM_DEFAULT_IRQ_NUM(), true);
}

// Caller holds SteeringIrqBlock while changing shared pulse state.
void run_steering() {
    if (!gpio_get_out_level(steering.enable)) {
        // enable the driver right away, even for a centered target, and hold off stepping until it
        // settles. it then stays enabled to hold position until a timeout stops steering
        gpio_put(steering.enable, 1);
        steering_pulses.resume(STEPPER_ENABLE_SETTLE_MS * 1000 / STEERING_IDLE_US);
    } else {
        steering_pulses.resume();
    }
    if (!steering_pwm_running()) {
        // both registers hold an idle period, which is where SteeringPulses starts from
        const uint slice = pwm_gpio_to_slice_num(steering.pulse);
        const SteeringPulses::Period idle = steering_pulses.idle();
        pwm_set_wrap(slice, idle.top);
        pwm_set_gpio_level(steering.pulse, idle.level);
        pwm_set_counter(slice, 0);
        pwm_clear_irq(slice);
        pwm_set_enabled(slice, true);
    }
}

void steer_to(float position) {
    const SteeringIrqBlock block;
    steering_motion.set_target(position);
    run_steering();
}

// the at most two pulses already loaded into the PWM still go out, then the interrupt
// disables the driver, so the counted position stays exact
void stop_steering() {
    const SteeringIrqBlock block;
    steering_pulses.stop();
}

// pwm is fixed frequency, variable duty. starts forward at 0% duty
void init_motor(const DrivetrainMotor& motor) {
    gpio_init(motor.fwd);
    gpio_put(motor.fwd, 1);
    gpio_set_dir(motor.fwd, GPIO_OUT);

    gpio_init(motor.back);
    gpio_put(motor.back, 0);
    gpio_set_dir(motor.back, GPIO_OUT);

    const uint slice = pwm_gpio_to_slice_num(motor.pwm);
    // The shared SDK mapper can return slices beyond this chip's PWM array.
    hard_assert(slice < NUM_PWM_SLICES);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, DRIVETRAIN_WRAP);
    pwm_init(slice, &cfg, false);
    pwm_set_gpio_level(motor.pwm, 0);
    gpio_set_function(motor.pwm, GPIO_FUNC_PWM);
    pwm_set_enabled(slice, true);
}

// [-1.0, 1.0]; the sign picks fwd or back (always opposite), the magnitude maps linearly onto duty
void set_speed(const DrivetrainMotor& motor, float speed) {
    speed = std::clamp(speed, -1.0f, 1.0f);
    const bool reverse = speed < 0.0f;

    // cut power before swapping direction, and write both pins at once so they're never equal
    if (gpio_get_out_level(motor.back) != reverse) {
        pwm_set_gpio_level(motor.pwm, 0);
        gpio_put_masked((1u << motor.fwd) | (1u << motor.back), reverse ? 1u << motor.back : 1u << motor.fwd);
    }

    // level = wrap + 1 holds the output high the whole period
    pwm_set_gpio_level(motor.pwm, static_cast<uint16_t>(std::abs(speed) * (DRIVETRAIN_WRAP + 1)));
}

void stop_all() {
    stop_steering();
    for (const DrivetrainMotor& motor : drivetrain) {
        set_speed(motor, 0.0f);
    }
}

// ---- AS5600 encoder: homing and telemetry, powered from Pico 3V3 OUT ----
constexpr uint ENCODER_SDA_PIN = 2;  // I2C1, physical pin 4
constexpr uint ENCODER_SCL_PIN = 3;  // I2C1, physical pin 5
constexpr uint ENCODER_VCC_PIN = 4;  // Power, physical pin 6
constexpr uint ENCODER_I2C_HZ = 100'000;
constexpr uint ENCODER_TIMEOUT_US = 1'000;
constexpr uint8_t ENCODER_ADDRESS = 0x36;

struct EncoderSample {
    uint16_t raw = 0;
    uint8_t status = 0;
    bool read_ok = false;  // Transport success, independent of magnetic-field warnings.
};

void init_encoder() {
    i2c_init(i2c1, ENCODER_I2C_HZ);
    gpio_set_function(ENCODER_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(ENCODER_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(ENCODER_SDA_PIN);
    gpio_pull_up(ENCODER_SCL_PIN);
    gpio_init(ENCODER_VCC_PIN);
    gpio_set_dir(ENCODER_VCC_PIN, GPIO_OUT);
    gpio_put(ENCODER_VCC_PIN, 1);
}

EncoderSample read_encoder() {
    constexpr uint8_t status_register = 0x0B;
    std::array<uint8_t, 3> data{};  // STATUS, RAW_ANGLE high, RAW_ANGLE low
    if (i2c_write_timeout_us(i2c1, ENCODER_ADDRESS, &status_register, 1, true,
                             ENCODER_TIMEOUT_US) != 1 ||
        i2c_read_timeout_us(i2c1, ENCODER_ADDRESS, data.data(), data.size(), false,
                            ENCODER_TIMEOUT_US) != static_cast<int>(data.size())) return {};
    return {.raw = static_cast<uint16_t>(((data[1] & 0x0F) << 8) | data[2]),
            .status = data[0], .read_ok = true};
}

SteeringHoming homing;
SteeringAngle steering_angle;
EncoderSample latest_encoder;

void update_homing() {
    const EncoderSample sample = latest_encoder;
    const auto previous_state = homing.state;
    const bool valid = sample.read_ok && (sample.status & 0x20);
    const SteeringIrqBlock block;
    const int direction = homing.update(to_ms_since_boot(get_absolute_time()), sample.raw,
                                        valid, steering_pulses.drained());
    if (direction != 0) {
        steering_pulses.jog(direction);
        run_steering();
    } else {
        steering_pulses.stop();
        if (homing.state == SteeringHoming::State::ready) {
            steering_pulses.zero();
            if (previous_state != SteeringHoming::State::ready)
                steering_angle.reference(sample.raw, to_ms_since_boot(get_absolute_time()));
        }
    }
}

// ---- serial protocol ----
// every frame is COBS-encoded and ends in a 0x00 byte, so a dropped byte only loses one frame
// decoded frame: [type u8][payload], all multi-byte values are little-endian
//   jetson -> pico  MSG_DRIVE      0x01  f32 steering position [-1, 1], f32 drivetrain speed [-1, 1]
// steering: -1 = -half range, 0 = homed center, +1 = +half range
//   pico -> jetson  MSG_TELEMETRY  0x02  f32 left rps, f32 right rps, i32 steering pulses, u16 encoder raw, u8 status, u8 read_ok, f32 steering degrees (positive left, NaN until homed/invalid)
// every TELEMETRY_PERIOD_MS; encoder raw/status are zero on I2C failure
// steering pulses count emitted steps relative to the latest homed center (boot until homing), signed by direction
// rps is always >= 0: the sensors count magnets going by, not which way the wheel turns

static_assert(std::endian::native == std::endian::little, "frames are little-endian, same as the RP2040");

constexpr uint8_t MSG_DRIVE = 0x01;
constexpr uint8_t MSG_TELEMETRY = 0x02;
constexpr size_t DRIVE_FRAME_LEN = 1 + 2 * sizeof(float);
constexpr size_t TELEMETRY_FRAME_LEN = 1 + 2 * sizeof(float) + sizeof(int32_t) + 4 + sizeof(float);
constexpr size_t MAX_ENCODED_LEN = 32;     // longer frames are garbage and get dropped
constexpr uint32_t COMMAND_TIMEOUT_MS = 200;  // stop every motor if no valid command arrives for this long

float read_f32(std::span<const uint8_t, sizeof(float)> bytes) {
    std::array<uint8_t, sizeof(float)> raw;
    std::ranges::copy(bytes, raw.begin());
    return std::bit_cast<float>(raw);
}

void write_f32(std::span<uint8_t, sizeof(float)> bytes, float value) {
    std::ranges::copy(std::bit_cast<std::array<uint8_t, sizeof(float)>>(value), bytes.begin());
}

// a sliding window over the last RPS_WINDOW_PERIODS edge counts
class RpsWindow {
public:
    struct Rps {
        float left;
        float right;
    };

    // adds one period's counts and returns the rps over the whole window
    Rps add(cev::RpmCounts edges, float seconds) {
        periods_[next_] = {edges.left, edges.right, seconds};
        next_ = (next_ + 1) % periods_.size();

        // periods not filled in yet are all zeros, so they don't count toward either sum
        uint32_t left = 0;
        uint32_t right = 0;
        float total_seconds = 0.0f;
        for (const Period& period : periods_) {
            left += period.left;
            right += period.right;
            total_seconds += period.seconds;
        }
        return {to_rps(left, total_seconds), to_rps(right, total_seconds)};
    }

private:
    struct Period {
        uint32_t left = 0;
        uint32_t right = 0;
        float seconds = 0.0f;
    };

    static float to_rps(uint32_t edges, float seconds) {
        return static_cast<float>(edges) / WHEEL_EDGES_PER_REV / seconds;
    }

    std::array<Period, RPS_WINDOW_PERIODS> periods_{};
    size_t next_ = 0;
};

// if the host isn't reading, the frame (or its end) is dropped instead of blocking; see CMakeLists.txt
void send_telemetry(float left_rps, float right_rps) {
    std::array<uint8_t, TELEMETRY_FRAME_LEN> frame{MSG_TELEMETRY};
    write_f32(std::span(frame).subspan<1, sizeof(float)>(), left_rps);
    write_f32(std::span(frame).subspan<1 + sizeof(float), sizeof(float)>(), right_rps);
    int32_t position;
    {
        const SteeringIrqBlock block;
        position = steering_pulses.position();
    }
    std::ranges::copy(std::bit_cast<std::array<uint8_t, sizeof(int32_t)>>(position),
                      frame.begin() + 1 + 2 * sizeof(float));

    const EncoderSample encoder = latest_encoder;
    constexpr size_t encoder_offset = 1 + 2 * sizeof(float) + sizeof(int32_t);
    frame[encoder_offset] = static_cast<uint8_t>(encoder.raw & 0xFF);
    frame[encoder_offset + 1] = static_cast<uint8_t>(encoder.raw >> 8);
    frame[encoder_offset + 2] = encoder.status;
    frame[encoder_offset + 3] = static_cast<uint8_t>(encoder.read_ok);

    write_f32(std::span(frame).subspan<17, sizeof(float)>(), steering_angle.degrees());

    std::array<uint8_t, cobs::max_encoded_size(TELEMETRY_FRAME_LEN) + 1> encoded;
    size_t len = cobs::encode(frame, encoded);
    encoded[len++] = 0;

    // raw, since stdio would turn any 0x0A byte into 0x0D 0x0A
    stdio_put_string(reinterpret_cast<const char*>(encoded.data()), static_cast<int>(len), false, false);
}

// returns true if the frame was a valid drive command, which is then applied
[[nodiscard]] bool handle_frame(std::span<const uint8_t> encoded) {
    std::array<uint8_t, MAX_ENCODED_LEN> frame;
    const std::optional<size_t> len = cobs::decode(encoded, frame);
    if (len != DRIVE_FRAME_LEN || frame[0] != MSG_DRIVE) {
        return false;
    }

    const auto payload = std::span(frame).first<DRIVE_FRAME_LEN>();
    const float steering_position = read_f32(payload.subspan<1, sizeof(float)>());
    const float drivetrain_speed = read_f32(payload.subspan<1 + sizeof(float), sizeof(float)>());
    if (!std::isfinite(steering_position) || !std::isfinite(drivetrain_speed)) {
        return false;
    }

    if (homing.state == SteeringHoming::State::idle) {
        steering_angle.reset();
        homing.start(to_ms_since_boot(get_absolute_time()));
    }
    if (homing.state != SteeringHoming::State::ready) return true;

    steer_to(steering_position);
    for (const DrivetrainMotor& motor : drivetrain) {
        set_speed(motor, drivetrain_speed);
    }
    return true;
}

}  // namespace

int main() {
    stdio_init_all();

    gpio_init(ONBOARD_LED_PIN);
    gpio_set_dir(ONBOARD_LED_PIN, GPIO_OUT);

    init_motor(steering);
    for (const DrivetrainMotor& motor : drivetrain) {
        init_motor(motor);
    }
    stop_all();

    init_encoder();
    cev::rpm_init(RPM_LEFT_PIN, RPM_RIGHT_PIN);
    cev::Interval telemetry{.period_ms = TELEMETRY_PERIOD_MS};
    absolute_time_t last_telemetry = get_absolute_time();
    RpsWindow rps_window;

    std::array<uint8_t, MAX_ENCODED_LEN> rx;
    size_t rx_len = 0;
    bool rx_overflow = false;
    absolute_time_t last_command = get_absolute_time();
    bool timed_out = true;
    cev::Interval encoder_poll{.period_ms = 5};

    while (true) {
        const int c = getchar_timeout_us(0);
        if (c >= 0) {
            if (c == 0) {
                if (!rx_overflow && handle_frame(std::span(rx).first(rx_len))) {
                    last_command = get_absolute_time();
                    timed_out = false;
                }
                rx_len = 0;
                rx_overflow = false;
            } else if (rx_len < rx.size()) {
                rx[rx_len++] = static_cast<uint8_t>(c);
            } else {
                rx_overflow = true;
            }
        }

        if (!timed_out && absolute_time_diff_us(last_command, get_absolute_time()) > COMMAND_TIMEOUT_MS * 1000) {
            stop_all();
            timed_out = true;
            homing.reset();
        }

        if (encoder_poll.due()) {
            latest_encoder = read_encoder();
            steering_angle.update(latest_encoder.raw,
                                  latest_encoder.read_ok && (latest_encoder.status & 0x20),
                                  to_ms_since_boot(get_absolute_time()));
            if (!timed_out && homing.active()) update_homing();
        }

        // sent whether or not commands are arriving, so wheel speed can be read with the motors off.
        // divides by the real elapsed time, in case the loop ran late
        if (telemetry.due()) {
            const absolute_time_t now = get_absolute_time();
            const float seconds = static_cast<float>(absolute_time_diff_us(last_telemetry, now)) / 1e6f;
            last_telemetry = now;
            const RpsWindow::Rps rps = rps_window.add(cev::rpm_take(), seconds);
            send_telemetry(rps.left, rps.right);
        }

        // led on while commands are arriving
        gpio_put(ONBOARD_LED_PIN, !timed_out);
    }
}
