#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <mutex>
#include <string>
#include <thread>

#include "chuds/chuds.hpp"
#include "chuds/socketcan_transport.hpp"

constexpr float NANF             = std::numeric_limits<float>::quiet_NaN();
constexpr float wheel_diameter_m = 0.58166f;
constexpr float lambda           = 0.05f;
static uint64_t now_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
// Explicit deployment calibration; missing ADC calibration stays invalid.
static float setting(const char* name, float fallback) {
    const char* value = std::getenv(name);
    if (!value) return fallback;
    char* end{};
    float result = std::strtof(value, &end);
    if (end == value || *end || !std::isfinite(result)) {
        std::fprintf(stderr, "Invalid %s\n", name);
        std::exit(1);
    }
    return result;
}
#pragma pack(push, 1)
struct Power {
    uint32_t ts;
    float current;
    float voltage;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct Steering {  // Front steering and rear brake
    uint32_t ts;
    float brake_pressure;
    float turn_angle;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct RPM {
    uint32_t ts;
    float rpm_left;
    float rpm_right;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct GPS {  // All from GPS thread reading SIM7600
    uint32_t ts;
    float gps_lat;
    float gps_long;
    float heading;
    float speed;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct Motor {
    uint32_t ts;
    float rpm;
    float duty_cycle;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct Filtered {  // Filtered Data, currently just speed
    float speed;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct SensorSnapshot {  // 8 + 4+ 5 * 12 + 20 + 4 = 96 bytes
    uint64_t global_ts;
    uint32_t errcount;
    Power power_snap;
    Steering steering_snap;
    RPM rpm_snap_front;
    RPM rpm_snap_back;
    GPS gps_snap;
    Motor motor_snap;
    Filtered filtered_snap;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct SharedBlock {  // 100 bytes
    std::atomic<uint32_t> seq;
    SensorSnapshot data;
};
#pragma pack(pop)

static_assert(sizeof(std::atomic<uint32_t>) == 4, "atomic<uint32_t> must be 4 bytes");
static_assert(offsetof(SharedBlock, seq) == 0, "seq must be at offset 0");
static_assert(offsetof(SharedBlock, data) == 4, "data must start immediately after seq");
static_assert(sizeof(Power) == 12);
static_assert(sizeof(Steering) == 12);
static_assert(sizeof(RPM) == 12);
static_assert(sizeof(GPS) == 20);
static_assert(sizeof(Motor) == 12);
static_assert(sizeof(Filtered) == 4);
static_assert(sizeof(SensorSnapshot) == 96);
static_assert(sizeof(SharedBlock) == 100);

static constexpr const char* SHM_NAME = "/sensor_shm";

static volatile sig_atomic_t g_stop = 0;
static void handle_sigint(int) { g_stop = 1; }

class MasterShm {
   public:
    explicit MasterShm(const char* interface) {
        cache_.power_snap     = {0, NANF, NANF};
        cache_.steering_snap  = {0, NANF, NANF};
        cache_.rpm_snap_front = cache_.rpm_snap_back = {0, NANF, NANF};
        cache_.motor_snap                            = {0, NANF, NANF};
        gps_cache_                                   = {0, NANF, NANF, NANF, NANF};
        if (!bus_.open(interface)) {
            std::fprintf(stderr, "Cannot open CAN-FD interface %s\n", interface);
            return;
        }
        if (!init_shm()) return;
        const char* gps_path = std::getenv("SHM_GPS_DEVICE");
        gps_serial_ = open(gps_path ? gps_path : "/dev/serial0", O_RDONLY | O_NOCTTY | O_NONBLOCK);
        if (gps_serial_ >= 0) gps_file_ = fdopen(gps_serial_, "r");
        if (gps_file_) {
            gps_started_ = true;
            gps_thread_  = std::thread([this] { gps_loop(); });
        } else
            std::perror("GPS UART unavailable");
        ok_ = true;
    }
    ~MasterShm() {
        stop_.store(true);
        if (gps_thread_.joinable()) gps_thread_.join();
        if (gps_file_)
            fclose(gps_file_);
        else if (gps_serial_ >= 0)
            close(gps_serial_);
        shutdown_shm();
    }
    bool ok() const { return ok_; }
    void spin() {
        using namespace std::chrono;
        auto next = steady_clock::now();
        while (!g_stop) {
            // Bound receive work so a saturated bus cannot starve publication.
            for (int i = 0; i < 256; ++i) {
                auto rx = bus_.recv();
                if (!rx) {
                    if (rx.error() != chuds::RxError::Empty) ++cache_.errcount;
                    break;
                }
                receive(*rx);
                if (steady_clock::now() >= next) break;
            }
            auto now = steady_clock::now();
            if (now >= next) {
                cache_.global_ts = now_us();
                cache_.gps_snap  = read_gps_cached();
                const auto& rear = cache_.rpm_snap_back;
                speed            = filtered_speed(speed, (rear.rpm_left + rear.rpm_right) / 2);
                cache_.filtered_snap.speed = speed;
                write_snapshot(cache_);
                do {
                    next += milliseconds(5);
                } while (next <= now);
            }
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
    }

   private:
    chuds::Bus<chuds::SocketCanTransport> bus_;
    SensorSnapshot cache_{};
    bool ok_{false};
    int shm_fd_{-1};
    SharedBlock* shm_{nullptr};
    float speed           = NANF;
    float steer_scale_    = setting("SHM_STEERING_SCALE", NANF);
    float steer_offset_   = setting("SHM_STEERING_OFFSET", NANF);
    float brake_scale_    = setting("SHM_BRAKE_SCALE", NANF);
    float brake_offset_   = setting("SHM_BRAKE_OFFSET", NANF);
    float current_offset_ = setting("SHM_CURRENT_OFFSET_A", 0);
    void receive(const chuds::Message& m) {
        if (m.type != chuds::MsgType::Update) return;
        if (auto t = chuds::catalog_body<chuds::front_aux::Telemetry>(m)) {
            cache_.rpm_snap_front   = {t->ts, static_cast<float>(t->rpm_left),
                                       static_cast<float>(t->rpm_right)};
            cache_.steering_snap.ts = t->ts;  // front-node sample time only
            cache_.steering_snap.turn_angle =
                t->steering <= 4095 ? t->steering * steer_scale_ + steer_offset_ : NANF;
        } else if (auto t = chuds::catalog_body<chuds::back_aux::Telemetry>(m)) {
            cache_.rpm_snap_back = {t->ts, static_cast<float>(t->rpm_left),
                                    static_cast<float>(t->rpm_right)};
            cache_.steering_snap.brake_pressure =
                t->brake <= 4095 ? t->brake * brake_scale_ + brake_offset_ : NANF;
        } else if (auto t = chuds::catalog_body<chuds::joulemeter::Telemetry>(m)) {
            cache_.power_snap = {t->ts, t->current + current_offset_, t->voltage};
        }
    }
    std::mutex gps_mutex_;
    GPS gps_cache_{};
    std::thread gps_thread_;
    std::atomic<bool> stop_{false};
    std::string gps_line_;
    FILE* gps_file_{nullptr};
    bool gps_started_{false};
    int gps_serial_{-1};
    void publish_gps(const GPS& g) {
        std::lock_guard lock(gps_mutex_);
        gps_cache_ = g;
    }
    GPS read_gps_cached() {
        std::lock_guard lock(gps_mutex_);
        return gps_cache_;
    }
    bool init_shm() {
        shm_fd_ = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
        if (shm_fd_ < 0) {
            std::perror("shm_open");
            return false;
        }

        const size_t size = sizeof(SharedBlock);
        if (ftruncate(shm_fd_, size) != 0) {
            std::perror("ftruncate");
            close(shm_fd_);
            shm_fd_ = -1;
            return false;
        }

        void* p = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
        if (p == MAP_FAILED) {
            std::perror("mmap");
            close(shm_fd_);
            shm_fd_ = -1;
            return false;
        }

        shm_ = static_cast<SharedBlock*>(p);
        shm_->seq.store(0, std::memory_order_relaxed);
        std::memset(&shm_->data, 0, sizeof(shm_->data));
        std::fprintf(stderr, "Shared memory ready: %s (%zu bytes)\n", SHM_NAME, size);
        return true;
    }

    void shutdown_shm() {
        if (shm_) {
            munmap(shm_, sizeof(SharedBlock));
            shm_ = nullptr;
        }
        if (shm_fd_ >= 0) {
            close(shm_fd_);
            shm_fd_ = -1;
        }
        // If you want SHM to persist across writer restarts, comment this out.
        // shm_unlink(SHM_NAME);
    }

    // Writes snap to shared memory with a simple sequence lock for synchronization.
    inline void write_snapshot(const SensorSnapshot& snap) {
        if (!shm_) return;
        uint32_t s = shm_->seq.load(std::memory_order_relaxed);
        shm_->seq.store(s + 1, std::memory_order_release);  // odd => write in progress
        std::atomic_thread_fence(std::memory_order_seq_cst);
        shm_->data = snap;                                  // plain memcpy-able struct
        shm_->seq.store(s + 2, std::memory_order_release);  // even => stable
    }

    bool poll_gps_once(GPS& out) {
        if (!gps_started_ || !gps_file_) return false;

        char line[512];
        // Accumulate partial UART reads across EAGAIN without blocking shutdown.
        char byte;
        while (!stop_.load(std::memory_order_relaxed)) {
            if (::read(gps_serial_, &byte, 1) != 1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                return false;
            }
            if (byte == '\n') break;
            gps_line_ += byte;
            if (gps_line_.size() >= sizeof(line)) gps_line_.clear();
        }
        if (stop_.load(std::memory_order_relaxed)) return false;
        std::snprintf(line, sizeof(line), "%s", gps_line_.c_str());
        gps_line_.clear();

        if (std::strncmp(line, "$GNRMC,", 7) != 0 && std::strncmp(line, "$GPRMC,", 7) != 0) {
            return false;
        }

        double lat_ddmm, lon_ddmm;
        char status, ns, ew;

        if (std::sscanf(line, "%*[^,],%*[^,],%c,%lf,%c,%lf,%c", &status, &lat_ddmm, &ns, &lon_ddmm,
                        &ew) != 5) {
            return false;
        }

        if (status != 'A') return false;

        int lat_deg    = int(lat_ddmm / 100.0);
        double lat_min = lat_ddmm - lat_deg * 100.0;
        double lat     = lat_deg + lat_min / 60.0;

        int lon_deg    = int(lon_ddmm / 100.0);
        double lon_min = lon_ddmm - lon_deg * 100.0;
        double lon     = lon_deg + lon_min / 60.0;

        if (ns == 'S') lat = -lat;
        if (ew == 'W') lon = -lon;

        float speed   = NANF;
        float heading = NANF;

        int commas = 0;
        char* p    = line;
        while (*p && commas < 7) {
            if (*p == ',') commas++;
            p++;
        }
        if (commas < 7) return false;

        if (*p && *p != ',' && *p != '*') {
            double knots;
            if (std::sscanf(p, "%lf", &knots) == 1) {
                speed = static_cast<float>(0.514444 * knots);
            }
        }

        while (*p && *p != ',' && *p != '*') p++;
        if (*p == ',') p++;

        if (*p && *p != ',' && *p != '*') {
            double course;
            if (std::sscanf(p, "%lf", &course) == 1) {
                heading = static_cast<float>(course);
            }
        }

        out.ts       = static_cast<uint32_t>(now_us());
        out.gps_lat  = static_cast<float>(lat);
        out.gps_long = static_cast<float>(lon);
        out.heading  = heading;
        out.speed    = speed;

        return true;
    }

    void gps_loop() {
        while (!stop_.load(std::memory_order_relaxed)) {
            GPS g{};
            if (poll_gps_once(g)) {
                publish_gps(g);
            }
        }
    }

    float filtered_speed(float old_speed, float rpm) {
        float meas_speed = NANF;
        if (std::isfinite(rpm)) {
            meas_speed = rpm * 3.1415926535f * wheel_diameter_m / 60.0f;
        }

        if (!std::isfinite(meas_speed)) {
            return old_speed;  // keep previous estimate if measurement is bad
        }

        if (!std::isfinite(old_speed)) {
            return meas_speed;  // initialize filter from first valid measurement
        }

        return lambda * meas_speed + (1.0f - lambda) * old_speed;
    }
};
int main(int argc, char** argv) {
    if (argc > 2 || (argc == 2 && std::strcmp(argv[1], "--help") == 0)) {
        std::fprintf(stderr, "Usage: %s [CAN_INTERFACE] (default can0)\n", argv[0]);
        return argc > 2 ? 1 : 0;
    }
    std::signal(SIGINT, handle_sigint);
    std::signal(SIGTERM, handle_sigint);
    MasterShm node(argc == 2 ? argv[1] : "can0");
    if (!node.ok()) return 1;
    node.spin();
}
