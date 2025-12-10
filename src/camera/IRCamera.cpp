#include "camera/IRCamera.hpp"
#include <spdlog/spdlog.h>
#include <filesystem>
#include <iomanip>

namespace fs = std::filesystem;

namespace sober::camera {

// --------------------------------------------
// constructor
// --------------------------------------------
sober::camera::IRCamera::IRCamera()
    : dummyBus_(),
      telops_(dummyBus_, "IR"),
      running_(false),
      ticker_(nullptr)
{}

sober::camera::IRCamera::IRCamera(Ticker& sharedTicker)
    : dummyBus_(),
      telops_(dummyBus_, "IR"),
      running_(false),
      ticker_(&sharedTicker)
{}

// --------------------------------------------
// timestamp helper
// --------------------------------------------
static std::string make_filename()
{
    using clock = std::chrono::system_clock;

    const auto now = clock::now();
    const auto t   = clock::to_time_t(now);
    const auto us  = std::chrono::duration_cast<std::chrono::microseconds>(
                         now.time_since_epoch()) % 1000000;

    std::tm tm{};
#if defined(_POSIX_VERSION)
    localtime_r(&t, &tm);
#else
    tm = *std::localtime(&t);
#endif

    char buf[64];
    std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d_%06ld",
                  tm.tm_year + 1900,
                  tm.tm_mon + 1,
                  tm.tm_mday,
                  tm.tm_hour,
                  tm.tm_min,
                  tm.tm_sec,       // <-- FIXED
                  us.count());

    return std::string(buf);
}

// --------------------------------------------
// start
// --------------------------------------------
bool IRCamera::start()
{
    SPDLOG_INFO("[IR CAMERA] start()");

    telops_.setSaveDir("/home/sober/Autonomous_Control/ir");
    telops_.start();

    running_ = true;
    irThread_ = std::thread(&IRCamera::run, this);
    return true;
}

// --------------------------------------------
// stop
// --------------------------------------------
void IRCamera::stop()
{
    running_ = false;
    cv_.notify_all();

    if (irThread_.joinable())
        irThread_.join();

    telops_.stop();
}

// --------------------------------------------
// run
// --------------------------------------------
void IRCamera::run()
{
    SPDLOG_INFO("[IR CAMERA] run() started");

    constexpr double kFPS = 1.0;
    uint64_t seen = 0;

    while (running_)
    {
        ticker_->waitNext(seen);

        auto t0 = std::chrono::steady_clock::now();

        std::string name = make_filename();

        bool ok = telops_.triggerFrame(name);

        if (!ok)
        {
            SPDLOG_WARN("[IR CAMERA] triggerFrame failed ({})", name);
            continue;
        }

        auto t1 = std::chrono::steady_clock::now();
        SPDLOG_INFO("[IR CAMERA] Saved IR frame {} ({} ms)",
                    name,
                    std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count());
    }

    SPDLOG_INFO("[IR CAMERA] run() exiting");
}

} // namespace sober::camera
