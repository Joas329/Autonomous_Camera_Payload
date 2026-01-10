#include "manager/SystemController.hpp"
#include "communicator/SoberApi.hpp"
#include <crow.h>
#include <filesystem>
#include <fstream>
#include <vector>
#include <algorithm>
#include <sys/statvfs.h> 
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

SystemController::SystemController()
    : logger_(sober::logger::Logger::instance())
{
}

SystemController::~SystemController() {
    stop();
}

void SystemController::start() {
    SPDLOG_INFO("[SYSTEM CONTROLLER] Starting system...");
    std::cout << "Initial Entry of the program here." << std::endl;
    logger_.start();
    startExperiment();

    // Start the API
    static crow::SimpleApp app;
    static sober::communicator::SoberApi api(*this);

    api.registerRoutes(app);

    std::thread apiThread([] {
        app.port(8080)
           .multithreaded()
           .run();
    });

    apiThread.detach();
}

void SystemController::stop() {
    SPDLOG_INFO("[SYSTEM CONTROLLER] Stopping system...");

    stopExperiment();

    logger_.stop();
}

void SystemController::startExperiment() {
    bool expected = false;

    if (!experimentRunning_.compare_exchange_strong(expected, true)) {
        SPDLOG_WARN("[SYSTEM CONTROLLER] Experiment already running.");
        return;
    }

    SPDLOG_INFO("[SYSTEM CONTROLLER] Starting experiment...");

    startOpticalCamera();
    //startIRCamera();
}

void SystemController::stopExperiment() {
    bool expected = true;

    if (!experimentRunning_.compare_exchange_strong(expected, false)) {
        SPDLOG_WARN("[SYSTEM CONTROLLER] Experiment not running.");
        return;
    }

    SPDLOG_INFO("[SYSTEM CONTROLLER] Stopping experiment...");

    if (opticalCamera_) {
        opticalCamera_->stop();
        opticalCamera_.reset();
    }

    if (irCamera_) {
        irCamera_->stop();
        irCamera_.reset();
    }
}

void SystemController::stopOpticalCamera() {
    if (opticalCamera_) {
        opticalCamera_->stop();
        SPDLOG_INFO("[SYSTEM CONTROLLER] Optical camera stopped.");
    } else {
        SPDLOG_WARN("[SYSTEM CONTROLLER] Optical camera is not running.");
    }
}

void SystemController::startOpticalCamera() {
    opticalCamera_ = std::make_unique<sober::camera::FLIR_Blackfly_S>(ticker_);

    if (opticalCamera_->start()) {
        SPDLOG_INFO("[SYSTEM CONTROLLER] Optical camera started.");
    } else {
        SPDLOG_ERROR("[SYSTEM CONTROLLER] Failed to start optical camera.");
    }
}

void SystemController::startAcquisition() {
    if (opticalCamera_) {
        if (opticalCamera_->startAcquisition()) {
            SPDLOG_INFO("[SYSTEM CONTROLLER] Optical camera acquisition started.");
        } else {
            SPDLOG_ERROR("[SYSTEM CONTROLLER] Failed to start optical camera acquisition.");
        }
    } else {
        SPDLOG_ERROR("[SYSTEM CONTROLLER] Optical camera is not initialized.");
    }
}

void SystemController::stopAcquisition() {
    if (opticalCamera_) {
        if (opticalCamera_->stopAcquisition()) {
            SPDLOG_INFO("[SYSTEM CONTROLLER] Optical camera acquisition stopped.");
        } else {
            SPDLOG_ERROR("[SYSTEM CONTROLLER] Failed to stop optical camera acquisition.");
        }
    } else {
        SPDLOG_ERROR("[SYSTEM CONTROLLER] Optical camera is not initialized.");
    }
}

void SystemController::startIRCamera() {
    irCamera_ = std::make_unique<sober::camera::IRCamera>(ticker_);

    if (irCamera_->start()) {
        SPDLOG_INFO("[SYSTEM CONTROLLER] IR camera started.");
    } else {
        SPDLOG_ERROR("[SYSTEM CONTROLLER] Failed to start IR camera.");
    }
}

bool SystemController::setExposureTimeOptical(double exposure_us)
{
    if (!opticalCamera_) {
        SPDLOG_WARN("[SYSTEM CONTROLLER] Optical camera not available");
        return false;
    }
    return opticalCamera_->set_exposure_time(exposure_us);
}

bool SystemController::setOutputDirOptical(const std::string& dir)
{
    if (!opticalCamera_) {
        SPDLOG_WARN("[SYSTEM CONTROLLER] Optical camera not available");
        return false;
    }
    return opticalCamera_->set_output_dir(dir);
}

std::optional<std::vector<uint8_t>> SystemController::getLastOpticalFrame() const
{
    if (!opticalCamera_) {
        return std::nullopt;
    }

    const fs::path imgDir = "/home/sober/Autonomous_Control/images";

    if (!fs::exists(imgDir) || !fs::is_directory(imgDir)) {
        return std::nullopt;
    }

    fs::path newest;
    std::filesystem::file_time_type newestTime;

    for (const auto& e : fs::directory_iterator(imgDir)) {
        if (!e.is_regular_file())
            continue;

        if (e.path().extension() != ".jpg")
            continue;

        const auto t = fs::last_write_time(e);
        if (newest.empty() || t > newestTime) {
            newest = e.path();
            newestTime = t;
        }
    }

    if (newest.empty()) {
        return std::nullopt;
    }

    std::ifstream ifs(newest, std::ios::binary | std::ios::ate);
    if (!ifs) {
        return std::nullopt;
    }

    const std::streamsize size = ifs.tellg();
    ifs.seekg(0, std::ios::beg);

    std::vector<uint8_t> data(size);
    if (!ifs.read(reinterpret_cast<char*>(data.data()), size)) {
        return std::nullopt;
    }

    return data;
}

// -------------------- CPU TEMP --------------------
static double read_cpu_temp_c()
{
    std::ifstream ifs("/sys/class/thermal/thermal_zone0/temp");
    double milli;
    ifs >> milli;
    return milli / 1000.0;
}

// -------------------- CPU LOAD --------------------
static double read_cpu_load_pct()
{
    static uint64_t last_idle = 0, last_total = 0;

    std::ifstream ifs("/proc/stat");
    std::string cpu;
    uint64_t user, nice, system, idle, iowait, irq, softirq, steal;

    ifs >> cpu >> user >> nice >> system >> idle
        >> iowait >> irq >> softirq >> steal;

    const uint64_t idle_now = idle + iowait;
    const uint64_t total_now =
        user + nice + system + idle + iowait + irq + softirq + steal;

    const uint64_t delta_idle = idle_now - last_idle;
    const uint64_t delta_total = total_now - last_total;

    last_idle = idle_now;
    last_total = total_now;

    if (delta_total == 0) return 0.0;

    return 100.0 * (1.0 - (double)delta_idle / delta_total);
}

// -------------------- DISK USAGE --------------------
static void read_disk_usage_gb(double& used, double& total)
{
    struct statvfs fs {};
    statvfs("/", &fs);

    const double block = fs.f_frsize;
    total = fs.f_blocks * block / 1e9;
    const double free = fs.f_bfree * block / 1e9;
    used = total - free;
}

// -------------------- UPTIME --------------------
static double read_uptime_s()
{
    std::ifstream ifs("/proc/uptime");
    double uptime;
    ifs >> uptime;
    return uptime;
}

// -------------------- UTC TIME --------------------
static std::string utc_now_iso()
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    std::tm tm {};
    gmtime_r(&ts.tv_sec, &tm);

    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
    return std::string(buf);
}

// ==================== PUBLIC API ====================
SystemStatus SystemController::collectSystemStatus() const
{
    SystemStatus s{};

    s.cpu_temp_c   = read_cpu_temp_c();
    s.cpu_load_pct = read_cpu_load_pct();
    read_disk_usage_gb(s.disk_used_gb, s.disk_total_gb);
    s.uptime_s     = read_uptime_s();
    s.utc_iso      = utc_now_iso();

    return s;
}