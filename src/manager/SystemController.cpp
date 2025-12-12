#include "manager/SystemController.hpp"
#include "communicator/SoberApi.hpp"
#include <crow.h>
#include <filesystem>
#include <fstream>
#include <vector>
#include <algorithm>
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

    // startOpticalCamera();
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
        opticalCamera_.reset();
        SPDLOG_INFO("[SYSTEM CONTROLLER] Optical camera stopped.");
    } else {
        SPDLOG_WARN("[SYSTEM CONTROLLER] Optical camera is not running.");
    }
}

void SystemController::startOpticalCamera() {
    opticalCamera_ = std::make_unique<sober::camera::OpticalCamera>(ticker_);

    if (opticalCamera_->start()) {
        SPDLOG_INFO("[SYSTEM CONTROLLER] Optical camera started.");
    } else {
        SPDLOG_ERROR("[SYSTEM CONTROLLER] Failed to start optical camera.");
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