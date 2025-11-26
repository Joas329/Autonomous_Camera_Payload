#include "manager/SystemController.hpp"

#include <spdlog/spdlog.h>

SystemController::SystemController()
    : eventBus_(),
      logger_(sober::logger::Logger::instance())
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
