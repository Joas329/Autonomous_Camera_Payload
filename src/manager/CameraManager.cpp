#ifdef SOBER_ENABLE_HW_CAMERAS

#include "manager/CameraManager.hpp"
#include <spdlog/spdlog.h>

CameraManager::CameraManager(sober::eventbus::EventBus& bus)
    : bus_(bus) {}

CameraManager::~CameraManager() {
    stop();
}

void CameraManager::start() {
    SPDLOG_INFO("[CAMERA MANAGER] Starting camera...");

    camera1_ = std::make_unique<sober::camera::OpticalCamera>(ticker_);

    if (camera1_->start()) {
        SPDLOG_INFO("[CAMERA MANAGER] Optical camera started.");
    } else {
        SPDLOG_ERROR("[CAMERA MANAGER] Failed to start optical camera!");
    }
}

void CameraManager::stop() {
    SPDLOG_INFO("[CAMERA MANAGER] Stopping camera...");

    if (camera1_) {
        camera1_->stop();
        camera1_.reset();
    }
}

#endif
