#pragma once

#include <atomic>
#include <memory>

#include "eventbus/EventBus.hpp"
#include "logger/Logger.hpp"

#include "camera/OpticalCamera.hpp"
#include "camera/IRCamera.hpp"
#include "camera/Ticker.hpp"

class SystemController {
public:
    SystemController();
    ~SystemController();

    void start();
    void stop();

    void startExperiment();
    void stopExperiment();

private:
    std::atomic<bool> experimentRunning_{false};

    sober::eventbus::EventBus eventBus_;
    sober::logger::Logger& logger_;

    // Camera timing ticker
    sober::camera::Ticker ticker_{std::chrono::milliseconds(1000)};

    // Cameras
    std::unique_ptr<sober::camera::OpticalCamera> opticalCamera_;
    std::unique_ptr<sober::camera::IRCamera> irCamera_;

    // helpers
    void startOpticalCamera();
    void startIRCamera();
};
