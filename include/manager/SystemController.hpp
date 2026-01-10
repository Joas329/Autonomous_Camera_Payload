#pragma once

#include <atomic>
#include <memory>
#include <optional>
#include <vector>
#include <cstdint>

#include "SystemStatus.hpp"

#include "eventbus/EventBus.hpp"
#include "logger/Logger.hpp"

#include "camera/OpticalCamera.hpp"
#include "camera/OpticalSensor.hpp"
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
    void stopOpticalCamera();
    void startOpticalCamera();
    void startAcquisition();
    void stopAcquisition();
    bool setExposureTimeOptical(double exposure_us);
    bool setOutputDirOptical(const std::string& dir);
    SystemStatus collectSystemStatus() const;
    std::optional<std::vector<uint8_t>> getLastOpticalFrame() const;

private:
    std::atomic<bool> experimentRunning_{false};

    sober::eventbus::EventBus eventBus_;
    sober::logger::Logger& logger_;

    // Camera timing ticker
    sober::camera::Ticker ticker_{std::chrono::milliseconds(1000)};

    // Cameras
    std::unique_ptr<sober::camera::FLIR_Blackfly_S> opticalCamera_;
    std::unique_ptr<sober::camera::IRCamera> irCamera_;

    // helpers
    void startIRCamera();
};
