#pragma once

#include <memory>

#include "eventbus/EventBus.hpp"
#include "camera/OpticalCamera.hpp"
#include "camera/Ticker.hpp"

class CameraManager {
public:
    explicit CameraManager(sober::eventbus::EventBus& bus);
    ~CameraManager();

    void start();
    void stop();

private:
    sober::eventbus::EventBus& bus_;
    sober::camera::Ticker ticker_{std::chrono::milliseconds(1000)};

    std::unique_ptr<sober::camera::OpticalCamera> camera1_;
};
