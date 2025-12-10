#pragma once

#include <thread>
#include <condition_variable>
#include <mutex>
#include <atomic>
#include <string>

#include "camera/TelopsCamera.hpp"
#include "camera/Ticker.hpp"
#include "eventbus/EventBus.hpp"

namespace sober::camera {

class IRCamera {
public:
    IRCamera();                  // default
    IRCamera(Ticker& sharedTicker);  // <-- NEW constructor

    bool start();
    void stop();

private:
    void run();

private:
    // Required because TelopsTriggerSystem wants an EventBus ref
    sober::eventbus::EventBus dummyBus_;

    TelopsTriggerSystem telops_;

    std::thread irThread_;
    std::atomic<bool> running_;

    Ticker* ticker_;             // <--- pointer to shared ticker (non-owning)

    std::condition_variable cv_;
    std::mutex mtx_;
};

} // namespace sober::camera
