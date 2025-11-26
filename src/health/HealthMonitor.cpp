#include "health/HealthMonitor.hpp"

#include <chrono>
#include <mutex>
#include <thread>

#include "eventbus/Event.hpp"
#include "eventbus/EventBus.hpp"

namespace sober::health {

HealthMonitor::HealthMonitor(sober::eventbus::EventBus& bus)
    : bus_(bus), running_(false) {}

HealthMonitor::~HealthMonitor() {
  stop();
}

void HealthMonitor::start() {
  running_ = true;
  monitorThread_ = std::thread(&HealthMonitor::run, this);
}

void HealthMonitor::stop() {
  running_ = false;
  cv_.notify_all();
  if (monitorThread_.joinable()) {
    monitorThread_.join();
  }
}

void HealthMonitor::run() {
  while (running_) {
    sober::eventbus::Event event;
    event.type = sober::eventbus::EventType::HealthStatus;
    event.source = "HealthMonitor";
    event.value = 1.0;
    event.timestamp = std::chrono::system_clock::now();
    bus_.publish(event);

    std::unique_lock<std::mutex> lock(cvMutex_);
    if (cv_.wait_for(lock, std::chrono::seconds(1),
                     [this] { return !running_; })) {
      break;
    }
  }
}

}  // namespace sober::health