#pragma once

#include <atomic>
#include <mutex>
#include <thread>

#include "eventbus/EventBus.hpp"

namespace sober::health {

class HealthMonitor {
 public:
  HealthMonitor(sober::eventbus::EventBus& bus);
  ~HealthMonitor();

  void start();
  void stop();

 private:
  void run();

  sober::eventbus::EventBus& bus_;
  std::thread monitorThread_;
  std::atomic<bool> running_;

  std::mutex cvMutex_;
  std::condition_variable cv_;
};

}  // namespace sober::health