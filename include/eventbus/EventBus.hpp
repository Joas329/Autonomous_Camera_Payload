#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>

#include "Event.hpp"

namespace sober::eventbus {

class EventBus {
 public:
  void publish(const Event& event) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(event);
    cv_.notify_one();
  }

  Event consume() {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] { return !queue_.empty(); });
    Event event = queue_.front();
    queue_.pop();
    return event;
  }

 private:
  std::queue<Event> queue_;
  std::mutex mutex_;
  std::condition_variable cv_;
};

}  // namespace sober::eventbus
