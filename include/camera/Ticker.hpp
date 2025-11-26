#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <stop_token>
#include <thread>

namespace sober::camera {

struct TickBus {
  std::mutex tickM;
  std::condition_variable cv;
  uint64_t tick{0};
  bool stop = false;
};

class Ticker {
 public:
  explicit Ticker(std::chrono::nanoseconds period);
  ~Ticker();

  void waitNext(uint64_t& seen);

  uint64_t current();

 private:
  static void loop_(std::stop_token st, TickBus& bus,
                    std::chrono::nanoseconds period);
  TickBus bus_;
  std::jthread tickerThread_;
};

}  // namespace sober::camera
