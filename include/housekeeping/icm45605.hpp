#pragma once

#include "eventbus/EventBus.hpp"

#include <array>
#include <mutex>
#include <stop_token>
#include <thread>

#include "logger/Logger.hpp"

extern "C" {
#include "imu/inv_imu_driver.h"
}

namespace sober::housekeeping {

inline constexpr double ACCEL_SENS = 8192.0;
inline constexpr double GYRO_SENS = 16.4;
inline constexpr double G = 9.80665;
inline constexpr double TEMP_OFF = 25.0;
inline constexpr double TEMP_SENS = 128.0;

// #define DEBUG_IMU

class IMUICM45605 {
 public:
  IMUICM45605(sober::eventbus::EventBus& bus,
              std::shared_ptr<sober::logger::Logger::Queue> queue);
  ~IMUICM45605();

  void start();
  void stop();

 private:
  void run(std::stop_token stoken);

  bool init();
  bool readOnce(std::array<float, 7>& sensorData);

  void logData(const sober::logger::LogEntry& rec);

  sober::eventbus::EventBus& bus_;
  std::jthread sensorThread_;
  inv_imu_device_t dev_{};

  std::mutex cvMutex_;
  std::condition_variable cv_;

  std::shared_ptr<sober::logger::Logger::Queue> queue_;

  static constexpr auto sample_period = std::chrono::milliseconds(10);
};

}  // namespace sober::housekeeping