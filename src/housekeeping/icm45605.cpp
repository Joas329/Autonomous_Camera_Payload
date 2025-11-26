#include "housekeeping/icm45605.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <random>
#include <stop_token>
#include <thread>

#include <spdlog/spdlog.h>

#include "eventbus/EventBus.hpp"
#include "housekeeping/SensorSnapshot.hpp"
#include "imu/inv_imu_defs.h"
#include "imu/inv_imu_driver.h"
#include "imu/inv_imu_transport.h"
#include "logger/Logger.hpp"
#include "system_interface.h"

namespace sober::housekeeping {

IMUICM45605::IMUICM45605(sober::eventbus::EventBus& bus,
                         std::shared_ptr<sober::logger::Logger::Queue> queue)
    : bus_(bus), queue_(std::move(queue)) {}

IMUICM45605::~IMUICM45605() {
  stop();
}

void IMUICM45605::start() {
#ifndef DEBUG_IMU
  if (!init()) {
    sensorThread_.request_stop();
    return;
  }
#endif
  sensorThread_ =
      std::jthread([this](std::stop_token stoken) { this->run(stoken); });
}

void IMUICM45605::stop() {
  sensorThread_.request_stop();
  cv_.notify_all();
}

bool IMUICM45605::init() {
  si_board_init();
  if (si_io_imu_init(UI_I2C) < 0) {
    return false;
  }

  inv_imu_device_t imuDev{};

  imuDev.transport.read_reg = si_io_imu_read_reg;
  imuDev.transport.write_reg = si_io_imu_write_reg;
  imuDev.transport.sleep_us = si_sleep_us;
  imuDev.transport.serif_type = UI_I2C;

  dev_ = imuDev;

  inv_imu_soft_reset(&dev_);
  inv_imu_set_accel_mode(&dev_, PWR_MGMT0_ACCEL_MODE_LN);
  inv_imu_set_gyro_mode(&dev_, PWR_MGMT0_GYRO_MODE_LN);
  inv_imu_set_accel_frequency(&dev_, ACCEL_CONFIG0_ACCEL_ODR_100_HZ);
  inv_imu_set_gyro_frequency(&dev_, GYRO_CONFIG0_GYRO_ODR_100_HZ);
  inv_imu_set_accel_fsr(&dev_, ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G);
  inv_imu_set_gyro_fsr(&dev_, GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS);

  return true;
}

bool IMUICM45605::readOnce(std::array<float, 7>& sensorData) {
#ifndef DEBUG_IMU
  inv_imu_sensor_data_t data{};
  if (inv_imu_get_register_data(&dev_, &data) != 0) {
    return false;
  }

  double axG = data.accel_data[0] / ACCEL_SENS;
  double ayG = data.accel_data[1] / ACCEL_SENS;
  double azG = data.accel_data[2] / ACCEL_SENS;
  // double axMs2 = axG * G;
  // double ayMs2 = ayG * G;
  // double azMs2 = azG * G;

  double gxDps = data.gyro_data[0] / GYRO_SENS;
  double gyDps = data.gyro_data[1] / GYRO_SENS;
  double gzDps = data.gyro_data[2] / GYRO_SENS;
  // double gxRads = gxDps * (M_PI / 180.0);
  // double gyRads = gyDps * (M_PI / 180.0);
  // double gzRads = gzDps * (M_PI / 180.0);

  double tempC = (data.temp_data / TEMP_SENS) + TEMP_OFF;

  sensorData = {static_cast<float>(axG),   static_cast<float>(ayG),
                static_cast<float>(azG),   static_cast<float>(gxDps),
                static_cast<float>(gyDps), static_cast<float>(gzDps),
                static_cast<float>(tempC)};

  uint64_t generator = gSnapshot.generation.load(std::memory_order_relaxed);
  gSnapshot.generation.store(generator + 1, std::memory_order_release);

  gSnapshot.accX = sensorData[0];
  gSnapshot.accY = sensorData[1];
  gSnapshot.accZ = sensorData[2];
  gSnapshot.gyrX = sensorData[3];
  gSnapshot.gyrY = sensorData[4];
  gSnapshot.gyrZ = sensorData[5];
  gSnapshot.imuTemp = sensorData[6];

  gSnapshot.generation.store(generator + 2, std::memory_order_release);

  // SPDLOG_INFO(
  //     "[IMU] Accelerometer: [{}, {}, {}] m/s^2; Gyroscope: [{}, {}, {}] deg/s; "
  //     "Temperature: [{}] Celsius",
  //     axMs2, ayMs2, azMs2, gxDps, gyDps, gzDps, tempC);
#else
  static std::random_device randDist;
  static std::mt19937 rng(randDist());
  static std::normal_distribution<float> dist(0.0f, 0.5f);

  for (auto& value : sensorData) {
    value = dist(rng);
  }
  // SPDLOG_INFO(
  //     "[IMU] Accelerometer: [{}, {}, {}] m/s^2; Gyroscope: [{}, {}, {}] deg/s; "
  //     "Temperature: [{}] Celsius",
  //     sensorData[0], sensorData[1], sensorData[2], sensorData[3], sensorData[4],
  //     sensorData[5], sensorData[6]);

  uint64_t generator = gSnapshot.generation.load(std::memory_order_relaxed);
  gSnapshot.generation.store(generator + 1, std::memory_order_release);

  gSnapshot.accX = sensorData[0];
  gSnapshot.accY = sensorData[1];
  gSnapshot.accZ = sensorData[2];
  gSnapshot.gyrX = sensorData[3];
  gSnapshot.gyrY = sensorData[4];
  gSnapshot.gyrZ = sensorData[5];
  gSnapshot.imuTemp = sensorData[6];

  gSnapshot.generation.store(generator + 2, std::memory_order_release);
#endif
  return true;
}

void IMUICM45605::logData(const sober::logger::LogEntry& rec) {
  if (queue_) {
    const std::size_t HIGH_WATER = queue_->capacity() * 8 / 10;
    std::size_t qsz = queue_->size();
    std::size_t qcap = queue_->capacity();

    bool pushed = queue_->try_emplace(rec);

    if (!pushed) {
      SPDLOG_WARN("[IMU] queue overflow, dropping sample (size={}/{})", qsz,
                  qcap);
      SPDLOG_DEBUG("[IMU] notifying logger due to overflow");
      sober::logger::Logger::instance().notify();
    } else if (qsz >= HIGH_WATER) {
      SPDLOG_DEBUG("[IMU] high-water mark reached: {}/{} ({}%)", qsz, qcap,
                   (qsz * 100) / qcap);
      sober::logger::Logger::instance().notify();
    }
  }
}

void IMUICM45605::run(std::stop_token stoken) {
  auto next_wakeup = std::chrono::steady_clock::now() + sample_period;

  while (!stoken.stop_requested()) {
    std::array<float, 7> sensorData{};
    if (readOnce(sensorData)) {
      sober::logger::LogEntry rec{};
      struct timespec unixTimestamp {};
      if (clock_gettime(CLOCK_REALTIME, &unixTimestamp) != 0) {
        unixTimestamp.tv_sec = 0;
        unixTimestamp.tv_nsec = 0;
      }

      rec.timestamp =
          static_cast<uint64_t>(unixTimestamp.tv_sec) * 1000000000ULL +
          static_cast<uint64_t>(unixTimestamp.tv_nsec);

      constexpr size_t packageSize = sensorData.size() * sizeof(float);
      rec.length = static_cast<uint8_t>(packageSize);

      std::memcpy(rec.data.data(), sensorData.data(), packageSize);

      logData(rec);

      std::unique_lock<std::mutex> lk(cvMutex_);
      cv_.wait_until(lk, next_wakeup, [&] { return stoken.stop_requested(); });
      next_wakeup += sample_period;
    } else {
      SPDLOG_WARN("[IMU] Failed to read valid data from the IMU");
      std::unique_lock<std::mutex> lk(cvMutex_);
      cv_.wait_until(lk, next_wakeup, [&] { return stoken.stop_requested(); });
      next_wakeup += sample_period;
    }
  }
}

}  // namespace sober::housekeeping