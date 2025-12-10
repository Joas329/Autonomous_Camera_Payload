#include "housekeeping/LIS3MDLTR.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <random>
#include <stop_token>
#include <thread>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <spdlog/spdlog.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "eventbus/EventBus.hpp"
#include "housekeeping/SensorSnapshot.hpp"
#include "logger/Logger.hpp"

namespace sober::housekeeping {

LIS3MDLTR::LIS3MDLTR(sober::eventbus::EventBus& bus, uint8_t i2c_bus,
                     uint8_t i2c_adr,
                     std::shared_ptr<sober::logger::Logger::Queue> queue)
    : m_bus(bus),
      m_i2cFd(-1),
      m_i2cBus(i2c_bus),
      m_i2cAddr(i2c_adr),
      m_tFine(0),
      m_settings({}),
      m_readings({}),
      queue_(std::move(queue)) {
  m_settings.m_blockUpdate = BlocUpdateMagneticData::MSBLSB;
  // m_settings.m_commMode=CommunicationMode::Wire_4;
  m_settings.m_endian = BigLittleEndian::LSB;
  m_settings.m_fastRead = FastRead::Disabled;
  m_settings.m_fullScale = FullScale::Gauss_16;
  m_settings.m_modeXY = ModeXY::HighPerf;
  m_settings.m_modeZ = ModeZ::HighPerf;
  m_settings.m_opMode = OperationMode::Continuous;
  m_settings.m_outputDataRate = OutputDataRate::Hz_20;
  m_settings.m_powerMode = PowerMode::Normal;
  // m_settings.m_reboot
  m_settings.m_selfTest = SelfTest::ON;
  // m_settings.m_softReset
  // m_settings.m_tempEnabled
}

LIS3MDLTR::~LIS3MDLTR() {
  stop();
  if (m_i2cFd != -1) {
    close(m_i2cFd);
  }
}

bool LIS3MDLTR::start() {
#ifndef DEBUG_MAG
  if (!initialize_sensor()) {
    SPDLOG_CRITICAL("[MAGNETOMETER] Failed to initialize LIS3MDLTR");
    return false;
  }
  // Set default settings
  m_settings.m_blockUpdate = BlocUpdateMagneticData::MSBLSB;
  // m_settings.m_commMode=CommunicationMode::Wire_4;
  m_settings.m_endian = BigLittleEndian::LSB;
  m_settings.m_fastRead = FastRead::Disabled;
  m_settings.m_fullScale = FullScale::Gauss_16;
  m_settings.m_modeXY = ModeXY::HighPerf;
  m_settings.m_modeZ = ModeZ::HighPerf;
  m_settings.m_opMode = OperationMode::Continuous;
  m_settings.m_outputDataRate = OutputDataRate::Hz_20;
  m_settings.m_powerMode = PowerMode::Normal;
  // m_settings.m_reboot
  m_settings.m_selfTest = SelfTest::ON;
  // m_settings.m_softReset
  // m_settings.m_tempEnabled
  if (!apply_settings()) {
    SPDLOG_ERROR("[MAGNETOMETER] Failed to apply LIS3MDLTR settings");
    return false;
  }
  sensorThread_ =
      std::jthread([this](std::stop_token stoken) { this->run(stoken); });
  return true;
#else
  sensorThread_ = std::jthread(&LIS3MDLTR::run, this);
  return true;
#endif
}

bool LIS3MDLTR::apply_settings() {
  //Verify CTRL1  register
  uint8_t ctrl1 = 0;
  ctrl1 |= static_cast<uint8_t>(m_settings.m_outputDataRate) << BitShift2;
  ctrl1 |= static_cast<uint8_t>(m_settings.m_modeXY) << BitShift5;
  ctrl1 |= static_cast<uint8_t>(m_settings.m_tempEnabled) << BitShift7;
  // ctrl1 |= static_cast<uint8_t>(m_settings.m_fastRead) << 1;
  ctrl1 |= static_cast<uint8_t>(m_settings.m_selfTest);
  if (!write_register(MAG_CTRL_REG1, ctrl1)) {
    return false;
  }

  //Verify Ctrl2 register
  uint8_t ctrl2 = 0;
  ctrl2 |= static_cast<uint8_t>(m_settings.m_fullScale) << BitShift5;
  // uint8_t ctrl2 = static_cast<uint8_t>(m_settings.m_reboot) << 3;
  // uint8_t ctrl2 = static_cast<uint8_t>(m_settings.m_softReset) << 2;
  if (!write_register(MAG_CTRL_REG2, ctrl2)) {
    return false;
  }
  //Verify Ctrl3 register
  uint8_t ctrl3 = 0;
  ctrl3 |= static_cast<uint8_t>(m_settings.m_powerMode) << BitShift5;
  ctrl3 |= static_cast<uint8_t>(m_settings.m_opMode);
  // uint8_t ctrl3 = static_cast<uint8_t>(m_settings.m_commMode)<<2;

  if (!write_register(MAG_CTRL_REG3, ctrl3)) {
    return false;
  }
  //Verify Ctrl4 register
  uint8_t ctrl4 = 0;
  ctrl4 |= static_cast<uint8_t>(m_settings.m_modeZ) << BitShift2;
  ctrl4 |= static_cast<uint8_t>(m_settings.m_endian) << BitShift1;
  if (!write_register(MAG_CTRL_REG4, ctrl4)) {
    return false;
  }
  //Verify Ctrl5 register
  uint8_t ctrl5 = 0;
  ctrl5 |= static_cast<uint8_t>(m_settings.m_fastRead) << BitShift7;
  ctrl5 |= static_cast<uint8_t>(m_settings.m_blockUpdate) << BitShift6;
  return write_register(MAG_CTRL_REG5, ctrl5);
}

bool LIS3MDLTR::write_register(uint8_t reg, uint8_t value) {
  std::array<uint8_t, 2> buffer = {reg, value};
  if (write(m_i2cFd, buffer.data(), buffer.size()) !=
      static_cast<ssize_t>(buffer.size())) {
    SPDLOG_ERROR("[MAGNETOMETER] Failed to write to LIS3MDL register");
    return false;
  }
  return true;
}

bool LIS3MDLTR::read_registers(uint8_t reg, uint8_t* buffer, size_t length) {
  if (write(m_i2cFd, &reg, 1) != 1) {
    SPDLOG_ERROR("[MAGNETOMETER] Failed to write register address");
    return false;
  }

  if (read(m_i2cFd, buffer, length) != static_cast<ssize_t>(length)) {
    SPDLOG_ERROR("[MAGNETOMETER] Failed to read from register");
    return false;
  }
  return true;
}

std::array<float, 3> LIS3MDLTR::read_magnetic_data() {
  const std::lock_guard<std::mutex> lock(m_readingsMutex);
  return m_readings.m_mag;
}

float LIS3MDLTR::read_temp() {
  const std::lock_guard<std::mutex> lock(m_readingsMutex);
  return m_readings.m_temp;
}

void LIS3MDLTR::stop() {
  sensorThread_.request_stop();
  cv_.notify_all();
}

bool LIS3MDLTR::initialize_sensor() {
  if (m_i2cFd >= 0) {
    return true;
  }

  char filename[20];
  snprintf(filename, sizeof(filename), "/dev/i2c-%d", m_i2cBus);

  if ((m_i2cFd = open(filename, O_RDWR)) < 0) {
    SPDLOG_ERROR("[MAGNETOMETER] Failed to open Magnetometer I2C bus");
    return false;
  }

  if (ioctl(m_i2cFd, I2C_SLAVE, m_i2cAddr) < 0) {
    SPDLOG_ERROR("[MAGNETOMETER] Failed to set I2C slave address");
    close(m_i2cFd);
    m_i2cFd = -1;
    return false;
  }

  // Check chip ID
  uint8_t chipId = 0;
  if (!read_registers(MAG_ID_REG, &chipId, 1) || chipId != MAG_CHIPID) {
    SPDLOG_ERROR("[MAGNETOMETER] Invalid chip ID");
    close(m_i2cFd);
    m_i2cFd = -1;
    return false;
  }

  // Reset the device
  m_settings.m_softReset = SoftRst::Enable;
  auto ctrl2 =
      static_cast<uint8_t>(static_cast<uint8_t>(m_settings.m_softReset) << 2);
  if (!write_register(MAG_CTRL_REG2, ctrl2)) {
    SPDLOG_ERROR("[MAGNETOMETER] Failed to reset device");
    close(m_i2cFd);
    m_i2cFd = -1;
    return false;
  }

  // Wait for reset to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Read calibration data
  if (!read_data()) {
    SPDLOG_ERROR("[MAGNETOMETER] Failed to read raw data");
    close(m_i2cFd);
    m_i2cFd = -1;
    return false;
  }

  return true;
}
bool LIS3MDLTR::read_data() {
  std::array<uint8_t, 8> buffer{};
  return read_registers(MAG_OUTX_LSB, buffer.data(), 8);
}

uint8_t LIS3MDLTR::read_register(uint8_t reg) {
  uint8_t value = 0;
  if (!read_registers(reg, &value, 1)) {
    SPDLOG_ERROR("[MAGNETOMETER] Failed to read register");
    return 0;
  }
  return value;
}

void LIS3MDLTR::logData(const sober::logger::LogEntry& rec) {
  if (queue_) {
    const std::size_t HIGH_WATER = queue_->capacity() * 8 / 10;
    std::size_t qsz = queue_->size();
    std::size_t qcap = queue_->capacity();

    bool pushed = queue_->try_emplace(rec);

    if (!pushed) {
      SPDLOG_WARN("[MAGNETOMETER] queue overflow, dropping sample (size={}/{})",
                  qsz, qcap);
      SPDLOG_DEBUG("[MAGNETOMETER] notifying logger due to overflow");
      sober::logger::Logger::instance().notify();
    } else if (qsz >= HIGH_WATER) {
      SPDLOG_DEBUG("[MAGNETOMETER] high-water mark reached: {}/{} ({}%)", qsz,
                   qcap, (qsz * 100) / qcap);
      sober::logger::Logger::instance().notify();
    }
  }
}

void LIS3MDLTR::run(std::stop_token stoken) {
#ifndef DEBUG_MAG
  if (!initialize_sensor()) {
    SPDLOG_ERROR("[MAGNETOMETER] Failed to initialize LIS3MDLTR");
    return;
  }
#else
  std::random_device rd;
  std::mt19937 rng(rd());
  std::normal_distribution<float> magDist(0.0f, 0.5f);
#endif
  auto next_wakeup = std::chrono::steady_clock::now() + sample_period;

  while (!stoken.stop_requested()) {
#ifndef DEBUG_MAG
    std::array<uint8_t, 8> data{};

    // Read sensor data block
    if (!read_registers(MAG_OUTX_LSB, data.data(), data.size())) {
      SPDLOG_ERROR("[MAGNETOMETER] Failed to read sensor data");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    // Convert raw
    auto raw_x = static_cast<int16_t>((data[1] << 8) | data[0]);
    auto raw_y = static_cast<int16_t>((data[3] << 8) | data[2]);
    auto raw_z = static_cast<int16_t>((data[5] << 8) | data[4]);
    auto raw_temp = static_cast<int16_t>((data[7] << 8) | data[6]);

    // Convert to physical units ( mG MiliGauss)
    constexpr float Gauss_16 = 1711.0f;
    float Conversion_1 = 8.0f;
    float Conversion_2 = 25.0f;

    {
      const std::lock_guard<std::mutex> lock(m_readingsMutex);
      m_readings.m_mag = {static_cast<float>(raw_x) / Gauss_16,
                          static_cast<float>(raw_y) / Gauss_16,
                          static_cast<float>(raw_z) / Gauss_16};
      m_readings.m_temp =
          static_cast<float>(raw_temp) / Conversion_1 + Conversion_2;
    }

    sober::logger::LogEntry rec{};
    struct timespec unixTimestamp {};
    if (clock_gettime(CLOCK_REALTIME, &unixTimestamp) != 0) {
      unixTimestamp.tv_sec = 0;
      unixTimestamp.tv_nsec = 0;
    }

    rec.timestamp =
        static_cast<uint64_t>(unixTimestamp.tv_sec) * 1000000000ULL +
        static_cast<uint64_t>(unixTimestamp.tv_nsec);

    constexpr size_t packageSize = 4 * sizeof(float);
    rec.length = static_cast<uint8_t>(packageSize);

    std::array<float, 4> mag_values = {
        static_cast<float>(raw_x) / Gauss_16,
        static_cast<float>(raw_y) / Gauss_16,
        static_cast<float>(raw_z) / Gauss_16,
        static_cast<float>(raw_temp) / Conversion_1 + Conversion_2};
    std::memcpy(rec.data.data(), mag_values.data(), packageSize);

    uint64_t generator = gSnapshot.generation.load(std::memory_order_relaxed);
    gSnapshot.generation.store(generator + 1, std::memory_order_release);

    gSnapshot.magX = static_cast<float>(raw_x) / Gauss_16;
    gSnapshot.magY = static_cast<float>(raw_y) / Gauss_16;
    gSnapshot.magZ = static_cast<float>(raw_z) / Gauss_16;
    gSnapshot.magTemp =
        static_cast<float>(raw_temp) / Conversion_1 + Conversion_2;

    gSnapshot.generation.store(generator + 2, std::memory_order_release);

#else
    float x = magDist(rng);
    float y = magDist(rng);
    float z = magDist(rng);

    uint64_t generator = gSnapshot.generation.load(std::memory_order_relaxed);
    gSnapshot.generation.store(generator + 1, std::memory_order_release);

    gSnapshot.magX = x;
    gSnapshot.magY = y;
    gSnapshot.magZ = z;

    gSnapshot.generation.store(generator + 2, std::memory_order_release);

    // SPDLOG_INFO("[MAGNETOMETER] MAGx = {} MAGy = {} MAGz = {}", x, y, z);

    sober::logger::LogEntry rec{};
    struct timespec unixTimestamp {};
    if (clock_gettime(CLOCK_REALTIME, &unixTimestamp) != 0) {
      unixTimestamp.tv_sec = 0;
      unixTimestamp.tv_nsec = 0;
    }

    rec.timestamp =
        static_cast<uint64_t>(unixTimestamp.tv_sec) * 1000000000ULL +
        static_cast<uint64_t>(unixTimestamp.tv_nsec);

    constexpr size_t packageSize = 4 * sizeof(float);
    rec.length = static_cast<uint8_t>(packageSize);

    std::array<float, 4> mag_values = {x, y, z, 30.0F};
    std::memcpy(rec.data.data(), mag_values.data(), packageSize);
#endif

    logData(rec);

    std::unique_lock<std::mutex> lk(cvMutex_);
    cv_.wait_until(lk, next_wakeup, [&] { return stoken.stop_requested(); });
    next_wakeup += sample_period;
  }
}

}  // namespace sober::housekeeping
