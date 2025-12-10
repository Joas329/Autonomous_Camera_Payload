#include "housekeeping/Ina237.hpp"
#include <cstdint>
#include "housekeeping/SensorSnapshot.hpp"
namespace sober::housekeeping {

Ina237::Ina237(sober::eventbus::EventBus& bus, uint8_t i2cBus, uint8_t i2cAddr,
               std::shared_ptr<sober::logger::Logger::Queue> queue,
               uint8_t voltageIndex)
    : voltageIndex_(voltageIndex),
      m_bus(bus),
      m_nFd(-1),
      m_nI2cBus(i2cBus),
      m_nI2cAddr(i2cAddr),
      m_nAdcRange(INA237_ADC_RANGE_163_84mV),
      m_dShuntRes(0.0),
      m_dCurrentLSB(0),
      queue_(std::move(queue)) {

  // Default current LSB calculation (you can adjust based on your expected current range)
  m_dCurrentLSB = static_cast<float>(0.001);  // 1 mA per bit default
}

Ina237::~Ina237() {
  stop();
  if (m_nFd >= 0) {
    close(m_nFd);
  }
}

bool Ina237::init_i2c() {

  if (m_nFd >= 0) {
    return true;
  }

  char filename[20];
  snprintf(filename, sizeof(filename), "/dev/i2c-%d", m_nI2cBus);

  if ((m_nFd = open(filename, O_RDWR)) < 0) {
    SPDLOG_CRITICAL("[Current monitor] Failed to open I2C bus for address {}",
                    m_nI2cAddr);
    return false;
  }

  if (ioctl(m_nFd, I2C_SLAVE, m_nI2cAddr) < 0) {
    SPDLOG_CRITICAL("[Current monitor] Failed to set I2C slave address {}",
                    m_nI2cAddr);
    close(m_nFd);
    m_nFd = -1;
    return false;
  }

  // Reset the device
  if (!writeRegister(INA237_REG_CONFIG, (static_cast<uint16_t>(1 << 15)))) {
    SPDLOG_CRITICAL("[Current monitor] Failed to reset device for address {}",
                    m_nI2cAddr);
    close(m_nFd);
    m_nFd = -1;
    return false;
  }

  // Wait for reset to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  return true;
}

bool Ina237::start() {
#ifndef DEBUG_INA
  if (!init_i2c()) {  // --- missing function
    return false;
  }
#endif

  m_sensorThread = std::jthread([this](std::stop_token st) { this->run(st); });
  return true;
}

void Ina237::stop() {
  m_sensorThread.request_stop();
  cv_.notify_all();
}

void Ina237::run(std::stop_token stoken) {
#ifndef DEBUG_INA
  if (!init_i2c()) {  // --- missing function
    SPDLOG_CRITICAL("[Current monitor] Failed to initialize INA237 {}",
                    m_nI2cAddr);
    return;
  }
#endif

  auto next_wakeup = std::chrono::steady_clock::now() + m_nSamplePeriod;

  while (!stoken.stop_requested()) {
#ifndef DEBUG_INA
    // Read raw data from registers
    float current = getCurrent();
    float busVoltage = getBusVoltage();
    float shuntVoltage = getShuntVoltage();
    float temperature = getTemperature();
    uint64_t generator = gSnapshot.generation.load(std::memory_order_relaxed);
    gSnapshot.generation.store(generator + 1, std::memory_order_release);

    switch (voltageIndex_) {
      case (0U):
        gSnapshot.voltage3v3 = busVoltage;
        gSnapshot.current3v3 = current;
        break;
      case (1U):
        gSnapshot.voltage5 = busVoltage;
        gSnapshot.current5 = current;
        break;
      case (2U):
        gSnapshot.voltage12 = busVoltage;
        gSnapshot.current12 = current;
        break;
      default:
        SPDLOG_WARN("[Current monitor] Unexpected voltageIndex_: {}",
                    voltageIndex_);
        break;
    }
    gSnapshot.generation.store(generator + 2, std::memory_order_release);

#else
    float current = 0.00125;
    float busVoltage = 3.29;
    float shuntVoltage = 1.17;
    float temperature = 29.69;
#endif

    // SPDLOG_INFO(
    //     "[Current monitor] Current: {} A, Bus: {} V, Shunt: {} V, Temp: {} C",
    //     current, busVoltage, shuntVoltage, temperature);

    sober::logger::LogEntry rec{};
    struct timespec unixTimestamp {};
    if (clock_gettime(CLOCK_REALTIME, &unixTimestamp) != 0) {
      unixTimestamp.tv_sec = 0;
      unixTimestamp.tv_nsec = 0;
    }

    rec.timestamp =
        static_cast<uint64_t>(unixTimestamp.tv_sec) * 1000000000ULL +
        static_cast<uint64_t>(unixTimestamp.tv_nsec);

    rec.length =
        static_cast<uint8_t>(sizeof(current) + sizeof(busVoltage) +
                             sizeof(shuntVoltage) + sizeof(temperature));

    std::memcpy(rec.data.data(), &current, sizeof(current));
    std::memcpy(rec.data.data() + sizeof(current), &busVoltage,
                sizeof(busVoltage));
    std::memcpy(rec.data.data() + sizeof(current) + sizeof(busVoltage),
                &shuntVoltage, sizeof(shuntVoltage));
    std::memcpy(rec.data.data() + sizeof(current) + sizeof(busVoltage) +
                    sizeof(shuntVoltage),
                &temperature, sizeof(temperature));

    logData(rec);  // --- missing function

    std::unique_lock<std::mutex> lk(cvMutex_);
    cv_.wait_until(lk, next_wakeup, [&] { return stoken.stop_requested(); });
    next_wakeup += m_nSamplePeriod;
  }
}

void Ina237::logData(const sober::logger::LogEntry& rec) {
  if (queue_) {
    const std::size_t HIGH_WATER = queue_->capacity() * 8 / 10;
    std::size_t qsz = queue_->size();
    std::size_t qcap = queue_->capacity();

    bool pushed = queue_->try_emplace(rec);

    if (!pushed) {
      SPDLOG_WARN(
          "[Current monitor] queue overflow, dropping sample (size={}/{})", qsz,
          qcap);
      SPDLOG_DEBUG("[Current monitor] notifying logger due to overflow");
      sober::logger::Logger::instance().notify();
    } else if (qsz >= HIGH_WATER) {
      SPDLOG_DEBUG("[Current monitor] high-water mark reached: {}/{} ({}%)",
                   qsz, qcap, (qsz * 100) / qcap);
      sober::logger::Logger::instance().notify();
    }
  }
}

void Ina237::reset() {
  writeRegister(INA237_REG_CONFIG, (1 << 15));
  m_dCurrentLSB = static_cast<float>(0.001);
  m_nAdcRange = INA237_ADC_RANGE_163_84mV;
}

float Ina237::getCurrent() {
  uint8_t buffer[2];
  if (!readRegisters(INA237_REG_CURRENT, buffer, 2)) {
    SPDLOG_ERROR("[Current monitor] Failed to read current for address {}",
                 m_nI2cAddr);
    return NAN;
  }
  int16_t raw = ((int16_t)buffer[0] << 8) | buffer[1];
  return raw * m_dCurrentLSB;
}

float Ina237::getBusVoltage() {
  uint8_t buffer[2];
  if (!readRegisters(INA237_REG_VBUS, buffer, 2)) {
    SPDLOG_ERROR("[Current monitor] Failed to read bus voltage for address {}",
                 m_nI2cAddr);
    return NAN;
  }
  uint16_t raw = ((uint16_t)buffer[0] << 8) | buffer[1];
  return raw * INA237_VBUS_LSB_RES;
}

float Ina237::getShuntVoltage() {
  uint8_t buffer[2];
  if (!readRegisters(INA237_REG_VSHUNT, buffer, 2)) {
    SPDLOG_ERROR(
        "[Current monitor] Failed to read shunt voltage for address {}",
        m_nI2cAddr);
    return NAN;
  }
  int16_t raw = ((int16_t)buffer[0] << 8) | buffer[1];
  return raw * INA237_VSHUNT_LSB_RES[m_nAdcRange];
}

float Ina237::getTemperature() {
  uint8_t buffer[2];
  if (!readRegisters(INA237_REG_DIETEMP, buffer, 2)) {
    SPDLOG_ERROR("[Current monitor] Failed to read temperature for address {}",
                 m_nI2cAddr);
    return NAN;
  }
  int16_t raw = ((int16_t)buffer[0] << 8) | buffer[1];
  return raw * INA237_TEMP_LSB_RES;
}

// ---- Low level I2C helpers ----
bool Ina237::writeRegister(uint8_t reg, uint16_t data) {
  uint8_t buffer[3];
  buffer[0] = reg;
  buffer[1] = (data >> 8) & 0xFF;
  buffer[2] = data & 0xFF;
  if (write(m_nFd, buffer, 3) != 3) {
    SPDLOG_ERROR("[Current monitor] Failed to write to register for address {}",
                 m_nI2cAddr);
    return false;
  }
  return true;
}

bool Ina237::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length) {
  if (length < 1 || length > 3) {
    SPDLOG_ERROR("[Current monitor] Invalid read length: {} for address {}",
                 length, m_nI2cAddr);
    return false;
  }

  // Set register address
  if (write(m_nFd, &reg, 1) != 1) {
    SPDLOG_ERROR("[Current monitor] I2C write (set reg) failed for address {}",
                 m_nI2cAddr);
    return false;
  }

  // Read register contents
  if (read(m_nFd, buffer, length) != length) {
    SPDLOG_ERROR("[Current monitor] I2C read failed for address {}",
                 m_nI2cAddr);
    return false;
  }

  return true;
}
}  // namespace sober::housekeeping