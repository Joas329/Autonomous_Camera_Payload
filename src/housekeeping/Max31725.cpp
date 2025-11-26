#include "housekeeping/Max31725.hpp"
#include "housekeeping/SensorSnapshot.hpp"

namespace sober::housekeeping {

/** Helper: path buffer length */
static constexpr size_t I2C_PATH_MAX = 32;

Max31725::Max31725(sober::eventbus::EventBus& bus, uint8_t i2cBus,
                   uint8_t i2cAddr,
                   std::shared_ptr<sober::logger::Logger::Queue> queue,
                   uint8_t one_shot, uint8_t timeout_enable,
                   uint8_t extendedDataFormat_enable, int faults_queue,
                   uint8_t osPolarity_enable, uint8_t interrupt_enable)
    : m_bus_(bus),
      m_nFd(-1),
      m_nI2cBus(i2cBus),
      m_nI2cAddr(i2cAddr),
      m_pQueue(std::move(queue)) {
  MAX31725_SetConfig_ShutdownMode(0);

  MAX31725_SetConfig_OneShot(one_shot);
  MAX31725_SetConfig_Timeout(timeout_enable);
  MAX31725_SetConfig_ExtendedDataFormat(extendedDataFormat_enable);
  MAX31725_SetConfig_FaultsQueue(faults_queue);
  MAX31725_SetConfig_OSPolarity(osPolarity_enable);
  MAX31725_SetConfig_InterruptMode(interrupt_enable);
}

Max31725::~Max31725() {
  stop();
  if (m_nFd >= 0) {
    close(m_nFd);
    m_nFd = -1;
  }
}

bool Max31725::init_i2c() {
  if (m_nFd >= 0)
    return true;

  char path[I2C_PATH_MAX];
  snprintf(path, sizeof(path), "/dev/i2c-%u", static_cast<unsigned>(m_nI2cBus));

  m_nFd = open(path, O_RDWR);
  if (m_nFd < 0) {
    SPDLOG_ERROR("[MAX31725] Failed to open I2C bus {} for address {}", path,
                 m_nI2cAddr);
    return false;
  }

  if (ioctl(m_nFd, I2C_SLAVE, m_nI2cAddr) < 0) {
    SPDLOG_ERROR("[MAX31725] ioctl(I2C_SLAVE, 0x{:02X}) failed", m_nI2cAddr);
    close(m_nFd);
    m_nFd = -1;
    return false;
  }
  return true;
}

bool Max31725::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t len) {
  if (m_nFd < 0) {
    SPDLOG_ERROR("[MAX31725] readRegisters: i2c fd not open for address {}",
                 m_nI2cAddr);
    return false;
  }
  if (len == 0) {
    SPDLOG_ERROR("[MAX31725] readRegisters: len==0 for address {}", m_nI2cAddr);
    return false;
  }

  // write register address
  if (write(m_nFd, &reg, 1) != 1) {
    SPDLOG_ERROR("[MAX31725] I2C write (set reg 0x{:02X}) failed on address {}",
                 reg, m_nI2cAddr);
    return false;
  }

  // read bytes
  if (read(m_nFd, buffer, len) != len) {
    SPDLOG_ERROR("[MAX31725] I2C read failed (reg 0x{:02X} len {}) : {}", reg,
                 len, m_nI2cAddr);
    return false;
  }
  return true;
}

/** Low-level write: write buffer of length len (first byte usually register) */
bool Max31725::writeRegisters(const uint8_t* buffer, uint8_t len) {
  if (m_nFd < 0) {
    SPDLOG_ERROR("[MAX31725] writeRegisters: i2c fd not open");
    return false;
  }

  if (write(m_nFd, buffer, len) != (ssize_t)len) {
    SPDLOG_ERROR("[MAX31725] I2C write failed (len={}) for address {}",
                 (int)len, m_nI2cAddr);
    return false;
  }
  return true;
}

int32_t Max31725::readTemperature() {
  uint8_t buf[2];
  if (!readRegisters(REG_TEMP_MSB, buf, 2)) {
    return -1;
  }
  int16_t raw = static_cast<int16_t>((buf[0] << 8) | buf[1]);
  float tempC = static_cast<float>(raw) *
                static_cast<float>(0.00390625);  // raw * (1/256)
  int32_t scaled = static_cast<int32_t>(
      tempC * static_cast<float>(100.0));  // scaled as original
  return scaled;
}

int32_t Max31725::readTHYST() {
  uint8_t buf[2] = {0, 0};
  if (!readRegisters(REG_THYST, buf, 2)) {
    return -1;
  }
  int16_t raw = static_cast<int16_t>((buf[0] << 8) | buf[1]);
  int32_t scaled =
      static_cast<int32_t>((static_cast<int32_t>(raw) + 32768) * 256);
  return scaled;
}

int32_t Max31725::readTOS() {
  uint8_t buf[2] = {0, 0};
  if (!readRegisters(REG_TOS, buf, 2)) {
    return -1;
  }
  int16_t raw = static_cast<int16_t>((buf[0] << 8) | buf[1]);
  int32_t scaled =
      static_cast<int32_t>((static_cast<int32_t>(raw) + 32768) * 256);
  return scaled;
}

bool Max31725::writeTHYST(uint8_t msb, uint8_t lsb) {
  uint8_t buf[3] = {REG_THYST, msb, lsb};
  return writeRegisters(buf, 3);
}

bool Max31725::writeTOS(uint8_t msb, uint8_t lsb) {
  uint8_t buf[3] = {REG_TOS, msb, lsb};
  return writeRegisters(buf, 3);
}

uint8_t Max31725::getConfig() {
  uint8_t reg = REG_CONFIG;
  uint8_t data = 0;
  if (!readRegisters(reg, &data, 1)) {
    return 0;
  }
  return data;
}

bool Max31725::writeConfig(uint8_t data) {
  uint8_t buf[2] = {REG_CONFIG, data};
  return writeRegisters(buf, 2);
}

/**
 * @brief Start background sampling thread (same pattern as BME280)
 */
bool Max31725::start() {
#ifndef DEBUG_MAX31725
  if (!init_i2c()) {
    return false;
  }

#endif

  m_sensorThread_ = std::jthread([this](std::stop_token st) { this->run(st); });
  return true;
}

/** @brief Stop background thread */
void Max31725::stop() {
  m_sensorThread_.request_stop();
  cv_.notify_all();
}

/**
 * @brief Main loop: read sensor periodically and publish/log values
 */
void Max31725::run(std::stop_token stoken) {
#ifndef DEBUG_MAX31725
  if (!init_i2c()) {
    SPDLOG_CRITICAL("[MAX31725] Failed to initialize I2C for address {}",
                    m_nI2cAddr);
    return;
  }
#endif

  auto next_wakeup = std::chrono::steady_clock::now() + sample_period;

  while (!stoken.stop_requested()) {
#ifndef DEBUG_MAX31725
    int32_t tempScaled = readTemperature();  // --- ok
    int32_t thyst = readTHYST();             // --- ok
    int32_t tos = readTOS();                 // --- ok

    // Convert scaled values back to floating for logging
    float tempC =
        (tempScaled == -1)
            ? NAN
            : (static_cast<float>(tempScaled) / static_cast<float>(100.0));
    float thystC =
        (thyst == -1)
            ? NAN
            : (static_cast<float>(thyst) / static_cast<float>(256.0) -
               static_cast<float>(32768.0));  // reverse formula approx
    float tosC = (tos == -1)
                     ? NAN
                     : (static_cast<float>(tos) / static_cast<float>(256.0) -
                        static_cast<float>(32768.0));

#else
    float tempC = 25.69;
    float thystC = tempC - 2.0;
    float tosC = tempC + 10.0;
#endif

    SPDLOG_INFO("[MAX31725] Temperature: {:.2f} �C THYST: {:.2f} TOS: {:.2f}",
                tempC, thystC, tosC);

    sober::logger::LogEntry rec{};  // --- depends on your logger struct
    struct timespec unixTimestamp {};
    if (clock_gettime(CLOCK_REALTIME, &unixTimestamp) != 0) {
      unixTimestamp.tv_sec = 0;
      unixTimestamp.tv_nsec = 0;
    }
    rec.timestamp =
        static_cast<uint64_t>(unixTimestamp.tv_sec) * 1000000000ULL +
        static_cast<uint64_t>(unixTimestamp.tv_nsec);

    uint64_t generator = gSnapshot.generation.load(std::memory_order_relaxed);
    gSnapshot.generation.store(generator + 1, std::memory_order_release);

    switch (tempIdx) {
      case (1U):
        gSnapshot.tempC1 = tempC;
        // gSnapshot.tempC1 = 15.0;
        gSnapshot.thystC1 = thystC;
        gSnapshot.tosC1 = tosC;
        break;
      case (2U):
        gSnapshot.tempC2 = tempC;
        gSnapshot.thystC2 = thystC;
        gSnapshot.tosC2 = tosC;
        break;
      default:
        SPDLOG_WARN("[MAX31725] Unexpected temp index");
        break;
    }
    gSnapshot.generation.store(generator + 2, std::memory_order_release);

    // pack three doubles into rec.data (adjust if your LogEntry has different layout)
    rec.length =
        static_cast<uint8_t>(sizeof(tempC) + sizeof(thystC) + sizeof(tosC));
    std::memcpy(rec.data.data(), &tempC, sizeof(tempC));
    std::memcpy(rec.data.data() + sizeof(tempC), &thystC, sizeof(thystC));
    std::memcpy(rec.data.data() + sizeof(tempC) + sizeof(thystC), &tosC,
                sizeof(tosC));

    // logData equivalent (same pattern as BME280)
    if (m_pQueue) {  // push to logger queue, notify if needed
      const std::size_t HIGH_WATER =
          m_pQueue->capacity() * 8 / 10;  // may be different
      std::size_t qsz = m_pQueue->size();
      std::size_t qcap = m_pQueue->capacity();
      bool pushed = m_pQueue->try_emplace(
          rec);  // try_emplace behavior depends on your queue type

      if (!pushed) {
        SPDLOG_WARN(
            "[MAX31725] logger queue overflow, dropping sample (size={}/{})",
            qsz, qcap);
        // notify logger
        sober::logger::Logger::instance()
            .notify();  // --- missing: confirm Logger API
      } else if (qsz >= HIGH_WATER) {
        sober::logger::Logger::instance()
            .notify();  // --- missing: confirm Logger API
      }
    }  // end m_pQueue

    // wait until next sample or stop requested
    std::unique_lock<std::mutex> lk(cvMutex_);
    cv_.wait_until(lk, next_wakeup, [&] { return stoken.stop_requested(); });
    next_wakeup += sample_period;
  }
}
void Max31725::MAX31725_SetConfig_OneShot(uint8_t active) {
  uint8_t data = getConfig();
  if (active) {
    data |= static_cast<uint8_t>(1 << 7);
  } else {
    data &= static_cast<uint8_t>(~(static_cast<uint8_t>(1 << 7)));
  }
  writeConfig(data);
}

void Max31725::MAX31725_SetConfig_Timeout(uint8_t enabled) {
  uint8_t data = getConfig();
  if (enabled) {
    data |= static_cast<uint8_t>(1 << 6);
  } else {
    data &= static_cast<uint8_t>(~(static_cast<uint8_t>(1 << 6)));
  }
  writeConfig(data);
}

void Max31725::MAX31725_SetConfig_ExtendedDataFormat(uint8_t enabled) {
  uint8_t data = getConfig();
  if (enabled) {
    data |= static_cast<uint8_t>(1 << 5);
  } else {
    data &= static_cast<uint8_t>(~(static_cast<uint8_t>(1 << 5)));
  }
  writeConfig(data);
}

void Max31725::MAX31725_SetConfig_FaultsQueue(int faults) {
  uint8_t data = getConfig();
  if (faults) {
    data |= static_cast<uint8_t>(1 << 4);
  } else {
    data &= static_cast<uint8_t>(~(static_cast<uint8_t>(1 << 4)));
  }
  writeConfig(data);
}

void Max31725::MAX31725_SetConfig_OSPolarity(uint8_t active_high) {
  uint8_t data = getConfig();
  if (active_high) {
    data |= static_cast<uint8_t>(1 << 3);
  } else {
    data &= static_cast<uint8_t>(~(static_cast<uint8_t>(1 << 3)));
  }
  writeConfig(data);
}

void Max31725::MAX31725_SetConfig_InterruptMode(uint8_t interrupt_active) {
  uint8_t data = getConfig();
  if (interrupt_active) {
    data |= static_cast<uint8_t>(1 << 2);
  } else {
    data &= static_cast<uint8_t>(~(static_cast<uint8_t>(1 << 2)));
  }
  writeConfig(data);
}

void Max31725::MAX31725_SetConfig_ShutdownMode(uint8_t shutdown_active) {
  uint8_t data = getConfig();
  if (shutdown_active) {
    data |= static_cast<uint8_t>(1 << 1);
  } else {
    data &= static_cast<uint8_t>(~(static_cast<uint8_t>(1 << 1)));
  }
  writeConfig(data);
}

}  // namespace sober::housekeeping