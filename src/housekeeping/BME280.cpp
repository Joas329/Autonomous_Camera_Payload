#include "housekeeping/BME280.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <random>
#include <thread>
#include <utility>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <spdlog/spdlog.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "eventbus/Event.hpp"
#include "eventbus/EventBus.hpp"
#include "housekeeping/SensorSnapshot.hpp"
#include "logger/Logger.hpp"

namespace sober::housekeeping {

/**
 * @brief Constructor for the BME280 sensor driver
 * @param bus Reference to the event bus for publishing sensor data
 * @param i2c_bus I2C bus number
 * @param i2c_addr I2C device address
 */
BME280::BME280(sober::eventbus::EventBus& bus, uint8_t i2c_bus,
               uint8_t i2c_addr,
               std::shared_ptr<sober::logger::Logger::Queue> queue)
    : m_bus(bus),
      m_i2cFd(-1),
      m_i2cBus(i2c_bus),
      m_i2cAddr(i2c_addr),
      m_tFine(0),
      m_calibData({}),
      m_settings({}),
      m_readings({}),
      queue_(std::move(queue)) {
  m_settings.m_tempOversampling = Oversampling::X1;
  m_settings.m_pressOversampling = Oversampling::X1;
  m_settings.m_humOversampling = Oversampling::X1;
  m_settings.m_mode = Mode::SLEEP;
  m_settings.m_filter = Filter::OFF;
  m_settings.m_standbyTime = StandbyTime::MS_0_5;
  m_settings.m_measurementIntervalMs = 1000;
}

/**
 * @brief Destructor for the BME280 sensor driver
 * Ensures proper cleanup of resources
 */
BME280::~BME280() {
  stop();
  if (m_i2cFd != -1) {
    close(m_i2cFd);
  }
}

/**
 * @brief Starts the BME280 sensor
 * Initializes I2C communication and starts the sensor thread
 */
bool BME280::start() {
#ifndef DEBUG_BME
  if (!init_i2c()) {
    return false;
  }

  // Set default settings
  m_settings.m_tempOversampling = Oversampling::X1;
  m_settings.m_pressOversampling = Oversampling::X1;
  m_settings.m_humOversampling = Oversampling::X1;
  m_settings.m_mode = Mode::NORMAL;
  m_settings.m_filter = Filter::OFF;
  m_settings.m_standbyTime = StandbyTime::MS_0_5;
  m_settings.m_measurementIntervalMs = 1000;

  // Apply settings
  if (!set_mode(m_settings.m_mode) ||
      !set_oversampling(m_settings.m_tempOversampling,
                        m_settings.m_pressOversampling,
                        m_settings.m_humOversampling) ||
      !set_filter(m_settings.m_filter) ||
      !set_standby_time(m_settings.m_standbyTime)) {
    return false;
  }
#endif
  m_sensorThread =
      std::jthread([this](std::stop_token stoken) { this->run(stoken); });
  return true;
}

/**
 * @brief Stops the BME280 sensor
 * Stops the sensor thread and cleans up resources
 */
void BME280::stop() {
  m_sensorThread.request_stop();
  cv_.notify_all();
}

/**
 * @brief Reads the current temperature from the sensor
 * @return Temperature in degrees Celsius, or 0.0 if read failed
 */
float BME280::read_temperature() {
  const std::lock_guard<std::mutex> lock(m_readingsMutex);
  return m_readings.m_temperature;
}

/**
 * @brief Reads the current pressure from the sensor
 * @return Pressure in Pa, or 0.0 if read failed
 */
float BME280::read_pressure() {
  const std::lock_guard<std::mutex> lock(m_readingsMutex);
  return m_readings.m_pressure;
}

/**
 * @brief Reads the current humidity from the sensor
 * @return Humidity in %RH, or 0.0 if read failed
 */
float BME280::read_humidity() {
  const std::lock_guard<std::mutex> lock(m_readingsMutex);
  return m_readings.m_humidity;
}

/**
 * @brief Sets the operating mode of the sensor
 * @param mode The desired operating mode
 * @return true if successful, false otherwise
 */
bool BME280::set_mode(Mode mode) {
  uint8_t value = 0;
  if (!read_registers(kRegCtrlMeas, &value, 1)) {
    return false;
  }

  value = (value & 0xFC) | static_cast<uint8_t>(mode);
  if (!write_register(kRegCtrlMeas, value)) {
    return false;
  }

  m_settings.m_mode = mode;
  return true;
}

/**
 * @brief Sets the oversampling rates for temperature, pressure, and humidity
 * @param temp Temperature oversampling rate
 * @param press Pressure oversampling rate
 * @param hum Humidity oversampling rate
 * @return true if successful, false otherwise
 */
bool BME280::set_oversampling(Oversampling temp, Oversampling press,
                              Oversampling hum) {
  // Set humidity oversampling
  if (!write_register(kRegCtrlHum, static_cast<uint8_t>(hum))) {
    return false;
  }

  // Set temperature and pressure oversampling
  const uint8_t ctrlMeas =
      (static_cast<uint8_t>(static_cast<uint8_t>(temp)
                            << 5)) |  // [7:5] temperature oversampling
      static_cast<uint8_t>(static_cast<uint8_t>(press)
                           << 2) |              // [4:2] pressure oversampling
      static_cast<uint8_t>(m_settings.m_mode);  // [1:0] mode
  if (!write_register(kRegCtrlMeas, ctrlMeas)) {
    return false;
  }

  m_settings.m_tempOversampling = temp;
  m_settings.m_pressOversampling = press;
  m_settings.m_humOversampling = hum;
  return true;
}

/**
 * @brief Sets the filter coefficient for the sensor
 * @param filter The desired filter coefficient
 * @return true if successful, false otherwise
 */
bool BME280::set_filter(Filter filter) {
  // Read current config
  uint8_t config = read_register(kRegConfig);
  if (config == 0xFF) {  // Error reading register
    return false;
  }

  // Update filter bits (4:2) while preserving standby time (7:5) and SPI-3-wire (0)
  config = static_cast<uint8_t>(
               config &
               0xE3) |  // Clear filter bits, preserve standby and SPI-3-wire
           static_cast<uint8_t>(static_cast<uint8_t>(filter)
                                << 2);  // Set filter bits
  if (!write_register(kRegConfig, config)) {
    return false;
  }

  m_settings.m_filter = filter;
  return true;
}

/**
 * @brief Sets the standby time for the sensor
 * @param time The desired standby time
 * @return true if successful, false otherwise
 */
bool BME280::set_standby_time(StandbyTime time) {
  // Read current config
  uint8_t config = read_register(kRegConfig);
  if (config == 0xFF) {  // Error reading register
    return false;
  }

  // Update standby time bits (7:5) while preserving filter (4:2) and SPI-3-wire (0)
  config = static_cast<uint8_t>(
               config &
               0x1F) |  // Clear standby bits, preserve filter and SPI-3-wire
           static_cast<uint8_t>(static_cast<uint8_t>(time)
                                << 5);  // Set standby bits
  if (!write_register(kRegConfig, config)) {
    return false;
  }

  m_settings.m_standbyTime = time;
  return true;
}

/**
 * @brief Initializes the I2C communication with the sensor
 * @return true if successful, false otherwise
 */
bool BME280::init_i2c() {
  if (m_i2cFd >= 0) {
    return true;
  }

  char filename[20];
  snprintf(filename, sizeof(filename), "/dev/i2c-%d", m_i2cBus);

  if ((m_i2cFd = open(filename, O_RDWR)) < 0) {
    SPDLOG_CRITICAL("[TEMPERATURE] Failed to open I2C bus");
    return false;
  }

  if (ioctl(m_i2cFd, I2C_SLAVE, m_i2cAddr) < 0) {
    SPDLOG_CRITICAL("[TEMPERATURE] Failed to set I2C slave address");
    close(m_i2cFd);
    m_i2cFd = -1;
    return false;
  }

  // Check chip ID
  uint8_t chipId = 0;
  if (!read_registers(kRegId, &chipId, 1) || chipId != kChipId) {
    SPDLOG_CRITICAL("[TEMPERATURE] Invalid chip ID");
    close(m_i2cFd);
    m_i2cFd = -1;
    return false;
  }

  // Reset the device
  if (!write_register(kRegReset, kSoftReset)) {
    SPDLOG_CRITICAL("[TEMPERATURE] Failed to reset device");
    close(m_i2cFd);
    m_i2cFd = -1;
    return false;
  }

  // Wait for reset to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Read calibration data
  if (!read_calibration_data()) {
    SPDLOG_CRITICAL("[TEMPERATURE] Failed to read calibration data");
    close(m_i2cFd);
    m_i2cFd = -1;
    return false;
  }

  return true;
}

/**
 * @brief Reads calibration data from the sensor
 * @return true if successful, false otherwise
 */
bool BME280::read_calibration_data() {
  std::array<uint8_t, 24> buffer{};

  // Read temperature calibration data
  if (!read_registers(kRegDigT1, buffer.data(), 6)) {
    return false;
  }
  m_calibData.m_digT1 =
      static_cast<uint16_t>(buffer[1] << kBitShift8) | buffer[0];
  m_calibData.m_digT2 =
      static_cast<int16_t>(buffer[3] << kBitShift8) | buffer[2];
  m_calibData.m_digT3 =
      static_cast<int16_t>(buffer[5] << kBitShift8) | buffer[4];

  // Read pressure calibration data
  if (!read_registers(kRegDigP1, buffer.data(), 18)) {
    return false;
  }
  m_calibData.m_digP1 =
      static_cast<uint16_t>(buffer[1] << kBitShift8) | buffer[0];
  m_calibData.m_digP2 =
      static_cast<int16_t>(buffer[3] << kBitShift8) | buffer[2];
  m_calibData.m_digP3 =
      static_cast<int16_t>(buffer[5] << kBitShift8) | buffer[4];
  m_calibData.m_digP4 =
      static_cast<int16_t>(buffer[7] << kBitShift8) | buffer[6];
  m_calibData.m_digP5 =
      static_cast<int16_t>(buffer[9] << kBitShift8) | buffer[8];
  m_calibData.m_digP6 =
      static_cast<int16_t>(buffer[11] << kBitShift8) | buffer[10];
  m_calibData.m_digP7 =
      static_cast<int16_t>(buffer[13] << kBitShift8) | buffer[12];
  m_calibData.m_digP8 =
      static_cast<int16_t>(buffer[15] << kBitShift8) | buffer[14];
  m_calibData.m_digP9 =
      static_cast<int16_t>(buffer[17] << kBitShift8) | buffer[16];

  // Read humidity calibration data
  if (!read_registers(kRegDigH1, &m_calibData.m_digH1, 1)) {
    return false;
  }

  if (!read_registers(kRegDigH2, buffer.data(), 2)) {
    return false;
  }
  m_calibData.m_digH2 =
      static_cast<int16_t>(buffer[1] << kBitShift8) | buffer[0];

  if (!read_registers(kRegDigH3, &m_calibData.m_digH3, 1)) {
    return false;
  }

  if (!read_registers(kRegDigH4, buffer.data(), 2)) {
    return false;
  }
  m_calibData.m_digH4 =
      static_cast<int16_t>((buffer[0] << kBitShift4) | (buffer[1] & 0x0F));

  if (!read_registers(kRegDigH5, buffer.data(), 2)) {
    return false;
  }
  m_calibData.m_digH5 = static_cast<int16_t>((buffer[1] << kBitShift4) |
                                             (buffer[0] >> kBitShift4));

  return read_registers(kRegDigH6,
                        reinterpret_cast<uint8_t*>(&m_calibData.m_digH6), 1);
}

/**
 * @brief Writes a value to a sensor register
 * @param reg Register address
 * @param value Value to write
 * @return true if successful, false otherwise
 */
bool BME280::write_register(uint8_t reg, uint8_t value) {
  uint8_t buffer[2] = {reg, value};
  if (write(m_i2cFd, buffer, 2) != 2) {
    SPDLOG_ERROR("[TEMPERATURE] Failed to write to register");
    return false;
  }
  return true;
}

/**
 * @brief Reads multiple registers from the sensor
 * @param reg Starting register address
 * @param buffer Buffer to store read data
 * @param length Number of bytes to read
 * @return true if successful, false otherwise
 */
bool BME280::read_registers(uint8_t reg, uint8_t* buffer, uint8_t length) {
  if (write(m_i2cFd, &reg, 1) != 1) {
    SPDLOG_ERROR("[TEMPERATURE] Failed to write register address");
    return false;
  }

  if (read(m_i2cFd, buffer, length) != length) {
    SPDLOG_ERROR("[TEMPERATURE] Failed to read from register");
    return false;
  }
  return true;
}

/**
 * @brief Reads a single register from the sensor
 * @param reg Register address
 * @return Register value, or 0 if read failed
 */
uint8_t BME280::read_register(uint8_t reg) {
  uint8_t value = 0;
  if (!read_registers(reg, &value, 1)) {
    SPDLOG_ERROR("[TEMPERATURE] Failed to read register");
    return 0;
  }
  return value;
}

/**
 * @brief Compensates raw temperature data
 * @param adc_T Raw temperature data
 * @return Compensated temperature in degrees Celsius
 */
float BME280::compensate_temperature(int32_t adc_T) {
  int32_t var1 = 0;
  int32_t var2 = 0;

  var1 = ((((adc_T >> 3) - (static_cast<int32_t>(m_calibData.m_digT1) << 1))) *
          static_cast<int32_t>(m_calibData.m_digT2)) >>
         11;

  constexpr int kBitShift14 = 14;
  int32_t t = static_cast<int32_t>(adc_T >> 4) -
              static_cast<int32_t>(m_calibData.m_digT1);
  int64_t t64 = static_cast<int64_t>(t);
  int64_t var2_64 =
      ((t64 * t64) >> 12) * static_cast<int64_t>(m_calibData.m_digT3);
  var2 = static_cast<int32_t>(var2_64 >> kBitShift14);

  m_tFine = var1 + var2;
  float temperature = static_cast<float>((m_tFine * 5 + 128) >> 8);
  return temperature / kCompensation100;
}

/**
 * @brief Compensates raw pressure data
 * @param adc_P Raw pressure data
 * @return Compensated pressure in Pa
 */
float BME280::compensate_pressure(int32_t adc_P) {
  int64_t var1 = 0;
  int64_t var2 = 0;
  int64_t pressure = 0;

  var1 =
      static_cast<int64_t>(m_tFine) - static_cast<int64_t>(kCompensation128000);
  var2 = var1 * var1 * static_cast<int64_t>(m_calibData.m_digP6);
  var2 = var2 + ((var1 * static_cast<int64_t>(m_calibData.m_digP5)) << 17);
  var2 = var2 + (static_cast<int64_t>(m_calibData.m_digP4) << 35);
  var1 = ((var1 * var1 * static_cast<int64_t>(m_calibData.m_digP3)) >> 8) +
         ((var1 * static_cast<int64_t>(m_calibData.m_digP2)) << 12);
  var1 = ((static_cast<int64_t>(1) << 47) + var1) *
             static_cast<int64_t>(m_calibData.m_digP1) >>
         33;

  if (var1 == 0) {
    return 0.0F;
  }

  pressure = static_cast<int64_t>(1048576 - adc_P);
  pressure = (((pressure << 31) - var2) * static_cast<int64_t>(3125)) / var1;
  var1 = (static_cast<int64_t>(m_calibData.m_digP9) * (pressure >> 13) *
          (pressure >> 13)) >>
         25;
  var2 = (static_cast<int64_t>(m_calibData.m_digP8) * pressure) >> 19;
  pressure = ((pressure + var1 + var2) >> 8) +
             (static_cast<int64_t>(m_calibData.m_digP7) << 4);

  return static_cast<float>(pressure) / kCompensation256;
}

/**
 * @brief Compensates raw humidity data
 * @param adc_H Raw humidity data
 * @return Compensated humidity in %RH
 */
float BME280::compensate_humidity(int32_t adc_H) {
  int32_t var1 = 0;

  var1 = m_tFine - static_cast<int32_t>(kCompensation76800);
  var1 = (((((adc_H << 14) - (static_cast<int32_t>(m_calibData.m_digH4) << 20) -
             (static_cast<int32_t>(m_calibData.m_digH5) * var1)) +
            static_cast<int32_t>(kCompensation16384)) >>
           15) *
          (((((((var1 * static_cast<int32_t>(m_calibData.m_digH6)) >> 10) *
               (((var1 * static_cast<int32_t>(m_calibData.m_digH3)) >> 11) +
                static_cast<int32_t>(kCompensation32768))) >>
              10) +
             static_cast<int32_t>(kCompensation2097152)) *
                static_cast<int32_t>(m_calibData.m_digH2) +
            static_cast<int32_t>(kCompensation512)) >>
           14));
  var1 = var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) *
                  static_cast<int32_t>(m_calibData.m_digH1)) >>
                 4);
  var1 = var1 < 0 ? 0 : var1;
  var1 = var1 > static_cast<int32_t>(kCompensation419430400)
             ? static_cast<int32_t>(kCompensation419430400)
             : var1;

  return static_cast<float>(var1 >> 12) / kCompensation1024;
}

/**
 * @brief Calculates the measurement time based on oversampling settings
 * @param osrs_t Temperature oversampling
 * @param osrs_p Pressure oversampling
 * @param osrs_h Humidity oversampling
 * @return Measurement time in milliseconds
 */
float BME280::calculate_measurement_time() const {
  // Formula from datasheet section 9.1:
  // t_ms = 1.25 + 2.3·osr_t + 2.3·osr_p + 0.575 + 2.4·osr_h + 0.575
  float t_ms = 1.25f;  // Base time

  // Temperature measurement time
  t_ms += 2.3f * static_cast<float>(m_settings.m_tempOversampling);

  // Pressure measurement time
  t_ms += 2.3f * static_cast<float>(m_settings.m_pressOversampling) + 0.575f;

  // Humidity measurement time
  t_ms += 2.4f * static_cast<float>(m_settings.m_humOversampling) + 0.575f;

  return t_ms;
}

void BME280::logData(const sober::logger::LogEntry& rec) {
  if (queue_) {
    const std::size_t HIGH_WATER = queue_->capacity() * 8 / 10;
    std::size_t qsz = queue_->size();
    std::size_t qcap = queue_->capacity();

    bool pushed = queue_->try_emplace(rec);

    if (!pushed) {
      SPDLOG_WARN("[TEMPERATURE] queue overflow, dropping sample (size={}/{})",
                  qsz, qcap);
      SPDLOG_DEBUG("[TEMPERATURE] notifying logger due to overflow");
      sober::logger::Logger::instance().notify();
    } else if (qsz >= HIGH_WATER) {
      SPDLOG_DEBUG("[TEMPERATURE] high-water mark reached: {}/{} ({}%)", qsz,
                   qcap, (qsz * 100) / qcap);
      sober::logger::Logger::instance().notify();
    }
  }
}

/**
 * @brief Main sensor thread function
 * Reads sensor data and publishes events
 */
void BME280::run(std::stop_token stoken) {
#ifndef DEBUG_BME
  if (!init_i2c()) {
    SPDLOG_CRITICAL("[TEMPERATURE] Failed to initialize BME280");
    return;
  }
#endif

  auto next_wakeup = std::chrono::steady_clock::now() + sample_period;

  while (!stoken.stop_requested()) {
#ifndef DEBUG_BME
    if (!set_mode(Mode::FORCED)) {
      SPDLOG_ERROR("[TEMPERATURE] Failed to trigger BME280 measurements");
      std::this_thread::sleep_for(std::chrono::milliseconds(TRIGGER_GRACE));
      continue;
    }

    const uint32_t convTime =
        static_cast<uint32_t>(calculate_measurement_time()) + 1;
    std::this_thread::sleep_for(std::chrono::milliseconds(convTime));

    // Read raw data
    std::array<uint8_t, 8> data{};
    if (!read_registers(kRegPressMsb, data.data(), data.size())) {
      SPDLOG_ERROR("[TEMPERATURE] Failed to read sensor data");
      // Attempt to recover from I2C error
      if (!init_i2c()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
    }

    // Convert raw data to actual values
    const int32_t adcPress = (data[0] << kBitShift12) |
                             (data[1] << kBitShift4) | (data[2] >> kBitShift4);
    const int32_t adcTemp = (data[3] << kBitShift12) | (data[4] << kBitShift4) |
                            (data[5] >> kBitShift4);
    const int32_t adcHum = (data[6] << kBitShift8) | data[7];

    float Temp = compensate_temperature(adcTemp);
    float Press = compensate_pressure(adcPress);
    float Hum = compensate_humidity(adcHum);
#else
    static std::random_device randDist;
    static std::mt19937 rng(randDist());
    static std::normal_distribution<float> dist(20.0f, 5.5f);
    float Temp = dist(rng);
    float Press = dist(rng);
    float Hum = dist(rng);
#endif

    uint64_t generator = gSnapshot.generation.load(std::memory_order_relaxed);
    gSnapshot.generation.store(generator + 1, std::memory_order_release);

    gSnapshot.temp = Temp;
    gSnapshot.press = Press;
    gSnapshot.hum = Hum;

    gSnapshot.generation.store(generator + 2, std::memory_order_release);

    SPDLOG_INFO("[TEMPERATURE] Temperature: {} Pressure: {} Humidity: {}", Temp,
                Press, Hum);

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
        static_cast<uint8_t>(sizeof(Temp) + sizeof(Press) + sizeof(Hum));

    std::memcpy(rec.data.data(), &Temp, sizeof(Temp));
    std::memcpy(rec.data.data() + sizeof(Temp), &Press, sizeof(Press));
    std::memcpy(rec.data.data() + sizeof(Temp) + sizeof(Press), &Hum,
                sizeof(Hum));

    logData(rec);

    std::unique_lock<std::mutex> lk(cvMutex_);
    cv_.wait_until(lk, next_wakeup, [&] { return stoken.stop_requested(); });
    next_wakeup += sample_period;
  }
}

/**
 * @brief Calculates the sleep time between measurements
 * @return Sleep time in milliseconds
 */
uint32_t BME280::calculate_sleep_time() const {
  uint32_t tempTime = 0;
  uint32_t pressTime = 0;
  uint32_t humTime = 0;

  // Temperature measurement time
  switch (m_settings.m_tempOversampling) {
    case Oversampling::SKIP:
      tempTime = 0;
      break;
    case Oversampling::X1:
      tempTime = kMeasureTimeX1;
      break;
    case Oversampling::X2:
      tempTime = kMeasureTimeX2;
      break;
    case Oversampling::X4:
      tempTime = kMeasureTimeX4;
      break;
    case Oversampling::X8:
      tempTime = kMeasureTimeX8;
      break;
    case Oversampling::X16:
      tempTime = kMeasureTimeX16;
      break;
    default:
      break;
  }

  // Pressure measurement time
  switch (m_settings.m_pressOversampling) {
    case Oversampling::SKIP:
      pressTime = 0;
      break;
    case Oversampling::X1:
      pressTime = kMeasureTimeX1;
      break;
    case Oversampling::X2:
      pressTime = kMeasureTimeX2;
      break;
    case Oversampling::X4:
      pressTime = kMeasureTimeX4;
      break;
    case Oversampling::X8:
      pressTime = kMeasureTimeX8;
      break;
    case Oversampling::X16:
      pressTime = kMeasureTimeX16;
      break;
    default:
      break;
  }

  // Humidity measurement time
  switch (m_settings.m_humOversampling) {
    case Oversampling::SKIP:
      humTime = 0;
      break;
    case Oversampling::X1:
      humTime = kMeasureTimeX1;
      break;
    case Oversampling::X2:
      humTime = kMeasureTimeX2;
      break;
    case Oversampling::X4:
      humTime = kMeasureTimeX4;
      break;
    case Oversampling::X8:
      humTime = kMeasureTimeX8;
      break;
    case Oversampling::X16:
      humTime = kMeasureTimeX16;
      break;
    default:
      break;
  }

  // Add 1ms for safety margin
  return tempTime + pressTime + humTime + 1;
}

/**
 * @brief Reads all sensor data
 * @return true if successful, false otherwise
 */
bool BME280::read_sensor_data() {
  uint8_t data[8];
  if (!read_registers(kRegPressMsb, data, 8)) {
    SPDLOG_ERROR("[TEMPERATURE] Failed to read sensor data");
    return false;
  }

  // Convert raw data to actual values
  const int32_t adcPress = (data[0] << kBitShift12) | (data[1] << kBitShift4) |
                           (data[2] >> kBitShift4);
  const int32_t adcTemp = (data[3] << kBitShift12) | (data[4] << kBitShift4) |
                          (data[5] >> kBitShift4);
  const int32_t adcHum = (data[6] << kBitShift8) | data[7];

  // Compensate raw data
  m_readings.m_temperature = compensate_temperature(adcTemp);
  m_readings.m_pressure = compensate_pressure(adcPress);
  m_readings.m_humidity = compensate_humidity(adcHum);

  return true;
}

}  // namespace sober::housekeeping
