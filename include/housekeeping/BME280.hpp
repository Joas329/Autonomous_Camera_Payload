#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

#include "eventbus/EventBus.hpp"
#include "logger/Logger.hpp"

// #define DEBUG_BME

namespace sober::housekeeping {

/******************************************************************************/
/*! @name        Constants                                                    */
/******************************************************************************/
inline constexpr uint8_t kChipId = 0x60;
inline constexpr uint8_t kSoftReset = 0xB6;

inline constexpr int64_t TRIGGER_GRACE = 10;

// Register addresses
inline constexpr uint8_t kRegId = 0xD0;
inline constexpr uint8_t kRegReset = 0xE0;
inline constexpr uint8_t kRegCtrlHum = 0xF2;
inline constexpr uint8_t kRegStatus = 0xF3;
inline constexpr uint8_t kRegCtrlMeas = 0xF4;
inline constexpr uint8_t kRegConfig = 0xF5;
inline constexpr uint8_t kRegPressMsb = 0xF7;
inline constexpr uint8_t kRegPressLsb = 0xF8;
inline constexpr uint8_t kRegPressXlsb = 0xF9;
inline constexpr uint8_t kRegTempMsb = 0xFA;
inline constexpr uint8_t kRegTempLsb = 0xFB;
inline constexpr uint8_t kRegTempXlsb = 0xFC;
inline constexpr uint8_t kRegHumMsb = 0xFD;
inline constexpr uint8_t kRegHumLsb = 0xFE;

// Calibration data registers
inline constexpr uint8_t kRegDigT1 = 0x88;
inline constexpr uint8_t kRegDigT2 = 0x8A;
inline constexpr uint8_t kRegDigT3 = 0x8C;
inline constexpr uint8_t kRegDigP1 = 0x8E;
inline constexpr uint8_t kRegDigP2 = 0x90;
inline constexpr uint8_t kRegDigP3 = 0x92;
inline constexpr uint8_t kRegDigP4 = 0x94;
inline constexpr uint8_t kRegDigP5 = 0x96;
inline constexpr uint8_t kRegDigP6 = 0x98;
inline constexpr uint8_t kRegDigP7 = 0x9A;
inline constexpr uint8_t kRegDigP8 = 0x9C;
inline constexpr uint8_t kRegDigP9 = 0x9E;
inline constexpr uint8_t kRegDigH1 = 0xA1;
inline constexpr uint8_t kRegDigH2 = 0xE1;
inline constexpr uint8_t kRegDigH3 = 0xE3;
inline constexpr uint8_t kRegDigH4 = 0xE4;
inline constexpr uint8_t kRegDigH5 = 0xE5;
inline constexpr uint8_t kRegDigH6 = 0xE7;

// Bit operations
inline constexpr uint8_t kBitShift4 = 4;
inline constexpr uint8_t kBitShift8 = 8;
inline constexpr uint8_t kBitShift12 = 12;

// Compensation constants
inline constexpr int32_t kCompensation128000 = 128000;
inline constexpr int32_t kCompensation32768 = 32768;
inline constexpr int32_t kCompensation16384 = 16384;
inline constexpr int32_t kCompensation2097152 = 2097152;
inline constexpr int32_t kCompensation512 = 512;
inline constexpr int32_t kCompensation419430400 = 419430400;
inline constexpr int32_t kCompensation76800 = 76800;
inline constexpr float kCompensation100 = 100.0F;
inline constexpr float kCompensation256 = 256.0F;
inline constexpr float kCompensation1024 = 1024.0F;

// Measurement timing (ms) - from datasheet section 9.1
inline constexpr uint32_t kMeasureTimeX1 = 1;
inline constexpr uint32_t kMeasureTimeX2 = 2;
inline constexpr uint32_t kMeasureTimeX4 = 3;
inline constexpr uint32_t kMeasureTimeX8 = 4;
inline constexpr uint32_t kMeasureTimeX16 = 5;

/******************************************************************************/
/*! @name        Enums                                                        */
/******************************************************************************/
enum class Oversampling : uint8_t {
  SKIP = 0x00,
  X1 = 0x01,
  X2 = 0x02,
  X4 = 0x03,
  X8 = 0x04,
  X16 = 0x05
};

enum class Mode : uint8_t { SLEEP = 0x00, FORCED = 0x01, NORMAL = 0x03 };

enum class Filter : uint8_t {
  OFF = 0x00,
  X2 = 0x01,
  X4 = 0x02,
  X8 = 0x03,
  X16 = 0x04
};

enum class StandbyTime : uint8_t {
  MS_0_5 = 0x00,
  MS_62_5 = 0x01,
  MS_125 = 0x02,
  MS_250 = 0x03,
  MS_500 = 0x04,
  MS_1000 = 0x05,
  MS_10 = 0x06,
  MS_20 = 0x07
};

/******************************************************************************/
/*! @name        Structures                                                   */
/******************************************************************************/
struct CalibrationData {
  uint16_t m_digT1;
  int16_t m_digT2;
  int16_t m_digT3;
  uint16_t m_digP1;
  int16_t m_digP2;
  int16_t m_digP3;
  int16_t m_digP4;
  int16_t m_digP5;
  int16_t m_digP6;
  int16_t m_digP7;
  int16_t m_digP8;
  int16_t m_digP9;
  uint8_t m_digH1;
  int16_t m_digH2;
  uint8_t m_digH3;
  int16_t m_digH4;
  int16_t m_digH5;
  int8_t m_digH6;
};

struct SensorSettings {
  Oversampling m_tempOversampling;
  Oversampling m_pressOversampling;
  Oversampling m_humOversampling;
  Mode m_mode;
  Filter m_filter;
  StandbyTime m_standbyTime;
  uint32_t m_measurementIntervalMs;
};

struct SensorReadings {
  float m_temperature;
  float m_pressure;
  float m_humidity;
};

/******************************************************************************/
/*! @name        Class Definition                                             */
/******************************************************************************/
/**
 * @brief BME280 environmental sensor driver
 * Provides temperature, pressure, and humidity measurements
 */
class BME280 {
 public:
  /**
     * @brief Constructor
     * @param bus Reference to the event bus for publishing sensor data
     * @param i2c_bus I2C bus number
     * @param i2c_addr I2C device address
     */
  BME280(sober::eventbus::EventBus& bus, uint8_t i2c_bus, uint8_t i2c_addr,
         std::shared_ptr<sober::logger::Logger::Queue> queue);

  /**
     * @brief Destructor
     * Ensures proper cleanup of resources
     */
  ~BME280();

  /**
     * @brief Main sensor thread function
     * Reads sensor data and publishes events
     */
  void run(std::stop_token stoken);

  /**
     * @brief Starts the BME280 sensor
     */
  bool start();

  /**
     * @brief Stops the BME280 sensor
     */
  void stop();

  /**
     * @brief Reads the current temperature from the sensor
     * @return Temperature in degrees Celsius
     */
  float read_temperature();

  /**
     * @brief Reads the current pressure from the sensor
     * @return Pressure in Pa
     */
  float read_pressure();

  /**
     * @brief Reads the current humidity from the sensor
     * @return Relative humidity in percent
     */
  float read_humidity();

  /**
     * @brief Sets the operating mode of the sensor
     * @param mode The desired operating mode
     * @return true if successful, false otherwise
     */
  bool set_mode(Mode mode);

  /**
     * @brief Sets the oversampling rates for temperature, pressure, and humidity
     * @param temp Temperature oversampling rate
     * @param press Pressure oversampling rate
     * @param hum Humidity oversampling rate
     * @return true if successful, false otherwise
     */
  bool set_oversampling(Oversampling temp, Oversampling press,
                        Oversampling hum);

  /**
     * @brief Sets the filter coefficient for the sensor
     * @param filter The desired filter coefficient
     * @return true if successful, false otherwise
     */
  bool set_filter(Filter filter);

  /**
     * @brief Sets the standby time for the sensor
     * @param time The desired standby time
     * @return true if successful, false otherwise
     */
  bool set_standby_time(StandbyTime time);

 private:
  /**
     * @brief Initializes the I2C communication with the sensor
     * @return true if successful, false otherwise
     */
  bool init_i2c();

  /**
     * @brief Reads calibration data from the sensor
     * @return true if successful, false otherwise
     */
  bool read_calibration_data();

  /**
     * @brief Writes a value to a sensor register
     * @param reg Register address
     * @param value Value to write
     * @return true if successful, false otherwise
     */
  bool write_register(uint8_t reg, uint8_t value);

  /**
     * @brief Reads multiple registers from the sensor
     * @param reg Starting register address
     * @param buffer Buffer to store read data
     * @param length Number of bytes to read
     * @return true if successful, false otherwise
     */
  bool read_registers(uint8_t reg, uint8_t* buffer, uint8_t length);

  /**
     * @brief Reads a single register from the sensor
     * @param reg Register address
     * @return Register value, or 0 if read failed
     */
  uint8_t read_register(uint8_t reg);

  /**
     * @brief Compensates raw temperature data
     * @param adc_T Raw temperature data
     * @return Compensated temperature in degrees Celsius
     */
  float compensate_temperature(int32_t adc_T);

  /**
     * @brief Compensates raw pressure data
     * @param adc_P Raw pressure data
     * @return Compensated pressure in Pa
     */
  float compensate_pressure(int32_t adc_P);

  /**
     * @brief Compensates raw humidity data
     * @param adc_H Raw humidity data
     * @return Compensated humidity in %RH
     */
  float compensate_humidity(int32_t adc_H);

  /**
     * @brief Calculates the sleep time between measurements
     * @return Sleep time in milliseconds
     */
  uint32_t calculate_sleep_time() const;

  /**
     * @brief Reads all sensor data
     * @return true if successful, false otherwise
     */
  bool read_sensor_data();

  /**
     * @brief Calculates the measurement time based on oversampling settings
     * @return Measurement time in milliseconds
     */
  float calculate_measurement_time() const;

  void logData(const sober::logger::LogEntry& rec);

  sober::eventbus::EventBus& m_bus;  ///< Reference to the event bus
  int m_i2cFd;                       ///< I2C file descriptor
  uint8_t m_i2cBus;                  ///< I2C bus number
  uint8_t m_i2cAddr;                 ///< I2C device address
  int32_t m_tFine;                   ///< Fine resolution temperature value
  CalibrationData m_calibData;       ///< Calibration data
  SensorSettings m_settings;         ///< Sensor settings
  SensorReadings m_readings;         ///< Current sensor readings
  std::jthread m_sensorThread;       ///< Thread for continuous measurements
  std::mutex m_readingsMutex;        // Protect m_readings access
  std::shared_ptr<sober::logger::Logger::Queue> queue_;

  std::mutex cvMutex_;
  std::condition_variable cv_;

  static constexpr auto sample_period = std::chrono::milliseconds(1000);
};

}  // namespace sober::housekeeping
