#pragma once

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "eventbus/EventBus.hpp"
#include "logger/Logger.hpp"

namespace sober::housekeeping {

//#define DEBUG_INA

// --- Register addresses ---
#define INA237_REG_CONFIG 0x00
#define INA237_REG_ADC_CONFIG 0x01
#define INA237_REG_SHUNT_CAL 0x02
#define INA237_REG_VSHUNT 0x04
#define INA237_REG_VBUS 0x05
#define INA237_REG_DIETEMP 0x06
#define INA237_REG_CURRENT 0x07
#define INA237_REG_POWER 0x08
#define INA237_REG_DIAG_ALRT 0x0B
#define INA237_REG_SOVL 0x0C
#define INA237_REG_SUVL 0x0D
#define INA237_REG_BOVL 0x0E
#define INA237_REG_BUVL 0x0F
#define INA237_REG_TEMP_LIMIT 0x10
#define INA237_REG_PWR_LIMIT 0x11
#define INA237_REG_MANUFACTURER_ID 0x3E

// --- ADC range ---
#define INA237_ADC_RANGE_163_84mV 0x0
#define INA237_ADC_RANGE_40_96mV 0x1

// Resolution constants
const float INA237_VSHUNT_LSB_RES[2] = {static_cast<float>(5E-6),
                                        static_cast<float>(1.25E-6)};
const float INA237_VBUS_LSB_RES = static_cast<float>(3.125E-3);
const float INA237_TEMP_LSB_RES = static_cast<float>(125E-3);

/*----------ADC CONFIG DEFINES------------------------------------------------*/
/* define operating modes*/
#define INA237_MODE_SHUTDOWN_1 0x0
#define INA237_MODE_TRIG_VBUS 0x1
#define INA237_MODE_TRIG_VSHUNT 0x2
#define INA237_MODE_TRIG_VBUS_VSHUNT 0x3
#define INA237_MODE_TRIG_TEMP 0x4
#define INA237_MODE_TRIG_TEMP_VBUS 0x5
#define INA237_MODE_TRIG_TEMP_VSHUNT 0x6
#define INA237_MODE_TRIG_TEMP_VBUS_VSHUNT 0x7
#define INA237_MODE_SHUTDOWN_2 0x8
#define INA237_MODE_CONT_VBUS 0x9
#define INA237_MODE_CONT_VSHUNT 0xA
#define INA237_MODE_CONT_VBUS_VSHUNT 0xB
#define INA237_MODE_CONT_TEMP 0xC
#define INA237_MODE_CONT_TEMP_VBUS 0xD
#define INA237_MODE_CONT_TEMP_VSHUNT 0xE
#define INA237_MODE_CONT_TEMP_VBUS_VSHUNT 0xF

/* define conversion times for VBUS, VSHUNT and TEMP */
#define INA237_CT_50US 0x0
#define INA237_CT_84US 0x1
#define INA237_CT_150US 0x2
#define INA237_CT_280US 0x3
#define INA237_CT_540US 0x4
#define INA237_CT_1052US 0x5
#define INA237_CT_2074US 0x6
#define INA237_CT_4120US 0x7

/* define AVG counts */
// defines for avg samples
#define INA237_AVG_1 0x0
#define INA237_AVG_4 0x1
#define INA237_AVG_16 0x2
#define INA237_AVG_64 0x3
#define INA237_AVG_128 0x4
#define INA237_AVG_256 0x5
#define INA237_AVG_512 0x6
#define INA237_AVG_1024 0x7

class Ina237 {
 public:
  Ina237(sober::eventbus::EventBus& bus, uint8_t i2cBus, uint8_t i2cAddr,
         std::shared_ptr<sober::logger::Logger::Queue> queue,
         uint8_t voltageIndex);
  ~Ina237();

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

  void reset();
  float getCurrent();
  float getBusVoltage();
  float getShuntVoltage();
  float getTemperature();

 private:
  uint8_t voltageIndex_{};
  void logData(const sober::logger::LogEntry& rec);

  /**
		* @brief Initializes the I2C communication with the sensor
		* @return true if successful, false otherwise
		*/
  bool init_i2c();

  bool writeRegister(uint8_t reg, uint16_t data);
  /**
		* @brief Read one or more bytes from a register.
		*
		* @param reg Register address to read from
		* @param buffer Pointer to destination buffer
		* @param length Number of bytes to read (1�3 for INA237)
		* @return true if the operation succeeded, false otherwise
		*/
  bool readRegisters(uint8_t reg, uint8_t* buffer, uint8_t len);

  sober::eventbus::EventBus& m_bus;  ///< Reference to the event bus

  int m_nFd;            // file descriptor for I2C bus
  uint8_t m_nI2cBus;    ///< I2C bus number
  uint8_t m_nI2cAddr;   // I2C device address
  uint8_t m_nAdcRange;  // ADC range config
  float m_dShuntRes;    // Shunt resistor value in ohms
  float m_dCurrentLSB;  // Current LSB scaling factor

  std::jthread m_sensorThread;  ///< Thread for continuous measurements
  std::shared_ptr<sober::logger::Logger::Queue> queue_;

  std::mutex m_readingsMutex;  // Protect m_readings
  std::mutex cvMutex_;
  std::condition_variable cv_;

  static constexpr auto m_nSamplePeriod = std::chrono::milliseconds(1000);
};
}  // namespace sober::housekeeping