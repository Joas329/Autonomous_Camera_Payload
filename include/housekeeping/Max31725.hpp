#pragma once

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <stop_token>
#include <string>
#include <thread>

#include "eventbus/EventBus.hpp"
#include "logger/Logger.hpp"

/**
 * @file MAX31725.h
 * @brief Header file for the MAX31725 temperature sensor driver.
 *
 * This header file contains the I2C addresses for multiple MAX31725 sensors
 * and function prototypes to initialize the sensors, read temperature data, and configure the sensors.
 */
//#define DEBUG_MAX31725

namespace sober::housekeeping {
#define REG_TEMP_MSB 0x00
#define REG_CONFIG 0x01
#define REG_THYST 0x02
#define REG_TOS 0x03
/** Default sample period for reading sensor (adjust as needed) */
static constexpr auto sample_period = std::chrono::milliseconds(1000);

/**
 * @class MAX31725
 * @brief Linux i2c-dev driver + threaded sampling for MAX31725 temperature sensor.
 *
 * This class implements:
 *  - I2C initialization using /dev/i2c-X
 *  - register read/write helpers
 *  - start/stop/run thread model similar to your BME280 driver
 *  - high-level register operations: read temperature, read THYST/TOS, write THYST/TOS, get/set config
 */
class Max31725 {
 public:
  /**
    * @brief Construct a MAX31725 driver instance.
    * @param bus Reference to event bus (used to publish events)  // --- may be optional
    * @param i2cBus I2C bus number (e.g. 1 for /dev/i2c-1)
    * @param i2cAddr 7-bit I2C address of the sensor
    * @param queue Optional logger queue pointer (same type used by BME280)
    */
  Max31725(sober::eventbus::EventBus& bus, uint8_t i2cBus, uint8_t i2cAddr,
           std::shared_ptr<sober::logger::Logger::Queue> queue,
           uint8_t one_shot, uint8_t timeout_enable,
           uint8_t extendedDataFormat_enable, int faults_queue,
           uint8_t osPolarity_enable, uint8_t interrupt_enable);

  /** @brief Destructor: stops thread and closes I2C */
  ~Max31725();

  /** @brief Start background sampling thread. */
  bool start();

  /** @brief Stop background sampling thread. */
  void stop();

  /**
    * @brief Main sensor thread.
    * @param stoken stop_token used to request thread stop
    */
  void run(std::stop_token stoken);

  /** @brief Initialize I2C device (open bus and verify device) */
  bool init_i2c();

  /** @brief Read temperature register (returns value in 1/256 �C units like original) */
  int32_t readTemperature();

  /** @brief Read THYST register (1/256 �C units) */
  int32_t readTHYST();

  /** @brief Read TOS register (1/256 �C units) */
  int32_t readTOS();

  /** @brief Write THYST register (msb/lsb) */
  bool writeTHYST(uint8_t msb, uint8_t lsb);

  /** @brief Write TOS register (msb/lsb) */
  bool writeTOS(uint8_t msb, uint8_t lsb);

  /** @brief Read config byte */
  uint8_t getConfig();

  /** @brief Write config byte */
  bool writeConfig(uint8_t data);

  void setTempIdx(uint8_t index) { tempIdx = index; }

 private:
  uint8_t tempIdx{0};
  /**
    * @brief Read/Write helpers using linux i2c-dev
    * readRegisters: writes register pointer then reads `len` bytes into buffer
    * writeRegisters: writes register followed by data bytes
    */
  bool readRegisters(uint8_t reg, uint8_t* buffer, uint8_t len);
  bool writeRegisters(const uint8_t* buffer, uint8_t len);
  void updateConfigBit(uint8_t bit, bool enable);
  void MAX31725_SetConfig_OneShot(uint8_t active);
  void MAX31725_SetConfig_Timeout(uint8_t enabled);
  void MAX31725_SetConfig_ExtendedDataFormat(uint8_t enabled);
  void MAX31725_SetConfig_FaultsQueue(int faults);
  void MAX31725_SetConfig_OSPolarity(uint8_t active_high);
  void MAX31725_SetConfig_InterruptMode(uint8_t interrupt_active);
  void MAX31725_SetConfig_ShutdownMode(uint8_t shutdown_active);

  sober::eventbus::EventBus& m_bus_;  ///< reference to eventbus (external)
  int m_nFd;                          ///< file descriptor for /dev/i2c-X
  uint8_t m_nI2cBus;                  ///< bus number (for path formation)
  uint8_t m_nI2cAddr;                 ///< 7-bit I2C address
  std::shared_ptr<sober::logger::Logger::Queue>
      m_pQueue;  ///< optional logger queue

  std::jthread m_sensorThread_;
  std::mutex cvMutex_;
  std::condition_variable cv_;
};
}  // namespace sober::housekeeping