#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <thread>

#include "eventbus/EventBus.hpp"
#include "logger/Logger.hpp"

// #define DEBUG_MAG

namespace sober::housekeeping {

/******************************************************************************/
/*! @name        Constants                                                    */
/******************************************************************************/

/************** I2C Address *****************/
inline constexpr uint8_t MAG_I2C_AddressL = 0x38;
inline constexpr uint8_t MAG_I2C_AddressH = 0x3C;

/************** Who am I  *******************/

inline constexpr uint8_t MAG_CHIPID = 0x3D;

/************** Configuration Registers  *******************/

inline constexpr uint8_t MAG_ID_REG = 0x0F;
inline constexpr uint8_t MAG_CTRL_REG1 = 0x20;
inline constexpr uint8_t MAG_CTRL_REG2 = 0x21;
inline constexpr uint8_t MAG_CTRL_REG3 = 0x22;
inline constexpr uint8_t MAG_CTRL_REG4 = 0x23;
inline constexpr uint8_t MAG_CTRL_REG5 = 0x24;

/************** Bit Operations ****************** */
inline constexpr uint8_t BitShift1 = 1;
inline constexpr uint8_t BitShift2 = 2;
inline constexpr uint8_t BitShift5 = 5;
inline constexpr uint8_t BitShift7 = 7;
inline constexpr uint8_t BitShift6 = 6;
inline constexpr uint8_t BitShift8 = 8;

/************** Data Registers  *******************/
inline constexpr uint8_t MAG_STATUS_REG = 0x27;
inline constexpr uint8_t MAG_OUTX_LSB = 0x28;
inline constexpr uint8_t MAG_OUTX_MSB = 0x29;
inline constexpr uint8_t MAG_OUTY_LSB = 0x2A;
inline constexpr uint8_t MAG_OUTY_MSB = 0x2B;
inline constexpr uint8_t MAG_OUTZ_LSB = 0x2C;
inline constexpr uint8_t MAG_OUTZ_MSB = 0x2D;
inline constexpr uint8_t MAG_TEMP_LSB = 0x2E;
inline constexpr uint8_t MAG_TEMP_MSB = 0x2F;

/************** Intrerupt Registers  *******************/
inline constexpr uint8_t MAG_INT_CFG = 0x30;
inline constexpr uint8_t MAG_INT_SRC = 0x31;
inline constexpr uint8_t MAG_INT_THS_LSB = 0x32;
inline constexpr uint8_t MAG_INT_THS_MSB = 0x33;

/******************************************************************************/
/*! @name        Enums                                                        */
/*******/

enum class TempEn : uint8_t { OFF = 0x00, ON = 0x01 };

enum class ModeXY : uint8_t {
  LowPower = 0x00,
  MedPerf = 0x01,
  HighPerf = 0x02,
  UHighPerf = 0x03,
};

enum class ModeZ : uint8_t {
  LowPower = 0x00,
  MedPerf = 0x01,
  HighPerf = 0x02,
  UHighPerf = 0x03,
};

enum class OutputDataRate : uint8_t {
  Hz_0_625 = 0x00,
  Hz_1_25 = 0x01,
  Hz_2_5 = 0x02,
  Hz_5_0 = 0x03,
  Hz_10 = 0x04,
  Hz_20 = 0x05,
  Hz_40 = 0x06,
  Hz_80 = 0x07
  // Note: For fast output data rate (FAST_ODR), combine with MODE_XY and shift bits manually:
  // Example:
  // ModeXY mode = ModeXY::HighPerf;
  // uint8_t configByte = (static_cast<uint8_t>(mode) << 5) | (1 << 1);
};

enum class SelfTest : uint8_t { OFF = 0x00, ON = 0X01 };

enum class FullScale : uint8_t {
  Gauss_4 = 0x00,
  Gauss_8 = 0x01,
  Gauss_12 = 0x02,
  Gauss_16 = 0x03
};

/*  0: normal mode; 1: reboot memory content) */
enum class Reboot : uint8_t { Default = 0X00, Enable = 0X01 };

enum class SoftRst : uint8_t { Default = 0X00, Enable = 0x01 };

enum class CommunicationMode : uint8_t { Wire_4 = 0x00, Wire_3 = 0x01 };

enum class PowerMode : uint8_t { Normal = 0x00, LowPower = 0x01 };

enum class OperationMode : uint8_t {
  Continuous = 0x00,
  Single = 0x01,
  PowerDown1 = 0x02,
  PowerDown = 0x03
};

enum class BigLittleEndian : uint8_t { LSB = 0x00, MSB = 0x01 };

enum class FastRead : uint8_t { Disabled = 0x00, Enabled = 0x01 };

enum class BlocUpdateMagneticData : uint8_t {
  Continuous = 0x00,
  MSBLSB = 0x01
};

struct SensorReadings1 {
  std::array<float, 3> m_mag;
  float m_temp;
};

struct SensorSettings1 {
  OutputDataRate m_outputDataRate;
  PowerMode m_powerMode;
  FullScale m_fullScale;
  ModeXY m_modeXY;
  ModeZ m_modeZ;
  TempEn m_tempEnabled;
  SelfTest m_selfTest;
  Reboot m_reboot;
  SoftRst m_softReset;
  CommunicationMode m_commMode;
  OperationMode m_opMode;
  BigLittleEndian m_endian;
  FastRead m_fastRead;
  BlocUpdateMagneticData m_blockUpdate;
};

class LIS3MDLTR {
 public:
  LIS3MDLTR(sober::eventbus::EventBus& bus, uint8_t i2c_bus, uint8_t i2c_adr,
            std::shared_ptr<sober::logger::Logger::Queue> queue);

  ~LIS3MDLTR();

  void stop();

  bool start();

  bool apply_settings();

  bool initialize_sensor();

  std::array<float, 3> read_magnetic_data();

  float read_temp();

 private:
  void run(std::stop_token stoken);

  bool init_i2c();

  bool write_register(uint8_t reg, uint8_t value);

  uint8_t read_register(uint8_t reg);

  bool read_registers(uint8_t start_reg, uint8_t* buffer, size_t len);

  bool read_data();

  void logData(const sober::logger::LogEntry& rec);

  std::once_flag stopFlag_;

  sober::eventbus::EventBus& m_bus;  ///< Reference to the event bus
  int m_i2cFd;                       ///< I2C file descriptor
  uint8_t m_i2cBus;                  ///< I2C bus number
  uint8_t m_i2cAddr;                 ///< I2C device address
  int32_t m_tFine;                   ///< Fine resolution temperature value
  SensorSettings1 m_settings;        ///< Sensor settings
  SensorReadings1 m_readings;        ///< Current sensor readings
  std::jthread sensorThread_;        ///< Thread for continuous measurements
  std::mutex m_readingsMutex;        // Protect m_readings access

  std::mutex cvMutex_;
  std::condition_variable cv_;

  std::shared_ptr<sober::logger::Logger::Queue> queue_;

  static constexpr auto sample_period = std::chrono::milliseconds(50);
};

}  // namespace sober::housekeeping