#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <thread>

#include <linux/spi/spidev.h>

#include "eventbus/EventBus.hpp"
#include "logger/Logger.hpp"

//#define DEBUG_THERMO

enum class Chip { MAX31855, MAX31865 };
inline constexpr Chip kChip = Chip::MAX31855;  // or Chip::MAX31865
inline constexpr uint32_t Mode =
    (kChip == Chip::MAX31855) ? SPI_MODE_0 : SPI_MODE_1;
inline constexpr uint32_t Speed = 2000;
inline constexpr uint8_t Bits = 8;

inline constexpr uint8_t Thermocouple_Reg = 0x80;
inline constexpr double Rref = 400.0;
namespace sober::housekeeping {
class Thermocouple {
 public:
  /**
     * @param bus   Event bus to publish readings/faults
     * @param queue Logger queue (can be nullptr if you don’t want logs)
     * @param device_path SPI device node, e.g. "/dev/spidev0.0"
     */
  struct Reading {
    double thermocouple_c;  // external thermocouple temperature (°C)
    double internal_c;      // internal/reference junction (°C)
    bool fault;             // any fault present
    bool scv;               // short to VCC
    bool scg;               // short to GND
    bool oc;                // open circuit
  };

  Thermocouple(std::shared_ptr<sober::logger::Logger::Queue> queue,
               std::string device_path);

  ~Thermocouple();

  void run(std::stop_token stoken);

  bool start();
  bool spi_set(int fd, unsigned long req, uint32_t value);
  void stop();
  void closeDevice();

 private:
  bool writeReg(int fd, uint8_t reg, uint8_t value);
  bool readRegs(int fd, uint8_t startReg, uint8_t* buf, size_t len);
  double TempConversion(uint16_t rtdCode, double rRef);
  bool readPT100(Reading& values);
  bool readThermocouple(Reading& values);

  bool readOnce(Reading& out);

  const std::string& devicePath() const { return dev_; }

  void logData(const sober::logger::LogEntry& rec);

  std::jthread sensorThread;   ///< Thread for continuous measurements
  std::mutex m_readingsMutex;  ///< (kept in case you cache readings later)
  std::shared_ptr<sober::logger::Logger::Queue> queue_;

  std::mutex cvMutex_;
  std::condition_variable cv_;

  int fd_;           // POSIX file descriptor
  std::string dev_;  // e.g. "/dev/spidev0.0"
  static constexpr auto sample_period = std::chrono::milliseconds(100);
};

}  // namespace sober::housekeeping
