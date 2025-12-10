#include "housekeeping/Thermocouple.hpp"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <spdlog/spdlog.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stop_token>
#include <thread>

#include "eventbus/EventBus.hpp"
#include "housekeeping/SensorSnapshot.hpp"
#include "logger/Logger.hpp"

namespace sober::housekeeping {
using sober::logger::LogEntry;

// -------- Thermocouple implementation --------
Thermocouple::Thermocouple(std::shared_ptr<sober::logger::Logger::Queue> queue,
                           std::string device_path_)
    : m_readingsMutex(),
      queue_(std::move(queue)),
      fd_(-1),
      dev_(device_path_) {}

Thermocouple::~Thermocouple() {
  closeDevice();
}

bool Thermocouple::spi_set(int fd, unsigned long req, uint32_t value) {
  return ioctl(fd, req, &value) == 0;
}
bool Thermocouple::start() {
#ifndef DEBUG_THERMO
  stop();
  fd_ = ::open(dev_.c_str(), O_RDWR);
  if (fd_ < 0) {
    std::perror("open");
    SPDLOG_ERROR("[TC] open({}): {}", dev_, std::strerror(errno));
    return false;
  }

  if (Mode == SPI_MODE_1) {
    if (!spi_set(fd_, SPI_IOC_WR_MODE32, Mode) ||
        !spi_set(fd_, SPI_IOC_RD_MODE32, Mode)) {
      SPDLOG_ERROR("[TC] SPI_IOC_*_MODE32 failed: {}", std::strerror(errno));
      closeDevice();
      return false;
    }
    if (!spi_set(fd_, SPI_IOC_WR_BITS_PER_WORD, Bits) ||
        !spi_set(fd_, SPI_IOC_RD_BITS_PER_WORD, Bits)) {
      SPDLOG_ERROR("[TC] SPI_IOC_*_BITS_PER_WORD failed: {}",
                   std::strerror(errno));
      closeDevice();
      return false;
    }
    if (!spi_set(fd_, SPI_IOC_WR_MAX_SPEED_HZ, Speed) ||
        !spi_set(fd_, SPI_IOC_RD_MAX_SPEED_HZ, Speed)) {
      SPDLOG_ERROR("[TC] SPI_IOC_*_SPEED failed: {}", std::strerror(errno));
      closeDevice();
      return false;
    }
  } else {
    if (!spi_set(fd_, SPI_IOC_RD_MODE32, Mode)) {
      SPDLOG_ERROR("[TC] SPI_IOC_*_MODE32 failed: {}", std::strerror(errno));
      closeDevice();
      return false;
    }
    if (!spi_set(fd_, SPI_IOC_RD_BITS_PER_WORD, Bits)) {
      SPDLOG_ERROR("[TC] SPI_IOC_*_BITS_PER_WORD failed: {}",
                   std::strerror(errno));
      closeDevice();
      return false;
    }
    if (!spi_set(fd_, SPI_IOC_RD_MAX_SPEED_HZ, Speed)) {
      SPDLOG_ERROR("[TC] SPI_IOC_*_SPEED failed: {}", std::strerror(errno));
      closeDevice();
      return false;
    }
  }
#endif
  // SPDLOG_INFO("[TC] Opened SPI device {}", dev_);
  sensorThread = std::jthread([this](std::stop_token st) { this->run(st); });

  return true;
}

bool Thermocouple::writeReg(int fd, uint8_t reg, uint8_t value) {
  uint8_t tx[2] = {static_cast<uint8_t>(Thermocouple_Reg | reg), value};
  spi_ioc_transfer tr{};
  tr.tx_buf = reinterpret_cast<__u64>(tx);
  tr.rx_buf = 0;
  tr.len = sizeof(tx);
  return ::ioctl(fd, SPI_IOC_MESSAGE(1), &tr) >= 0;
}

bool Thermocouple::readRegs(int fd, uint8_t startReg, uint8_t* buf,
                            size_t len) {
  std::vector<uint8_t> tx(len + 1, 0);
  tx[0] = startReg;

  spi_ioc_transfer tr{};
  tr.tx_buf = reinterpret_cast<__u64>(tx.data());
  tr.rx_buf = reinterpret_cast<__u64>(tx.data());
  tr.len = tx.size();

  if (::ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0)
    return false;
  std::memcpy(buf, tx.data() + 1, len);
  return true;
}

double Thermocouple::TempConversion(uint16_t rtdCode, double rRef) {
  constexpr double R0 = 100.0;
  constexpr double A = 3.90830e-3;
  constexpr double B = -5.775e-7;
  const double rtdRes =
      (static_cast<double>(rtdCode) * rRef) / 32768.0;  // 15-bit code
  const double disc = A * A * R0 * R0 - 4 * B * R0 * (R0 - rtdRes);
  if (disc < 0)
    return std::numeric_limits<double>::quiet_NaN();
  return (-A * R0 + std::sqrt(disc)) / (2 * B * R0);
}

void Thermocouple::stop() {
  if (sensorThread.joinable()) {
    sensorThread.request_stop();
    cv_.notify_all();
    sensorThread.join();
  }
}

void Thermocouple::closeDevice() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
    // SPDLOG_INFO("[TC] Close SPI device {}", dev_);
  }
}

bool Thermocouple::readThermocouple(Reading& r) {
  uint8_t tx[4] = {0, 0, 0, 0};  // just clocks; device is read-only
  uint8_t rx[4] = {0};

  spi_ioc_transfer tr{};
  tr.tx_buf = reinterpret_cast<__u64>(tx);
  tr.rx_buf = reinterpret_cast<__u64>(rx);
  tr.len = 4;
  tr.speed_hz = Speed;      // MAX31855 SCK max 5 MHz
  tr.bits_per_word = Bits;  // 8
  tr.cs_change = 0;         // deassert CS after transfer

  if (::ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) < 1)
    return false;

  uint32_t raw = (uint32_t(rx[0]) << 24) | (uint32_t(rx[1]) << 16) |
                 (uint32_t(rx[2]) << 8) | uint32_t(rx[3]);
  // SPDLOG_INFO("[TC] Raw=0x{:08X}", raw);
  int32_t tc14 = int32_t((raw >> 18) & 0x3FFF);
  if (tc14 & 0x2000)
    tc14 |= ~0x3FFF;  // sign-extend 14-bit
  double rtdRes = static_cast<double>(tc14) * (-0.25);

  int32_t cj12 = static_cast<int32_t>((raw >> 4) & 0x0FFF);
  if (cj12 & 0x0800)
    cj12 |= ~0x0FFF;  // sign-extend 12-bit to 32-bit
  double rtdInt = static_cast<double>(cj12) * (-0.0625);  // °C

  r.thermocouple_c = rtdRes;  // reuse this field for the RTD temperature
  r.internal_c = rtdInt;      // not applicable for MAX31865; set to 0 or NaN
  r.fault = 0.0;
  r.scv = r.scg = r.oc = false;  // not mapped for MAX31865

  return true;
}

bool Thermocouple::readPT100(Reading& out) {
  uint8_t config = 0b10110010;  // 0xB2
  if (!writeReg(fd_, 0x00, config)) {
    SPDLOG_ERROR("[TC] write REG0 (config) failed: {}", std::strerror(errno));
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 100 ms

  uint8_t status = 0;
  if (!readRegs(fd_, 0x07, &status, 1)) {
    SPDLOG_ERROR("[TC] Failed to get the correct status: {}",
                 std::strerror(errno));
    return false;
  }

  std::array<uint8_t, 2> rtd = {};
  if (!readRegs(fd_, 0x01, rtd.data(), rtd.size())) {
    SPDLOG_ERROR("[TC] read RTD failed: {}", std::strerror(errno));
    return false;
  }
  uint16_t rtdCode = static_cast<uint16_t>((rtd[0] << 8) | rtd[1]);
  rtdCode >>= 1;

  //Convert to temperature (PT100, Rref=400Ω)
  double tempC = TempConversion(rtdCode, Rref);

  out.thermocouple_c = tempC;  // reuse this field for the RTD temperature
  out.internal_c = 0.0;        // not applicable for MAX31865; set to 0 or NaN
  out.fault = (status & Thermocouple_Reg) != 0;  // bit7
  out.scv = out.scg = out.oc = false;            // not mapped for MAX31865
  return true;
}

bool Thermocouple::readOnce(Reading& out) {
  if (fd_ < 0 && !start()) {
    SPDLOG_ERROR("[TC] SPI device not opened: {}");
    return false;
  }

  return (kChip == Chip::MAX31855) ? readThermocouple(out) : readPT100(out);
}

void Thermocouple::logData(const LogEntry& rec) {
  if (!queue_)
    return;

  const std::size_t HIGH_WATER = queue_->capacity() * 8 / 10;
  std::size_t qsz = queue_->size();
  std::size_t qcap = queue_->capacity();

  bool pushed = queue_->try_emplace(rec);

  if (!pushed) {
    SPDLOG_WARN("[TC] queue overflow, dropping sample (size={}/{})", qsz, qcap);
    SPDLOG_DEBUG("[TC] notifying logger due to overflow");
    sober::logger::Logger::instance().notify();
  } else if (qsz >= HIGH_WATER) {
    SPDLOG_DEBUG("[TC] high-water mark reached: {}/{} ({}%)", qsz, qcap,
                 (qsz * 100) / qcap);
    sober::logger::Logger::instance().notify();
  }
}

void Thermocouple::run(std::stop_token stoken) {
  auto next_wakeup = std::chrono::steady_clock::now() + sample_period;

  while (!stoken.stop_requested()) {
#ifndef DEBUG_THERMO
    Reading res{};
    if (readOnce(res)) {
      // timestamp
      sober::logger::LogEntry rec{};
      timespec ts{};
      if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
        ts.tv_sec = 0;
        ts.tv_nsec = 0;
      }
      rec.timestamp = static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
                      static_cast<uint64_t>(ts.tv_nsec);

      // status bits: bit3=fault, bit2=scv, bit1=scg, bit0=oc
      // float status = static_cast<float>(
      //     ((res.fault ? 1u : 0u) << 3) |
      //     ((res.scv   ? 1u : 0u) << 2) |
      //     ((res.scg   ? 1u : 0u) << 1) |
      //     ((res.oc    ? 1u : 0u) << 0)
      // );

      // keep 4-float payload like other sensors: [Tc, CJ, status, reserved]
      std::array<float, 2> values = {
          static_cast<float>(res.thermocouple_c),
          static_cast<float>(res.internal_c),
      };

      uint64_t generator = gSnapshot.generation.load(std::memory_order_relaxed);
      gSnapshot.generation.store(generator + 1, std::memory_order_release);

      gSnapshot.thermocouple_c = values[0];
      gSnapshot.internal_c = values[1];

      gSnapshot.generation.store(generator + 2, std::memory_order_release);

      constexpr size_t packageSize = sizeof(values);
      rec.length = static_cast<uint8_t>(packageSize);
      std::memcpy(rec.data.data(), values.data(), packageSize);

      logData(rec);

      if (res.fault) {
        SPDLOG_WARN("[TC] Fault on {} [SCV={}, SCG={}, OC={}]", dev_, res.scv,
                    res.scg, res.oc);
      } else {
        // SPDLOG_INFO("[TC] Tc={:.2f}C, CJ={:.2f}C", res.thermocouple_c,
        //             res.internal_c);
      }
    } else {
      SPDLOG_WARN("[TC] Failed to read valid data");
    }
#else
    // Debug mode: synthesize data
    Reading res{};
    res.thermocouple_c = 25.0;
    res.internal_c = 24.0;

    sober::logger::LogEntry rec{};
    timespec ts{};
    if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
      ts.tv_sec = 0;
      ts.tv_nsec = 0;
    }
    rec.timestamp = static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
                    static_cast<uint64_t>(ts.tv_nsec);

    std::array<float, 2> values = {
        static_cast<float>(res.thermocouple_c),
        static_cast<float>(res.internal_c),
    };
    rec.length = static_cast<uint8_t>(sizeof(values));
    std::memcpy(rec.data.data(), values.data(), sizeof(values));

    logData(rec);
    // SPDLOG_INFO("[TC][DBG] Tc={:.2f}C, CJ={:.2f}C", res.thermocouple_c,
    //             res.internal_c);
#endif

    std::unique_lock<std::mutex> lk(cvMutex_);
    cv_.wait_until(lk, next_wakeup, [&] { return stoken.stop_requested(); });
    next_wakeup += sample_period;
  }
}

}  // namespace sober::housekeeping