#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <spdlog/spdlog.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>

#include <cstdarg>
#include <cstdint>
#include <cstdio>

#include "imu/inv_imu_transport.h"
#include "logger/Logger.hpp"
#include "system_interface.h"

static int fd = -1;
inline constexpr uint8_t IMU_ADDR = 0x68;

int si_board_init() {
  return 0;
}

int si_io_imu_init(inv_imu_serif_type_t serif_type) {
  if (serif_type != UI_I2C) {
    return -1;
  }

  fd = ::open("/dev/i2c-1", O_RDWR);

  if (fd < 0) {
    SPDLOG_CRITICAL("[IMU] Failed to open I2C Bus");
    return -1;
  }

  if (ioctl(fd, I2C_SLAVE, IMU_ADDR) < 0) {
    SPDLOG_CRITICAL("[IMU] Failed to set I2C slave address");
    return -1;
  }

  return 0;
}

static int rw(bool isWrite, uint8_t reg, uint8_t* buf, uint32_t len) {
  if (fd < 0) {
    return -1;
  }

  if (isWrite) {
    std::vector<uint8_t> tmp(len + 1);
    tmp[0] = reg;
    memcpy(tmp.data() + 1, buf, len);
    return (::write(fd, tmp.data(), len + 1) == static_cast<int>(len) + 1) ? 0
                                                                           : -1;
  }
  if (::write(fd, &reg, 1) != 1) {
    return -1;
  }
  return (::read(fd, buf, len) == static_cast<int>(len)) ? 0 : -1;
}

int si_io_imu_read_reg(uint8_t reg, uint8_t* buf, uint32_t len) {
  return rw(false, reg, buf, len);
}

int si_io_imu_write_reg(uint8_t reg, const uint8_t* buf, uint32_t len) {
  return rw(true, reg, const_cast<uint8_t*>(buf), len);
}

void si_sleep_us(uint32_t us) {
  usleep(us);
}

uint64_t si_get_time_us() {
  struct timeval timeVal {};
  gettimeofday(&timeVal, nullptr);
  return static_cast<uint64_t>(timeVal.tv_sec) * 1000000ULL +
         static_cast<uint64_t>(timeVal.tv_usec);
}

void inv_msg(int level, const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);

  char buffer[512];
  vsnprintf(buffer, sizeof(buffer), fmt, ap);

  switch (level) {
    case 1:  // ERR
      SPDLOG_CRITICAL("{}", buffer);
      break;
    case 2:  // WRN
      SPDLOG_WARN("{}", buffer);
      break;
    case 3:  // INF
      SPDLOG_INFO("{}", buffer);
      break;
    case 5:  // DBG
      SPDLOG_DEBUG("{}", buffer);
      break;
    default:
      SPDLOG_TRACE("{}", buffer);
      break;
  }

  va_end(ap);
}

int si_config_uart_for_print(...) {
  return 0;
}
int si_config_uart_for_bin(...) {
  return 0;
}
int si_get_uart_command(...) {
  return 0;
}
int si_init_timers() {
  return 0;
}
int si_init_gpio_int(...) {
  return 0;
}
int si_start_gpio_fsync(...) {
  return 0;
}
int si_stop_gpio_fsync() {
  return 0;
}
void si_toggle_gpio_fsync(void) {}
void si_disable_irq() {}
void si_enable_irq() {}
int si_print_error_if_any(int rc) {
  return (rc != 0);
}
