#pragma once
#include <spdlog/spdlog.h>
#include <cmath>
extern "C" {
#include "gpio.h"
}
#include <stdexcept>
#include <string>

class Heater {
 public:
  explicit Heater(const char* chip = "/dev/gpiochip0", unsigned line = 16)
      : gpio_(gpio_new()) {
    if (!gpio_) {
      SPDLOG_ERROR("[HEATER]: gpio_new failed");
    }
    if (gpio_open(gpio_, chip, line, GPIO_DIR_OUT) < 0) {
      std::string msg = gpio_errmsg(gpio_);
      gpio_free(gpio_);
      SPDLOG_ERROR("[HEATER] gpio_open failed: {}", msg);
    }
    // default OFF
    if (gpio_write(gpio_, false) < 0) {
      SPDLOG_ERROR("[HEATER] gpio_write failed");
    }
  }

  ~Heater() {
    if (gpio_) {
      gpio_close(gpio_);
      gpio_free(gpio_);
    }
  }

  bool on() { return gpio_write(gpio_, true) >= 0; }
  bool off() { return gpio_write(gpio_, false) >= 0; }

 private:
  gpio_t* gpio_{nullptr};
};
