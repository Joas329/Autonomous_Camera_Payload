#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <stop_token>
#include <thread>

class RateLimiter {
 public:
  explicit RateLimiter(std::int64_t rate_bps, std::int64_t burst_bytes = -1,
                       bool start_full = true)
      : rate_(static_cast<long double>(mustBePositive(rate_bps))),
        capacity_(static_cast<long double>(
            burst_bytes < 0 ? rate_bps : mustBeNonNegative(burst_bytes))),
        tokens_(start_full ? capacity_ : 0.0L),
        last_(std::chrono::steady_clock::now()) {}

  std::int64_t available() {
    refill();
    return static_cast<std::int64_t>(std::floor(tokens_));
  }

  bool tryConsume(std::int64_t n) {
    if (n <= 0) {
      return true;
    }
    refill();
    if (static_cast<long double>(n) > capacity_) {
      return false;
    }
    if (tokens_ + 1e-12L < static_cast<long double>(n)) {
      return false;
    }
    tokens_ -= static_cast<long double>(n);
    return true;
  }

  void consumeBlocking(std::int64_t n, std::stop_token st) {
    if (n <= 0) {
      return;
    }
    if (static_cast<long double>(n) > capacity_) {
      throw std::invalid_argument("request exceeds burst capacity");
    }
    for (;;) {
      refill();
      if (tokens_ + 1e-12L >= static_cast<long double>(n)) {
        tokens_ -= static_cast<long double>(n);
        return;
      }
      if (st.stop_requested()) {
        throw std::runtime_error("stop requested");
      }
      const long double deficit = static_cast<long double>(n) - tokens_;
      const auto wait_us =
          static_cast<long long>(std::ceil((deficit * 1000000.0) / rate_));
      std::this_thread::sleep_for(
          std::chrono::microseconds(std::max<long long>(1, wait_us)));
    }
  }

  void setRate(std::int64_t rate_bps) {
    if (rate_bps <= 0) {
      throw std::invalid_argument("rate_bps must be > 0");
    }
    refill();
    rate_ = static_cast<long double>(rate_bps);
  }

 private:
  static std::int64_t mustBePositive(std::int64_t v) {
    if (v <= 0) {
      throw std::invalid_argument("rate_bps must be > 0");
    }
    return v;
  }
  static std::int64_t mustBeNonNegative(std::int64_t v) {
    if (v < 0) {
      throw std::invalid_argument("burst_bytes must be >= 0");
    }
    return v;
  }

  void refill() {
    const auto now = std::chrono::steady_clock::now();
    const auto us =
        std::chrono::duration_cast<std::chrono::microseconds>(now - last_)
            .count();
    if (us <= 0) {
      return;
    }
    last_ = now;
    const long double add = (static_cast<long double>(us) * rate_) / 1000000.0;
    tokens_ = std::min(capacity_, tokens_ + add);
  }

  long double rate_;            // bytes/sec
  const long double capacity_;  // max burst (bytes)
  long double tokens_;          // fractional tokens
  std::chrono::steady_clock::time_point last_;
};

inline RateLimiter& getLimiterInstance() {
  static RateLimiter rateLimiter(28850);
  return rateLimiter;
}
