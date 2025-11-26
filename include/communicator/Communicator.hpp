#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <span>
#include <stop_token>
#include <thread>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

#include "communicator/SoberProtocol.hpp"
#include "housekeeping/SensorSnapshot.hpp"

inline constexpr uint16_t DEFAULT_PORT = 5000;
inline constexpr int HEADER = 10;
inline constexpr std::string_view PROJECT_ROOT = ROOT;
inline constexpr float G = 9.80665F;
inline constexpr float FS_G = 20.0F;
inline constexpr float MAX_I16 = 32767.0F;
inline constexpr float GYRO_FULL_SCALE = 2000.0F;
inline constexpr float MAG_FULL_SCALE_GAUSS = 16.0F;
inline constexpr float TEMP_FULL_SCALE_C = 85.0F;
inline constexpr float PRESS_FULL_SCALE_HPA = 110000.0F;
inline constexpr float HUM_FULL_SCALE_PERC = 100.0F;

static constexpr float SCALE = MAX_I16 / (FS_G * G);
static constexpr float GYRO_SCALE = MAX_I16 / GYRO_FULL_SCALE;
static constexpr float MAG_SCALE = MAX_I16 / MAG_FULL_SCALE_GAUSS;
static constexpr float TEMP_SCALE = MAX_I16 / TEMP_FULL_SCALE_C;
static constexpr float PRESS_SCALE = MAX_I16 / PRESS_FULL_SCALE_HPA;
static constexpr float HUM_SCALE = MAX_I16 / HUM_FULL_SCALE_PERC;
static constexpr float FS_VOLT = 36.0F;
static constexpr float SCALE_VOLT = MAX_I16 / FS_VOLT;
static constexpr float FS_CURR = 10.0F;
static constexpr float SCALE_CURR = MAX_I16 / FS_CURR;

inline int16_t saturateI16(long value) {
  if (value > 32767L) {
    return 32767;
  }
  if (value < -32767L) {
    return -32767;
  }
  return static_cast<int16_t>(value);
}

inline int16_t encodeAccel(float a_mps2) {
  return static_cast<int16_t>(lroundf(a_mps2 * SCALE));
}

inline int16_t encodeGyr(float g_degs) {
  return static_cast<int16_t>(lroundf(g_degs * GYRO_SCALE));
}

inline int16_t encodeImuTemp(float temp_c) {
  return static_cast<int16_t>(lroundf((temp_c - 25.0F) * 128.0F));
}

inline int16_t encodeMag(float mag_gauss) {
  // return static_cast<int16_t>(lroundf(mag_gauss * MAG_SCALE));
  return saturateI16(lroundf(mag_gauss * 300.0F));
}

inline int16_t encodeMagTemp(float temp_c) {
  return static_cast<int16_t>(lroundf((temp_c - 25.0F) * 8.0F));
}

inline int16_t encodeTemp(float temp_c) {
  return static_cast<int16_t>(lroundf(temp_c * TEMP_SCALE));
}

inline int16_t encodePress(float press_hpa) {
  return static_cast<int16_t>(lroundf(press_hpa * PRESS_SCALE));
}

inline int16_t encodeHum(float hum_perc) {
  return static_cast<int16_t>(lroundf(hum_perc * HUM_SCALE));
}

inline int16_t encodeBusVoltage(float volts) {
  // return static_cast<int16_t>(lroundf(volts * SCALE_VOLT));
  return saturateI16(llroundf(volts * 100.0F));
}

inline int16_t encodeCurrent(float amps) {
  // return static_cast<int16_t>(lroundf(amps * SCALE_CURR));
  return saturateI16(lroundf(amps * 1000.0F));
}

static constexpr int16_t I16_MAX =
    std::numeric_limits<int16_t>::max();  //  32767
static constexpr int16_t I16_MIN =
    std::numeric_limits<int16_t>::min();  // -32768

static constexpr int MAX_SCALE = 100;  // 0.01 °C
// static constexpr int SCALE = 10;  // 0.1 °C

inline int16_t encodeTempC(float temp_c) {
  int64_t raw = lroundf(temp_c * MAX_SCALE);  // nearest
  raw = std::clamp(raw, static_cast<int64_t>(I16_MIN),
                   static_cast<int64_t>(I16_MAX));
  return static_cast<int16_t>(raw);
}

namespace sober::communicator {

inline constexpr std::size_t BUFFER_SIZE = 4096;

inline constexpr uint32_t LISTEN_ADDR = 0x7F000001u;  // 127.0.0.1

inline std::string to_hex(const uint8_t* data, std::size_t len) {
  static const char* hexdigits = "0123456789ABCDEF";
  std::string s;
  s.reserve(len * 3);
  for (std::size_t i = 0; i < len; ++i) {
    uint8_t b = data[i];
    s.push_back(hexdigits[b >> 4]);
    s.push_back(hexdigits[b & 0x0F]);
    s.push_back(' ');
  }
  return s;
}
class Communicator {
 public:
  explicit Communicator(uint16_t port);
  ~Communicator();

  Communicator(const Communicator&) = delete;
  Communicator& operator=(const Communicator&) = delete;

  void start();
  void stop();

  void setCommandHandler(
      std::function<void(uint16_t, std::span<const uint8_t>)> handler) {
    soberProtocol_.setCommandHandler(std::move(handler));
  }

  void setTelemetryEnabled(bool on) {
    telemetryEnabled_.store(on, std::memory_order_release);
  }

  void enableSOBERProtocol(bool on) {
    soberProtocol_.enableProtocol(on);
  }  // VERY BAD!

 private:
  struct OutMsg {
    std::vector<uint8_t> data;
    std::size_t offset = 0;
  };

  void run(std::stop_token stoken);

  void recordSnapshot(SensorSnapshot& snapshot);

  static int make_listen_socket(uint16_t port);
  static int accept_client(int listen_fd);
  static bool set_nonblocking(int fd);
  static void close_fd(int& fd);

 private:
  uint16_t port_;
  std::jthread thread_;
  int listen_fd_{-1};
  int client_fd_{-1};

  SoberProtocol soberProtocol_;
  std::atomic<bool> telemetryEnabled_{false};

  std::array<uint8_t, BUFFER_SIZE> readBuffer_{};
  std::deque<OutMsg> outQueue_;

  InPacket<90000> optBuf_{};

  void drainOpticalQueueToOut();
};

}  // namespace sober::communicator
