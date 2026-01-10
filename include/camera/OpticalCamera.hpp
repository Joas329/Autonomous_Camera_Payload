#pragma once

#include <cstddef>
#include <mutex>
#include <stop_token>
#include <thread>
#include <vector>

#include "camera/Ticker.hpp"

namespace sober::camera {

class OpticalCamera {
 public:
  explicit OpticalCamera(Ticker& tick) : ticker_(tick) {}

  ~OpticalCamera() = default;

  bool start();
  bool stop();

 private:
  void run(std::stop_token stoken);

  void pushJpegToBuffer(std::vector<uint8_t> jpg, size_t w, size_t h,
                        size_t stride, uint8_t channels);

  sober::camera::Ticker& ticker_;
  std::jthread opticalThread_;
  std::mutex bufferMutex_;
  std::vector<uint8_t> lastImgae_;
  size_t last_w_{0};
  size_t last_h_{0};
  size_t last_stride_{0};
  size_t last_channels_{0};
};

}  // namespace sober::camera
