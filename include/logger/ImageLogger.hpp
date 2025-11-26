#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include "ringbuffer.hpp"

namespace imglog {

struct Slot {
  std::shared_ptr<std::vector<uint8_t>> data;  // non-trivial, kept out of ring
  uint64_t seq{};
  uint64_t realtime_ns{};
};

struct WriteTiming {
  uint64_t seq{};
  size_t bytes{};
  uint64_t open_us{};
  uint64_t fallocate_us{};
  uint64_t write_us{};
  uint64_t fsync_us{};
  uint64_t total_us{};
};

inline constexpr const char* kBaseDir = "/mnt/adata/OPTICAL";

enum class FlushPolicy : uint8_t { Never, EveryImage, OnStop };

template <size_t QueueCapacity>
class ImageLogger {
 public:
  explicit ImageLogger(FlushPolicy policy = FlushPolicy::Never);
  ~ImageLogger();

  void start();
  void stop(bool drain = true);

  bool enqueue(std::vector<uint8_t>&& img);

  uint64_t dropped() const { return dropped_; }
  uint64_t written() const { return written_; }

  void setTimingCallback(std::function<void(const WriteTiming&)> cb);

 private:
  std::function<void(const WriteTiming&)> timing_cb_;

  void workerLoop();
  bool writeOne(uint64_t seq, const std::vector<uint8_t>& buf);
  static uint64_t nowRealtimeNs();
  static bool ensureDirExists(const char* path);
  static std::string makeFilename(uint64_t seq, uint64_t rt_ns);

  using Index = uint32_t;
  using DataRing = jnk0le::Ringbuffer<Index, QueueCapacity, false, 64, size_t>;
  using FreeRing = jnk0le::Ringbuffer<Index, QueueCapacity, false, 64, size_t>;

  DataRing data_ring_;
  FreeRing free_ring_;

  Slot pool_[QueueCapacity];

  std::atomic<uint64_t> pending_{0};
  std::mutex m_;
  std::condition_variable cv_;
  std::atomic<bool> running_{false};
  std::thread worker_;

  std::atomic<uint64_t> seq_{0};
  std::atomic<uint64_t> dropped_{0};
  std::atomic<uint64_t> written_{0};

  const FlushPolicy flush_policy_;
};

}  //namespace imglog
