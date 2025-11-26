#include "logger/ImageLogger.hpp"

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <ctime>

#include <fcntl.h>
#include <linux/limits.h>
#include <spdlog/spdlog.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/syscall.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>

#ifndef IMGLOG_TIMING
#define IMGLOG_TIMING 1
#endif

#if IMGLOG_TIMING
#include <chrono>
using Steady = std::chrono::steady_clock;
static inline uint64_t us_since(const Steady::time_point& t0) {
  return (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(
             Steady::now() - t0)
      .count();
}
#endif

namespace imglog {

template <size_t QueueCapacity>
ImageLogger<QueueCapacity>::ImageLogger(FlushPolicy policy)
    : flush_policy_(policy) {}

template <size_t QueueCapacity>
ImageLogger<QueueCapacity>::~ImageLogger() {
  stop(true);
}

template <size_t QueueCapacity>
void ImageLogger<QueueCapacity>::start() {
  bool expected = false;
  if (!running_.compare_exchange_strong(expected, true))
    return;

  ensureDirExists(kBaseDir);

  // Initialize free list with all indices [0..QueueCapacity-1]
  for (Index i = 0; i < QueueCapacity; ++i) {
    (void)free_ring_.insert(i);
  }

  worker_ = std::thread([this] { this->workerLoop(); });
}

template <size_t QueueCapacity>
void ImageLogger<QueueCapacity>::stop(bool drain) {
  if (!running_.exchange(false))
    return;
  {
    std::lock_guard<std::mutex> lk(m_);
    if (!drain)
      pending_.store(0);
  }
  cv_.notify_all();
  if (worker_.joinable())
    worker_.join();
}

template <size_t QueueCapacity>
uint64_t ImageLogger<QueueCapacity>::nowRealtimeNs() {
  timespec ts{};
  clock_gettime(CLOCK_REALTIME, &ts);
  return uint64_t(ts.tv_sec) * 1000000000ull + uint64_t(ts.tv_nsec);
}

template <size_t QueueCapacity>
bool ImageLogger<QueueCapacity>::enqueue(std::vector<uint8_t>&& img) {
  if (!running_.load(std::memory_order_relaxed))
    return false;

  // Grab a free slot index from the free ring
  Index idx;
  if (!free_ring_.remove(idx)) {
    dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;  // no free slots -> back-pressure/drop
  }

  // Move the big vector into the pool slot (no deep copy)
  pool_[idx].data = std::make_shared<std::vector<uint8_t>>(std::move(img));
  pool_[idx].seq = seq_.fetch_add(1, std::memory_order_relaxed);
  pool_[idx].realtime_ns = nowRealtimeNs();

  // Publish the index to the consumer
  if (!data_ring_.insert(idx)) {
    // extremely unlikely with correct use, but recycle on failure
    pool_[idx] = {};
    (void)free_ring_.insert(idx);
    dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  pending_.fetch_add(1, std::memory_order_release);
  cv_.notify_one();
  return true;
}

template <size_t QueueCapacity>
void ImageLogger<QueueCapacity>::setTimingCallback(
    std::function<void(const WriteTiming&)> cb) {
  timing_cb_ = std::move(cb);
}

template <size_t QueueCapacity>
std::string ImageLogger<QueueCapacity>::makeFilename(uint64_t seq,
                                                     uint64_t rt_ns) {
  time_t sec = time_t(rt_ns / 1000000000ull);
  struct tm tm {};
  localtime_r(&sec, &tm);

  char buf[256];
  int n =
      snprintf(buf, sizeof(buf), "%s/image_%04d%02d%02d_%02d%02d%02d_%llu.raw",
               ::imglog::kBaseDir, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
               tm.tm_hour, tm.tm_min, tm.tm_sec, (unsigned long long)seq);
  return std::string(buf, n > 0 ? (size_t)n : 0);
}

template <size_t QueueCapacity>
bool ImageLogger<QueueCapacity>::ensureDirExists(const char* path) {
  struct stat st {};
  if (::stat(path, &st) == 0)
    return S_ISDIR(st.st_mode);
  if (::mkdir(path, 0755) == 0)
    return true;
  return errno == EEXIST;
}

template <size_t QueueCapacity>
bool ImageLogger<QueueCapacity>::writeOne(uint64_t seq,
                                          const std::vector<uint8_t>& buf) {
#if IMGLOG_TIMING
  WriteTiming wt{};
  wt.seq = seq;
  wt.bytes = buf.size();
  const auto t0 = Steady::now();
  auto t_open_start = t0;
#endif

  const std::string filename = makeFilename(seq, nowRealtimeNs());
  int fd =
      ::open(filename.c_str(), O_CREAT | O_WRONLY | O_TRUNC | O_CLOEXEC, 0644);
  if (fd < 0)
    return false;

#if IMGLOG_TIMING
  wt.open_us = us_since(t_open_start);
  auto t_falloc_start = Steady::now();
#endif

  (void)posix_fallocate(fd, 0, (off_t)buf.size());

#if IMGLOG_TIMING
  wt.fallocate_us = us_since(t_falloc_start);
  auto t_write_start = Steady::now();
#endif

  const uint8_t* p = buf.data();
  size_t left = buf.size();
  while (left > 0) {
    ssize_t n = ::write(fd, p, left);
    if (n < 0) {
      if (errno == EINTR)
        continue;
      ::close(fd);
      ::unlink(filename.c_str());
      return false;
    }
    p += (size_t)n;
    left -= (size_t)n;
  }

#if IMGLOG_TIMING
  wt.write_us = us_since(t_write_start);
  auto t_fsync_start = Steady::now();
#endif

  if (flush_policy_ == FlushPolicy::EveryImage)
    ::fdatasync(fd);

#if IMGLOG_TIMING
  wt.fsync_us = us_since(t_fsync_start);
#endif

  (void)posix_fadvise(fd, 0, 0, POSIX_FADV_DONTNEED);
  ::close(fd);

#if IMGLOG_TIMING
  wt.total_us = us_since(t0);
  //   if (timing_cb_) {
  //     timing_cb_(wt);  // send to your callback
  //   } else {
  // or log directly; keep formatting minimal to reduce overhead
  // (Uncomment if you want logs without a callback)
  SPDLOG_INFO(
      "[RAWWRITE] seq={} bytes={} open={}us falloc={}us write={}us fsync={}us "
      "total={}us",
      wt.seq, wt.bytes, wt.open_us, wt.fallocate_us, wt.write_us, wt.fsync_us,
      wt.total_us);
//   }
#endif

  return true;
}

template <size_t QueueCapacity>
void ImageLogger<QueueCapacity>::workerLoop() {
  while (running_.load(std::memory_order_acquire) ||
         pending_.load(std::memory_order_acquire) > 0) {

    Index idx;
    if (!data_ring_.remove(idx)) {
      std::unique_lock<std::mutex> lk(m_);
      cv_.wait_for(lk, std::chrono::milliseconds(10), [&] {
        return !running_.load(std::memory_order_relaxed) || pending_.load() > 0;
      });
      continue;
    }
    pending_.fetch_sub(1, std::memory_order_release);

    auto& slot = pool_[idx];
    if (slot.data && !slot.data->empty()) {
      if (writeOne(slot.seq, *slot.data)) {
        written_.fetch_add(1, std::memory_order_relaxed);
      }
    }
    // clear and recycle the slot
    slot = {};
    (void)free_ring_.insert(idx);
  }

  if (flush_policy_ == FlushPolicy::OnStop) {
    // nothing else to do; kept for symmetry/future
  }
}

// Explicit instantiations (optional)
template class ImageLogger<64>;
template class ImageLogger<128>;
template class ImageLogger<256>;

}  // namespace imglog