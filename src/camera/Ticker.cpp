#include "camera/Ticker.hpp"

#include <cerrno>
#include <cstdint>
#include <ctime>
#include <functional>
#include <mutex>

#include <sys/timerfd.h>
#include <unistd.h>

namespace {
inline long long nsNowMonotonic() {
  timespec now{};
  clock_gettime(CLOCK_MONOTONIC, &now);
  return 1000000000LL * now.tv_sec + now.tv_nsec;
}
}  // namespace

namespace sober::camera {

Ticker::Ticker(std::chrono::nanoseconds period)
    : tickerThread_(&Ticker::loop_, std::ref(bus_), period) {}

Ticker::~Ticker() {
  tickerThread_.request_stop();
  tickerThread_.join();
  {
    std::scoped_lock lk(bus_.tickM);
    bus_.stop = true;
  }
  bus_.cv.notify_all();
}

void Ticker::waitNext(uint64_t& seen) {
  std::unique_lock lock(bus_.tickM);
  bus_.cv.wait(lock, [&] { return bus_.tick > seen || bus_.stop; });
  seen = bus_.tick;
}

uint64_t Ticker::current() {
  std::scoped_lock lock(bus_.tickM);
  return bus_.tick;
}

void Ticker::loop_(std::stop_token st, TickBus& bus,
                   std::chrono::nanoseconds period) {
  const int tfd = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);
  if (tfd < 0) {
    throw std::runtime_error("timerfd_create failed");
  }

  const long long per_ns = period.count();
  const long long next_ns = ((nsNowMonotonic() + per_ns) / per_ns) * per_ns;

  itimerspec its{};
  its.it_value.tv_sec = next_ns / 1000000000LL;
  its.it_value.tv_nsec = next_ns % 1000000000LL;
  its.it_interval.tv_sec = per_ns / 1000000000LL;
  its.it_interval.tv_nsec = per_ns % 1000000000LL;

  if (timerfd_settime(tfd, TFD_TIMER_ABSTIME, &its, nullptr) < 0) {
    close(tfd);
    throw std::runtime_error("timerfd_settime failed");
  }

  while (!st.stop_requested()) {
    uint64_t expirations = 0;
    ssize_t r = read(tfd, &expirations, sizeof(expirations));
    if (r != sizeof(expirations)) {
      if (r < 0 && errno == EINTR) {
        continue;
      }
      continue;
    }
    {
      std::scoped_lock lk(bus.tickM);
      bus.tick += (expirations != 0u) ? expirations : 1;
      bus.cv.notify_all();
    }
  }

  close(tfd);
  {
    std::scoped_lock lk(bus.tickM);
    bus.stop = true;
  }
  bus.cv.notify_all();
}

}  // namespace sober::camera