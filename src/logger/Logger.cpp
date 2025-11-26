#include "logger/Logger.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>


#include "spdlog/spdlog.h"

#include "eventbus/Event.hpp"
#include "eventbus/EventBus.hpp"

namespace sober::logger {

void info(const std::string& message) {
  spdlog::info(message);
}

void error(const std::string& message) {
  spdlog::error(message);
}

Logger& Logger::instance() {
  static Logger instance;
  return instance;
}

void Logger::start() {
  if (running_) {
    return;
  }
  running_ = true;
  thread_ = std::thread(&Logger::run, this);
}

std::shared_ptr<Logger::Queue> Logger::createChannel(const std::string& path,
                                                     std::size_t capacity) {
  auto queue = std::make_shared<Queue>(capacity);
  int fileDescriptor =
      ::open(path.c_str(), O_CREAT | O_APPEND | O_WRONLY | O_CLOEXEC, 0644);
  if (fileDescriptor < 0) {
    SPDLOG_ERROR("[LOGGER] Open Log failed at path {}", path);
    return nullptr;
  }

  std::lock_guard lk(chMutex_);

  channels_.push_back(Channel{path, queue, fileDescriptor, {}});
  return queue;
}

void Logger::stop() {
  if (!running_) {
    return;
  }

  running_ = false;
  cv_.notify_all();

  if (thread_.joinable()) {
    thread_.join();
  }

  for (auto& ch : channels_) {
    ::close(ch.fileDescriptor_);
  }
}

void Logger::drainOnce() {
  std::vector<Channel*> chans;
  {
    std::lock_guard lk(chMutex_);
    chans.reserve(channels_.size());
    for (auto& ch : channels_) {
      chans.push_back(&ch);
    }
  }
  const auto now = std::chrono::steady_clock::now();
  for (auto* ch : chans) {
    for (;;) {
      LogEntry* p = ch->queue->front();
      if (!p) {
        break;
      }
      ch->flushBuffer.emplace_back(*p);
      ch->queue->pop();
    }
    if (!ch->flushBuffer.empty() &&
        (ch->flushBuffer.size() >= FLUSH_THRESHOLD ||
         (now - ch->lastFlush_) >= FLUSH_INTERVAL)) {
      ::write(ch->fileDescriptor_, ch->flushBuffer.data(),
              +ch->flushBuffer.size() * sizeof(LogEntry));
      ch->flushBuffer.clear();
      ch->lastFlush_ = now;
    }
  }
}

void Logger::notify() noexcept {
  cv_.notify_one();
}

void Logger::run() {
  std::unique_lock<std::mutex> lk(cvMutex_);
  while (running_.load(std::memory_order_relaxed)) {
    cv_.wait_for(lk, FLUSH_INTERVAL);
    lk.unlock();
    drainOnce();
    lk.lock();
  }

  lk.unlock();
  drainOnce();
  for (auto& c : channels_) {
    if (!c.flushBuffer.empty()) {
      ::write(c.fileDescriptor_, c.flushBuffer.data(),
              c.flushBuffer.size() * sizeof(LogEntry));
      c.flushBuffer.clear();
    }
  }
}

}  // namespace sober::logger
