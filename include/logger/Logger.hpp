#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>


#include <rigtorp/SPSCQueue.h>
#include <spdlog/fmt/fmt.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include "eventbus/EventBus.hpp"

inline constexpr std::size_t MAX_RAW_SIZE = 32;
inline constexpr std::size_t CAPACITY = 4096;
inline constexpr std::size_t FLUSH_THRESHOLD = 256;
inline constexpr std::chrono::milliseconds FLUSH_INTERVAL{1000};

namespace sober::logger {

inline void initVerbosity(int verbosity = 0)  // 0‑6
{
  using spdlog::level::level_enum;
  verbosity = std::clamp(verbosity, 0, 6);

  static constexpr std::string_view orange_ansi = "\033[38;5;208m";
  auto color_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  color_sink->set_color(level_enum::trace, color_sink->blue);
  color_sink->set_color(level_enum::debug, orange_ansi);
  color_sink->set_color(level_enum::info, color_sink->green);
  color_sink->set_color(level_enum::warn, color_sink->yellow);
  color_sink->set_color(level_enum::err, color_sink->red);
  color_sink->set_color(level_enum::critical, color_sink->magenta);

  auto logger = std::make_shared<spdlog::logger>("sober", color_sink);
  spdlog::set_default_logger(logger);

  spdlog::set_level(static_cast<level_enum>(verbosity));

  logger->set_pattern("%^[%Y-%m-%d %H:%M:%S.%e] [%l]%$ %v");
}

template <typename... Args>
void printLogInfoImpl(const std::string& fmt,
                      std::optional<std::string> colorOpt, Args&&... args);

struct LogColor {
  std::string color;
};

template <typename... Args>
inline void printLogInfo(const std::string& fmt, Args&&... args) {
  printLogInfoImpl(fmt, std::nullopt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void printLogInfo(const std::string& fmt, LogColor color,
                         Args&&... args) {
  printLogInfoImpl(fmt, color.color, std::forward<Args>(args)...);
}

// Template function definition
template <typename... Args>
inline void printLogInfoImpl(const std::string& fmt,
                             std::optional<std::string> colorOpt,
                             Args&&... args) {
  static const std::unordered_map<std::string, std::string> color_codes = {
      {"green", "\033[32m"},
      {"yellow", "\033[33m"},
      {"blue", "\033[34m"},
      {"magenta", "\033[35m"},
      {"orange", "\033[38;5;208m"},
      {"white", "\033[37m"},
      {"red", "\033[31m"},
      {"cyan", "\033[36m"},
      {"black", "\033[30m"},
      {"bright_green", "\033[92m"},
      {"bright_yellow", "\033[93m"},
      {"bright_blue", "\033[94m"},
      {"bright_magenta", "\033[95m"},
      {"bright_cyan", "\033[96m"}};

  auto logger = spdlog::default_logger_raw();
  std::string color_code = "\033[32m";  // Default green

  if (colorOpt) {
    auto it = color_codes.find(*colorOpt);
    if (it != color_codes.end()) {
      color_code = it->second;
    }
  }

  logger->set_pattern(color_code + "[%Y-%m-%d %H:%M:%S.%e] [info]\033[m %v");
  spdlog::info(fmt::runtime(fmt), std::forward<Args>(args)...);
}

void info(const std::string& message);
void error(const std::string& message);

#pragma pack(push, 1)
struct LogEntry {
  uint64_t timestamp{};
  uint8_t length{};
  std::array<uint8_t, MAX_RAW_SIZE> data{};
};
#pragma pack(pop)

class Logger {
 public:
  using Queue = rigtorp::SPSCQueue<LogEntry>;

  static Logger& instance();

  Logger(const Logger&) = delete;
  Logger& operator=(const Logger&) = delete;

  void start();

  std::shared_ptr<Queue> createChannel(const std::string& path,
                                       std::size_t capacity = CAPACITY);

  void stop();
  void notify() noexcept;

 private:
  Logger() = default;
  ~Logger() = default;

  void run();
  void drainOnce();

  struct Channel {
    std::string path;
    std::shared_ptr<Queue> queue;
    int fileDescriptor_;
    std::vector<LogEntry> flushBuffer;
    std::chrono::steady_clock::time_point lastFlush_{
        std::chrono::steady_clock::now()};
  };

  std::atomic<bool> running_{false};
  std::thread thread_;
  std::vector<Channel> channels_;
  std::mutex chMutex_;
  std::mutex cvMutex_;
  std::condition_variable cv_;
};

}  // namespace sober::logger
