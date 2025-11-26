#pragma once

#include <chrono>
#include <string>

namespace sober::eventbus {

enum class EventType { SensorData, CameraData, HealthStatus, Shutdown };

struct Event {
  EventType type;
  std::string source;
  double value;  // dummy sensor reading
  std::chrono::system_clock::time_point timestamp;
};

}  // namespace sober::eventbus