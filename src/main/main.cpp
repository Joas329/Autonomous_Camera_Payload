#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>

#include "logger/Logger.hpp"
#include "manager/SystemController.hpp"

namespace {
std::atomic<bool> g_running{true};

void onSignal(int /*signum*/) noexcept {
  g_running.store(false);
}
}  // namespace

// Verbosity levels mapping:
// 0 = trace    (prints EVERYTHING)
// 1 = debug    (prints debug + above)
// 2 = info     (prints info + above)
// 3 = warn     (prints warn + above)
// 4 = error    (prints error + above)
// 5 = critical (prints critical only)
// 6 = off      (prints NOTHING)

int main(int argc, char** argv) {
  int userVerbosity = argc > 1 ? std::atoi(argv[1]) : 0;
  sober::logger::initVerbosity(userVerbosity);

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  SystemController controller;
  controller.start();

  while (g_running.load()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  controller.stop();
  SPDLOG_INFO("[MAIN] SystemController stopped");
  return 0;
}