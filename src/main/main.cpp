#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>
#include <filesystem>
#include <string>
#include <cstdlib>
#include <unistd.h>   // readlink
#include <limits.h>   // PATH_MAX

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

static std::filesystem::path getExeDir() {
  char buf[PATH_MAX]{0};
  ssize_t len = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
  if (len <= 0) return {};
  buf[len] = '\0';
  return std::filesystem::path(buf).parent_path();
}

static void updateUiBaseFromWifi() {
  const auto exeDir = getExeDir();
  if (exeDir.empty()) {
    SPDLOG_WARN("[MAIN] Could not resolve executable directory; skipping UI BASE update");
    return;
  }

  const auto script = (exeDir / "ui" / "update_base_from_nmcli.sh");
  const auto html   = (exeDir / "ui" / "control_panel.html");

  if (!std::filesystem::exists(script) || !std::filesystem::exists(html)) {
    SPDLOG_WARN("[MAIN] UI update script or HTML missing: script='{}' html='{}'",
                script.string(), html.string());
    return;
  }

  // Run: update_base_from_nmcli.sh <html> 8080 wlan0
  std::string cmd = "\"" + script.string() + "\" \"" + html.string() + "\" 8080 wlan0";
  SPDLOG_INFO("[MAIN] Updating UI BASE via: {}", cmd);

  int rc = std::system(cmd.c_str());
  if (rc != 0) {
    SPDLOG_WARN("[MAIN] UI BASE update script returned non-zero: {}", rc);
  }
}

int main(int argc, char** argv) {
  int userVerbosity = argc > 1 ? std::atoi(argv[1]) : 0;
  sober::logger::initVerbosity(userVerbosity);

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  updateUiBaseFromWifi();

  SystemController controller;
  controller.start();

  while (g_running.load()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  controller.stop();
  SPDLOG_INFO("[MAIN] SystemController stopped");
  return 0;
}
