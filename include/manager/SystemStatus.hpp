#pragma once

#include <string>

struct SystemStatus {
    double cpu_temp_c;
    double cpu_load_pct;
    double disk_used_gb;
    double disk_total_gb;
    double uptime_s;
    std::string utc_iso;
};
