#define ASIO_STANDALONE
#include <crow.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>
#include <crow/json.h>

#include "manager/SystemController.hpp"
#include "communicator/SoberApi.hpp"
#include "manager/SystemController.hpp"

static crow::response serve_file(
    const std::string& path,
    const std::string& content_type = "text/html"
) {
    if (!std::filesystem::exists(path)) {
        return crow::response(404);
    }

    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) {
        return crow::response(500);
    }

    std::ostringstream oss;
    oss << ifs.rdbuf();

    crow::response r;
    r.code = 200;
    r.set_header("Content-Type", content_type);
    r.body = oss.str();
    return r;
}

namespace sober::communicator {

SoberApi::SoberApi(SystemController& controller)
    : controller_(controller)
{}

void SoberApi::registerRoutes(crow::SimpleApp& app)
{
    CROW_ROUTE(app, "/")([] {
        SPDLOG_INFO("CWD = {}", std::filesystem::current_path().string());
        return serve_file("ui/control_panel.html", "text/html");
    });

    CROW_ROUTE(app, "/ui/<path>")
    ([](const std::string& path) {
        const std::string fullPath = "ui/" + path;

        if (path.ends_with(".js"))
            return serve_file(fullPath, "application/javascript");
        if (path.ends_with(".css"))
            return serve_file(fullPath, "text/css");
        if (path.ends_with(".html"))
            return serve_file(fullPath, "text/html");

        return crow::response(404);
    });

    // Test endpoint
    CROW_ROUTE(app, "/hello")([] {
        crow::response r("Hello from API!");
        r.code = 200;
        return r;
    });

    // Turn ON optical camera
    CROW_ROUTE(app, "/camera/opt/on").methods("POST"_method)
    ([this] {
        // Run camera start in background to avoid blocking UI
        std::thread([this] { controller_.startAcquisition(); }).detach();

        crow::response r("Optical camera ON");
        r.code = 200;
        return r;
    });

    // Turn OFF optical camera
    CROW_ROUTE(app, "/camera/opt/off").methods("POST"_method)
    ([this] {
        std::thread([this] { controller_.stopAcquisition(); }).detach();

        crow::response r("Optical camera OFF");
        r.code = 200;
        return r;
    });

    // Set optical exposure (microseconds)
    CROW_ROUTE(app, "/camera/opt/exposure").methods("POST"_method)
    ([this](const crow::request& req) {

        // --- Parse exposure_us from query or JSON body ---
        double exposure_us = -1.0;

        // A) Query string: ?us=12345
        if (req.url_params.get("us")) {
            try {
                exposure_us = std::stod(req.url_params.get("us"));
            } catch (...) {
                crow::response r("Invalid 'us' query param");
                r.code = 400;
                return r;
            }
        }
        // B) JSON body: { "us": 12345 }
        else if (!req.body.empty()) {
            auto body = crow::json::load(req.body);
            if (!body) {
                crow::response r("Invalid JSON body");
                r.code = 400;
                return r;
            }
            if (!body.has("us")) {
                crow::response r("Missing field 'us'");
                r.code = 400;
                return r;
            }
            try {
                exposure_us = body["us"].d();
            } catch (...) {
                crow::response r("Field 'us' must be a number");
                r.code = 400;
                return r;
            }
        } else {
            crow::response r("Provide exposure as ?us=... or JSON {\"us\": ...}");
            r.code = 400;
            return r;
        }

        if (exposure_us <= 0.0) {
            crow::response r("Exposure must be > 0 microseconds");
            r.code = 400;
            return r;
        }

        // --- Queue exposure command (non-blocking) ---
        // No detached thread needed unless your set_exposure_time blocks (it shouldn't).
        const bool ok = controller_.setExposureTimeOptical(exposure_us);
        // or: opticalCamera_->set_exposure_time(exposure_us);

        if (!ok) {
            crow::response r("Failed to queue exposure command (camera not running?)");
            r.code = 503; // service unavailable
            return r;
        }

        crow::json::wvalue resp;
        resp["status"] = "queued";
        resp["exposure_us"] = exposure_us;

        crow::response r(resp);
        r.code = 200;
        return r;
    });

    // Capture optical (placeholder)
    CROW_ROUTE(app, "/camera/opt/capture").methods("POST"_method)
    ([] {
        crow::response r("Optical image captured");
        r.code = 200;
        return r;
    });

    // Communication test
    CROW_ROUTE(app, "/communication/test").methods("POST"_method)
    ([] {
        crow::response r("API ONLINE");
        r.code = 200;
        return r;
    });

    CROW_ROUTE(app, "/camera/optical/last_frame").methods(crow::HTTPMethod::GET)
    ([this] {

        auto frame = controller_.getLastOpticalFrame();
        if (!frame.has_value()) {
            return crow::response(404, "No frame available");
        }

        crow::response r;
        r.code = 200;
        r.set_header("Content-Type", "image/jpeg");
        r.set_header("Access-Control-Allow-Origin", "*");

        r.body = std::string(
            reinterpret_cast<const char*>(frame->data()),
            frame->size()
        );

        return r;
    });

    CROW_ROUTE(app, "/system/status").methods(crow::HTTPMethod::GET)
    ([this] {

        const auto status = controller_.collectSystemStatus();

        crow::json::wvalue json;
        json["utc_now"]       = status.utc_iso;
        json["cpu_temp_c"]    = status.cpu_temp_c;
        json["cpu_load_pct"]  = status.cpu_load_pct;
        json["disk_used_gb"]  = status.disk_used_gb;
        json["disk_total_gb"] = status.disk_total_gb;
        json["uptime_s"]      = status.uptime_s;

        crow::response r;
        r.code = 200;
        r.set_header("Content-Type", "application/json");
        r.set_header("Access-Control-Allow-Origin", "*");
        r.body = json.dump();

        return r;
    });
}

} // namespace sober::communicator
