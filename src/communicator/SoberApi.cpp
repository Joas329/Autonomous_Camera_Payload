#define ASIO_STANDALONE
#include <crow.h>

#include "communicator/SoberApi.hpp"
#include "manager/SystemController.hpp"
#include <iostream>

namespace sober::communicator {

SoberApi::SoberApi(SystemController& controller)
    : controller_(controller)
{}

void SoberApi::registerRoutes(crow::SimpleApp& app)
{
    // Test endpoint
    CROW_ROUTE(app, "/hello")([] {
        crow::response r("Hello from API!");
        r.code = 200;
        r.add_header("Content-Type", "text/plain");
        r.add_header("Access-Control-Allow-Origin", "*");
        return r;
    });

    // Turn ON optical camera
    CROW_ROUTE(app, "/camera/opt/on").methods("POST"_method)
    ([this] {
        // Run camera start in background to avoid blocking UI
        std::thread([this] { controller_.startOpticalCamera(); }).detach();

        crow::response r("Optical camera ON");
        r.code = 200;
        r.add_header("Content-Type", "text/plain");
        r.add_header("Access-Control-Allow-Origin", "*");
        return r;
    });

    // Turn OFF optical camera
    CROW_ROUTE(app, "/camera/opt/off").methods("POST"_method)
    ([this] {
        std::thread([this] { controller_.stopOpticalCamera(); }).detach();

        crow::response r("Optical camera OFF");
        r.code = 200;
        r.add_header("Content-Type", "text/plain");
        r.add_header("Access-Control-Allow-Origin", "*");
        return r;
    });

    // Capture optical (placeholder)
    CROW_ROUTE(app, "/camera/opt/capture").methods("POST"_method)
    ([] {
        crow::response r("Optical image captured");
        r.code = 200;
        r.add_header("Content-Type", "text/plain");
        r.add_header("Access-Control-Allow-Origin", "*");
        return r;
    });

    // Communication test
    CROW_ROUTE(app, "/communication/test").methods("POST"_method)
    ([] {
        crow::response r("API ONLINE");
        r.code = 200;
        r.add_header("Content-Type", "text/plain");
        r.add_header("Access-Control-Allow-Origin", "*");
        return r;
    });
}

} // namespace sober::communicator