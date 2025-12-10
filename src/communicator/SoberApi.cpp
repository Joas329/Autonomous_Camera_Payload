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
    CROW_ROUTE(app, "/hello")([] {
        return "Hello from API!";
    });

    // Turn ON optical camera
    CROW_ROUTE(app, "/camera/opt/on").methods("POST"_method)
    ([this] {
        controller_.startOpticalCamera();
        return crow::response(200, "Optical camera ON");
    });

    // Turn OFF optical camera
    CROW_ROUTE(app, "/camera/opt/off").methods("POST"_method)
    ([this] {
        controller_.stopOpticalCamera();
        return crow::response(200, "Optical camera OFF");
    });
    CROW_ROUTE(app, "/camera/opt/capture").methods("POST"_method)
    ([] {
        return crow::response(200, "Optical image captured");
    });

    CROW_ROUTE(app, "/communication/test").methods("POST"_method)
    ([] {
        return crow::response("API ONLINE");
    });
}

}
