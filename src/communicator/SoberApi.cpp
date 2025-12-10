#define ASIO_STANDALONE
#include <crow.h>

#include "communicator/SoberApi.hpp"
#include <iostream>

namespace sober::communicator {

void SoberApi::registerRoutes(crow::SimpleApp& app)
{
    CROW_ROUTE(app, "/hello")([] {
        return "Hello from API!";
    });

    CROW_ROUTE(app, "/camera/opt/on").methods("POST"_method)
    ([] {
        return crow::response(200, "Optical camera ON");
    });

    CROW_ROUTE(app, "/camera/opt/capture").methods("POST"_method)
    ([] {
        return crow::response(200, "Optical image captured");
    });

    CROW_ROUTE(app, "/camera/ir/on").methods("POST"_method)
    ([] {
        return crow::response("IR Camera ON");
    });
}

}
