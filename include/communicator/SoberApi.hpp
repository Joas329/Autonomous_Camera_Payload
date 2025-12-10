#pragma once

// Correct forward declaration
namespace crow {
    template <typename... Middlewares>
    class Crow;

    using SimpleApp = Crow<>;
}

namespace sober::communicator {

class SoberApi {
public:
    SoberApi() = default;

    void registerRoutes(crow::SimpleApp& app);
};

}
