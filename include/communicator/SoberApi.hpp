#pragma once

namespace crow {
    template <typename... Middlewares>
    class Crow;

    using SimpleApp = Crow<>;
}

class SystemController;

namespace sober::communicator {

class SoberApi {
public:
    explicit SoberApi(::SystemController& controller);

    void registerRoutes(crow::SimpleApp& app);

private:
    ::SystemController& controller_;
};

} // namespace sober::communicator