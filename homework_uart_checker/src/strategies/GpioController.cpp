#include <string>
#include <thread>

#include "strategies/GpioController.hpp"
#include "features/Logging.hpp"

GpioController::~GpioController() {
    try {
        cleanup();
    } catch (...) {
        // Destructors should not throw.
    }
}

GpioController::GpioController(const std::string& chip_path, const unsigned start_line, const unsigned drop_line) :
    start_line_(start_line), drop_line_(drop_line), chip_path_(chip_path) {
}

void GpioController::init() {
    gpiod::chip chip{chip_path_};

    gpiod::line_settings output_settings;
    output_settings
        .set_direction(gpiod::line::direction::OUTPUT)
        .set_output_value(gpiod::line::value::INACTIVE);

    request_.emplace(
        chip.prepare_request()
            .set_consumer("drone")
            .add_line_settings({start_line_, drop_line_}, output_settings)
            .do_request()
        );
}

void GpioController::start() {
    LOG("Starting the drone by setting the start GPIO line to ACTIVE");
    ensureInitialized();
    request_->set_value(gpiod::line::offset{start_line_}, gpiod::line::value::ACTIVE);
}

void GpioController::drop() {
    LOG("Dropping the drone by setting the drop GPIO line to ACTIVE");
    ensureInitialized();

    using namespace std::chrono_literals;
    request_->set_value(gpiod::line::offset{drop_line_}, gpiod::line::value::ACTIVE);
    std::this_thread::sleep_for(80ms);
    request_->set_value(gpiod::line::offset{drop_line_}, gpiod::line::value::INACTIVE);
}

void GpioController::cleanup() {
    if (!request_.has_value()) {
        return;
    }

    request_->set_value(gpiod::line::offset{start_line_}, gpiod::line::value::INACTIVE);
    request_->set_value(gpiod::line::offset{drop_line_}, gpiod::line::value::INACTIVE);

    request_.reset(); // releases GPIO lines
}

void GpioController::ensureInitialized() const {
    if (!request_.has_value()) {
        throw std::runtime_error("GPIO request not initialized");
    }
}