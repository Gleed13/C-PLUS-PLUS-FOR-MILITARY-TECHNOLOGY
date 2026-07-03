#include <string>

#include "strategies/MockCheckerController.hpp"

MockCheckerController::~MockCheckerController() {
    cleanup();
}

MockCheckerController::MockCheckerController(const std::string& chip_path, const int start_line, const int drop_line) :
    chip_path_(chip_path), start_line_(start_line), drop_line_(drop_line) {
}

void MockCheckerController::init() {
}

void MockCheckerController::start() {
}

void MockCheckerController::drop() {
}

void MockCheckerController::cleanup() {
}