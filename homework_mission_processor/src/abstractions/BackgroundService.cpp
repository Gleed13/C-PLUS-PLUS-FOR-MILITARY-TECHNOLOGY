#include "abstractions/BackgroundService.hpp"

BackgroundService::~BackgroundService() {
    stop();
}

void BackgroundService::start() {
    stop_requested_ = false;

    worker_ = std::thread([this] {
        run();
    });
}

void BackgroundService::stop() {
    stop_requested_ = true;
    cv_.notify_all();

    if (worker_.joinable()) {
        worker_.join();
    }
}

void BackgroundService::join() {
    if (worker_.joinable()) {
        worker_.join();
    }
}

bool BackgroundService::stop_requested() const {
    return stop_requested_;
}

bool BackgroundService::wait_for(std::chrono::milliseconds duration) {
    std::unique_lock<std::mutex> lock(mutex_);

    return cv_.wait_for(lock, duration, [this] {
        return stop_requested();
    });
}