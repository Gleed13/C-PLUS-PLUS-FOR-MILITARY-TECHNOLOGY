#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

class BackgroundService {
public:
    BackgroundService() = default;
    virtual ~BackgroundService();

    virtual void start();
    virtual void stop();
    virtual void join();

protected:
    bool stop_requested() const;

    bool wait_for(std::chrono::milliseconds duration);

private:
    virtual void run() = 0;

private:
    std::thread worker_;
    std::atomic_bool stop_requested_{false};

    std::mutex mutex_;
    std::condition_variable cv_;
};