#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <mutex>

#include "abstractions/BackgroundService.hpp"
#include "drone_link.h"
#include "features/Logging.hpp"
#include "features/UartBridge.hpp"

UartBridge::UartBridge(char* device_name) : device_name_(device_name) {
}

std::optional<dlink::Telemetry> UartBridge::getTelemetry() {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    return telemetry_;
}
std::vector<std::optional<dlink::TargetPos>> UartBridge::getTargetPositions() {
    std::lock_guard<std::mutex> lock(target_positions_mutex_);
    return target_positions_;
}
std::optional<dlink::AmmoCfg> UartBridge::getAmmoConfig() {
    std::lock_guard<std::mutex> lock(ammo_config_mutex_);
    return ammo_config_;
}
std::optional<dlink::Result>UartBridge::getResult() {
    std::lock_guard<std::mutex> lock(result_mutex_);
    return result_;
}

void UartBridge::sendControl(const float accel, const float turn_rate) {
    dlink::Control control{ accel, turn_rate }; // обидва float у [-1..1]
    uint8_t out[64];
    size_t m = encode(dlink::PKT_CONTROL, &control, sizeof control, out);
    write(file_descriptor_, out, m);
}

void UartBridge::start() {
    file_descriptor_ = open(device_name_, O_RDWR | O_NOCTTY | O_NONBLOCK); // "/tmp/ttyA" (sim) або "/dev/ttyAMA1" (плата)
    if (file_descriptor_ < 0) {
        perror("open");
        ERROR(std::string("could not open uart with device name ") + device_name_);
        return;
    }
    termios tio{};
    tcgetattr(file_descriptor_, &tio);
    cfmakeraw(&tio);                        // 8N1, без обробки символів
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);             // швидкість з обох боків однакова!
    tio.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(file_descriptor_, TCSANOW, &tio);

    BackgroundService::start();
}


void UartBridge::initializeNumberfTargets(uint8_t number_of_targets) {
    std::lock_guard<std::mutex> lock(target_positions_mutex_);
    target_positions_.resize(number_of_targets);
}
void UartBridge::updateTelemetry(dlink::Telemetry telemetry) {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    telemetry_ = telemetry;
}
void UartBridge::updateTargetPosition(dlink::TargetPos target_position) {
    std::lock_guard<std::mutex> lock(target_positions_mutex_);
    target_positions_[target_position.id] = target_position;
}
void UartBridge::setAmmoConfig(dlink::AmmoCfg ammo_config) {
    std::lock_guard<std::mutex> lock(ammo_config_mutex_);
    ammo_config_ = ammo_config;
}
void UartBridge::setResult(dlink::Result result) {
    std::lock_guard<std::mutex> lock(result_mutex_);
    result_ = result;
}

void UartBridge::handleCompletedPacket(uint8_t type, uint8_t payload[260]) {
    if (type == dlink::PKT_TELEMETRY) {
        dlink::Telemetry telemetry;
        memcpy(&telemetry, payload, sizeof telemetry);
        updateTelemetry(telemetry);
        return;
    }

    if (type == dlink::PKT_TARGET) {
        if (!config_inited_) {
            WARNING("target position data received before receiving ammo config -> will be skipped");
            return;
        }
        dlink::TargetPos targetPos;
        memcpy(&targetPos, payload, sizeof targetPos);
        updateTargetPosition(targetPos);
        return;
    }

    if (type == dlink::PKT_AMMO) {
        dlink::AmmoCfg ammoCfg;
        memcpy(&ammoCfg, payload, sizeof ammoCfg);
        setAmmoConfig(ammoCfg);
        if (config_inited_) {
            WARNING("another PKT_AMMO was received after first one");
        } else {
            initializeNumberfTargets(ammoCfg.nTargets);
        }
        config_inited_ = true;
        return;
    }

    if (type == dlink::PKT_RESULT) {
        if (result_.has_value()) {
            WARNING("another PKT_RESULT was received after first one");
        }
        dlink::Result result;
        memcpy(&result, payload, sizeof result);
        setResult(result);
        return;
    }

    ERROR(std::string("Unknown packet type was received of type ") + std::to_string(static_cast<int>(type)));
}

void UartBridge::run() {
    dlink::Parser parser; // тримає стан між викликами
    uint8_t buffer[256];
    uint8_t type, len, payload[260];

    // const auto interval = std::chrono::milliseconds(1);
    while (!stop_requested()) {
        int number_of_bytes = read(file_descriptor_, buffer, sizeof(buffer));
        for (int i = 0; i < number_of_bytes; ++i) {
            bool packet_completed = parser.feed(buffer[i], type, payload, len);
            if (packet_completed) {
                handleCompletedPacket(type, payload);
            }
        }

        // bool stopped = wait_for(interval);
        // if (stopped) {
        //     break;
        // }
    }
}