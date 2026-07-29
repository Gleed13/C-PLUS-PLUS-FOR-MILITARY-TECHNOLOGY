#include "c2_controller.hpp"
#include "fc_link.hpp"     // MAVSDK обгортка, API описано у fc_link.hpp
#include "udp_socket.hpp"  // UDP прийом, API описано у udp_socket.hpp

#include <nlohmann/json.hpp>  // Розбiр JSON з точками маршруту вiд auto_stub

#include <fstream>
#include <iostream>
#include <string>

static constexpr uint16_t STUB_PORT = 14560;

const char* state_name(C2State s) {
    switch (s) {
        case C2State::DISARMED:     return "DISARMED";
        case C2State::ARMED_HOLD:   return "ARMED_HOLD";
        case C2State::ARMED_GUIDED: return "ARMED_GUIDED";
        case C2State::ARMED_MANUAL: return "ARMED_MANUAL";
    }
    return "UNKNOWN";
}

struct C2Controller::Impl {
    C2State state = C2State::DISARMED;
    FcLink fc;
    UdpSocket udp{STUB_PORT};
    std::ofstream log{"/var/log/c2/c2.log", std::ios::app};
    bool healthy_marked = false;

    Impl(uint16_t fc_port) : fc(fc_port) {}

    void logMsg(const std::string& msg) {
        std::cout << msg << std::endl;
        log << msg << std::endl;
    }

    void transition(C2State next) {
        if (next != state) {
            logMsg(std::string("[C2] state: ") + state_name(state) +
                   " -> " + state_name(next));
            state = next;
            if (next == C2State::DISARMED) {
                logMsg("[C2] blocked: waypoint in DISARMED");
            } else if (next == C2State::ARMED_HOLD) {
                logMsg("[C2] blocked: waypoint in ARMED_HOLD");
            } else if (next == C2State::ARMED_MANUAL) {
                logMsg("[C2] blocked: waypoint in ARMED_MANUAL");
            }
        }
    }
};

C2Controller::C2Controller(uint16_t fc_port)
    : impl_(std::make_unique<Impl>(fc_port))
{
}

C2Controller::~C2Controller() = default;

void C2Controller::tick() {
    // healthcheck: створити /tmp/c2_healthy пiсля першого HEARTBEAT
    if (!impl_->healthy_marked && impl_->fc.is_connected()) {
        std::ofstream("/tmp/c2_healthy").close();
        impl_->healthy_marked = true;
    }
    if (impl_->fc.is_armed()) {
        if (impl_->fc.flight_mode() == FcLink::FlightMode::Guided) {
            impl_->transition(C2State::ARMED_GUIDED);
        } else if (impl_->fc.flight_mode() == FcLink::FlightMode::Hold) {
            impl_->transition(C2State::ARMED_HOLD);
        } else if (impl_->fc.flight_mode() == FcLink::FlightMode::Manual) {
            impl_->transition(C2State::ARMED_MANUAL);
        }
    } else {
        impl_->transition(C2State::DISARMED);
    }

    // читання точки маршруту вiд auto_stub
    char buf[1024];
    ssize_t n = impl_->udp.recv(buf, sizeof(buf));
    if (n > 0) {
        std::string msg(buf, static_cast<std::size_t>(n));

        if (impl_->state == C2State::ARMED_GUIDED) {
            try {
                auto j = nlohmann::json::parse(msg);
                double north = j.at("north_m").get<double>();
                double east = j.at("east_m").get<double>();

                impl_->fc.go_to_ned(north, east);

                impl_->logMsg("[C2] fwd: north=" + std::to_string(north) +
                              " east=" + std::to_string(east));
            }
            catch (const std::exception& e) {
                std::cout << "[C2] bad waypoint json: " << e.what() << std::endl;
                impl_->log << "[C2] bad waypoint json: " << e.what() << std::endl;
            }
        }
        else {
            impl_->logMsg(std::string("[C2] blocked: waypoint in ") + state_name(impl_->state));
        }
    }
}

C2State C2Controller::current_state() const {
    return impl_->state;
}
