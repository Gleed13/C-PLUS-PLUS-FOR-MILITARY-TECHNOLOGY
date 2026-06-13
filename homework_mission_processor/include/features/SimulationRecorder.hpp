#pragma once

#include <string>

#include "models/SimulationResult.hpp"

class SimulationRecorder final {
public:
    bool writeJson(const SimulationResult& result, const std::string& output_path) const;
};
