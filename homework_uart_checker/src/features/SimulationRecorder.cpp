#include <fstream>
#include <string>

#include <nlohmann/json.hpp>

#include "features/Logging.hpp"
#include "features/SimulationRecorder.hpp"

namespace {

nlohmann::json coordToJson(const Coord& coord)
{
    return {{"x", coord.x}, {"y", coord.y}};
}

nlohmann::json coordToJson(const std::optional<Coord>& coord)
{
    if (!coord.has_value()) {
        return nullptr; // JSON null
    }

    return coordToJson(*coord);
}

}    // namespace

bool SimulationRecorder::writeJson(const SimulationResult& result, const std::string& output_path) const
{
    std::ofstream output{output_path};
    if (!output.is_open()) {
        ERROR("Unable to open simulation output file: " << output_path);
        return false;
    }

    nlohmann::json json_output;
    json_output["totalSteps"] = result.steps.size();
    json_output["steps"] = nlohmann::json::array();

    for (const SimulationStep& simulation_step : result.steps) {
        nlohmann::json step;
        step["position"] = coordToJson(simulation_step.droneTelemetry.position);
        step["direction"] = simulation_step.droneTelemetry.direction;
        step["state"] = static_cast<int>(simulation_step.droneTelemetry.status);
        step["targetIndex"] = simulation_step.targetIndex;
        step["dropPoint"] = coordToJson(simulation_step.dropPoint);
        step["aimPoint"] = coordToJson(simulation_step.aimPoint);
        step["predictedTarget"] = coordToJson(simulation_step.predictedTarget);
        step["timeSecSinceStart"] = simulation_step.timeSecSinceStart;
        json_output["steps"].push_back(step);
    }

    output << json_output.dump(2);
    if (!output) {
        ERROR("Failed to write simulation output file: " << output_path);
        return false;
    }

    LOG("Simulation written to " << output_path << " with " << result.steps.size() << " steps");
    return true;
}
