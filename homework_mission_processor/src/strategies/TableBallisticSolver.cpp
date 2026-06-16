#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "features/Logging.hpp"
#include "strategies/TableBallisticSolver.hpp"

namespace {

constexpr std::size_t kDimensionCount = 5;
constexpr std::size_t kCornerCount = 1U << kDimensionCount;    // 32 neighboring grid points

bool readAxis(std::ifstream& input, std::size_t size, std::vector<float>& axis)
{
    if (size < 2) {
        return false;
    }

    axis.resize(size);
    for (float& value : axis) {
        if (!(input >> value) || !std::isfinite(value)) {
            return false;
        }
    }

    return std::adjacent_find(axis.begin(), axis.end(), std::greater_equal<>()) == axis.end();
}

bool checkedMultiply(std::size_t factor, std::size_t& product)
{
    if (factor != 0 && product > std::numeric_limits<std::size_t>::max() / factor) {
        return false;
    }

    product *= factor;
    return true;
}

}    // namespace

TableBallisticSolver::TableBallisticSolver(const std::string& table_path)
    : loaded_(load(table_path))
{
}

bool TableBallisticSolver::isLoaded() const
{
    return loaded_;
}

std::optional<BallisticSolution> TableBallisticSolver::solve(const DroneConfig& drone_config,
                                                             const Coord& target_position,
                                                             const Ammo& ammo)
{
    if (!loaded_) {
        ERROR("Ballistic table is not loaded");
        return std::nullopt;
    }

    const auto result = lookup(drone_config.altitude, drone_config.attackSpeed, ammo.mass, ammo.drag, ammo.lift);

    if (!result.has_value() || result->fallTime <= 0.0F || result->horizontalDistance <= 0.0F) {
        ERROR("Failed to interpolate ballistic table values");
        return std::nullopt;
    }

    DropPoint drop_point;
    if (!tryCalculateDropPoint(drone_config.startPos, target_position, drone_config.accelPath, result->horizontalDistance, drop_point)) {
        return std::nullopt;
    }

    return BallisticSolution{.dropPoint = drop_point, .fallTime = result->fallTime, .horizontalDistance = result->horizontalDistance};
}

bool TableBallisticSolver::load(const std::string& table_path)
{
    std::ifstream input{table_path};
    if (!input.is_open()) {
        ERROR("Unable to open ballistic table: " << table_path);
        return false;
    }

    std::array<std::size_t, kDimensionCount> dimensions{};
    for (std::size_t& dimension : dimensions) {
        if (!(input >> dimension)) {
            ERROR("Invalid ballistic table dimensions: " << table_path);
            return false;
        }
    }

    std::size_t total_size = 1;
    for (const std::size_t dimension : dimensions) {
        if (dimension < 2 || !checkedMultiply(dimension, total_size)) {
            ERROR("Invalid ballistic table dimensions: " << table_path);
            return false;
        }
    }

    if (!readAxis(input, dimensions[0], altitude_axis_) || !readAxis(input, dimensions[1], speed_axis_) ||
        !readAxis(input, dimensions[2], mass_axis_) || !readAxis(input, dimensions[3], drag_axis_) ||
        !readAxis(input, dimensions[4], lift_axis_)) {
        ERROR("Invalid ballistic table axes: " << table_path);
        return false;
    }

    data_.resize(total_size);
    for (TableResult& result : data_) {
        if (!(input >> result.fallTime >> result.horizontalDistance) || !std::isfinite(result.fallTime) ||
            !std::isfinite(result.horizontalDistance) || result.fallTime <= 0.0F || result.horizontalDistance <= 0.0F) {
            ERROR("Invalid ballistic table data: " << table_path);
            return false;
        }
    }

    std::string trailing_data;
    if (input >> trailing_data) {
        ERROR("Unexpected trailing data in ballistic table: " << table_path);
        return false;
    }

    LOG("Ballistic table loaded from " << table_path << " with " << data_.size() << " entries");
    return true;
}

std::optional<TableBallisticSolver::TableResult> TableBallisticSolver::lookup(
    float altitude, float speed, float mass, float drag, float lift) const
{
    const std::array<float, kDimensionCount> query{altitude, speed, mass, drag, lift};
    const std::array<const std::vector<float>*, kDimensionCount> axes{&altitude_axis_, &speed_axis_, &mass_axis_, &drag_axis_, &lift_axis_};
    std::array<Interpolation, kDimensionCount> interpolation{};
    for (std::size_t dimension = 0; dimension < kDimensionCount; ++dimension) {
        const auto axis_interpolation = findInterpolation(query.at(dimension), *axes.at(dimension));
        if (!axis_interpolation.has_value()) {
            return std::nullopt;
        }
        interpolation.at(dimension) = axis_interpolation.value();
    }

    std::array<TableResult, kCornerCount> values{};
    for (std::size_t corner = 0; corner < kCornerCount; ++corner) {
        std::array<std::size_t, kDimensionCount> indices{};
        for (std::size_t dimension = 0; dimension < kDimensionCount; ++dimension) {
            indices.at(dimension) = interpolation.at(dimension).lowerIndex + ((corner >> dimension) & 1U);
        }

        values.at(corner) = data_.at(index(indices.at(0), indices.at(1), indices.at(2), indices.at(3), indices.at(4)));
    }

    std::size_t active_values = kCornerCount;
    for (std::size_t dimension = 0; dimension < kDimensionCount; ++dimension) {
        for (std::size_t value_index = 0; value_index < active_values; value_index += 2) {
            values.at(value_index / 2) =
                interpolate(values.at(value_index), values.at(value_index + 1), interpolation.at(dimension).fraction);
        }
        active_values /= 2;
    }

    return values.front();
}

std::size_t TableBallisticSolver::index(
    std::size_t altitude_index, std::size_t speed_index, std::size_t mass_index, std::size_t drag_index, std::size_t lift_index) const
{
    return ((((altitude_index * speed_axis_.size() + speed_index) * mass_axis_.size() + mass_index) * drag_axis_.size() + drag_index) *
                lift_axis_.size() +
            lift_index);
}

std::optional<TableBallisticSolver::Interpolation> TableBallisticSolver::findInterpolation(float value, const std::vector<float>& axis)
{
    if (!std::isfinite(value) || axis.size() < 2) {
        return std::nullopt;
    }

    if (value <= axis.front()) {
        return Interpolation{0, 0.0F};
    }
    if (value >= axis.back()) {
        return Interpolation{axis.size() - 2, 1.0F};
    }

    const auto upper = std::lower_bound(axis.begin(), axis.end(), value);
    const auto lower_index = static_cast<std::size_t>(std::distance(axis.begin(), upper) - 1);
    const float fraction = (value - axis[lower_index]) / (axis[lower_index + 1] - axis[lower_index]);
    return Interpolation{lower_index, fraction};
}

TableBallisticSolver::TableResult TableBallisticSolver::interpolate(const TableResult& lower, const TableResult& upper, float fraction)
{
    return {.fallTime = std::lerp(lower.fallTime, upper.fallTime, fraction),
            .horizontalDistance = std::lerp(lower.horizontalDistance, upper.horizontalDistance, fraction)};
}

bool TableBallisticSolver::tryCalculateDropPoint(
    const Coord& drone_position, const Coord& target_position, float acceleration_path, float horizontal_distance, DropPoint& drop_point)
{
    const float distance_to_target = std::hypot(target_position.x - drone_position.x, target_position.y - drone_position.y);
    if (distance_to_target <= 0.0F) {
        ERROR("Non-positive distance to target: " << distance_to_target << " meters");
        return false;
    }

    const Coord target_offset = target_position - drone_position;
    const float ratio = (distance_to_target - horizontal_distance) / distance_to_target;
    if (horizontal_distance + acceleration_path > distance_to_target) {
        drop_point.intermPoint = target_position - target_offset * ((horizontal_distance + acceleration_path) / distance_to_target);
    }
    else {
        drop_point.intermPoint.reset();
    }
    drop_point.firePoint = target_position - target_offset * ratio;
    return true;
}
