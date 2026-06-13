#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "interfaces/IBallisticSolver.hpp"

class TableBallisticSolver final : public IBallisticSolver {
public:
    TableBallisticSolver(const std::string& table_path);

    bool isLoaded() const;

    std::optional<BallisticSolution> solve(
        const DroneConfig& drone_config,
        const Coord& target_position,
        const Ammo& ammo) override;

private:
    struct TableResult
    {
        float fallTime = 0.0F;
        float horizontalDistance = 0.0F;
    };

    struct Interpolation
    {
        std::size_t lowerIndex = 0;
        float fraction = 0.0F;
    };

    std::vector<float> altitude_axis_;
    std::vector<float> speed_axis_;
    std::vector<float> mass_axis_;
    std::vector<float> drag_axis_;
    std::vector<float> lift_axis_;
    std::vector<TableResult> data_;
    bool loaded_ = false;

    bool load(const std::string& table_path);
    std::optional<TableResult> lookup(
        float altitude,
        float speed,
        float mass,
        float drag,
        float lift) const;
    std::size_t index(
        std::size_t altitude_index,
        std::size_t speed_index,
        std::size_t mass_index,
        std::size_t drag_index,
        std::size_t lift_index) const;

    static std::optional<Interpolation> findInterpolation(
        float value,
        const std::vector<float>& axis);
    static TableResult interpolate(
        const TableResult& lower,
        const TableResult& upper,
        float fraction);
    static bool tryCalculateDropPoint(
        const Coord& drone_position,
        const Coord& target_position,
        float acceleration_path,
        float horizontal_distance,
        DropPoint& drop_point);
};
