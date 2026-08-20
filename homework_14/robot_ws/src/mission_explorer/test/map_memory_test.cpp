#include <gtest/gtest.h>

#include "mission_explorer/map_memory.hpp"
#include "underground_world/msg/move_command.hpp"

namespace {

using mission_explorer::MapMemory;
using mission_explorer::P;
using underground_world::msg::MoveCommand;

// Прямий коридор "#S..#" у рядку y=1, оточений стінами.
MapMemory make_corridor(const int known_until_x)
{
  MapMemory map;
  for (int x = 0; x <= known_until_x; ++x) {
    map.observe(x, 0, '#', 0);
    map.observe(x, 2, '#', 0);
    map.observe(x, 1, x == 0 ? '#' : (x == 1 ? 'S' : '.'), 0);
  }
  map.set_robot({1, 1});
  return map;
}

TEST(MapMemoryTest, StepsTowardsFrontierInCorridor)
{
  const auto map = make_corridor(3);

  ASSERT_TRUE(map.is_frontier({3, 1}));
  const auto step = map.next_step();
  ASSERT_TRUE(step.has_value());
  EXPECT_EQ(*step, (P{2, 1}));
  EXPECT_EQ(mission_explorer::delta_to_direction({1, 1}, *step), MoveCommand::RIGHT);
}

TEST(MapMemoryTest, FullyKnownRegionHasNoNextStep)
{
  MapMemory map;
  for (int x = 0; x <= 3; ++x) {
    for (int y = 0; y <= 2; ++y) {
      const bool wall = (y != 1) || (x == 0) || (x == 3);
      map.observe(x, y, wall ? '#' : '.', 0);
    }
  }
  map.set_robot({1, 1});

  EXPECT_FALSE(map.next_step().has_value());
}

TEST(MapMemoryTest, ProcessedContactStaysPassable)
{
  MapMemory map;
  map.observe(2, 1, 'C', 7);
  EXPECT_TRUE(map.passable({2, 1}));

  map.observe(2, 1, 'x', 7);
  EXPECT_TRUE(map.passable({2, 1}));
  EXPECT_FALSE(map.passable({9, 9}));
}

TEST(MapMemoryTest, RobotOwnCellNeverStallsPlanner)
{
  MapMemory map;
  // Робот стоїть на клітинці з невідомим сусідом - вона фронтир, але не ціль.
  map.observe(1, 1, 'S', 0);
  map.observe(2, 1, '.', 0);
  map.set_robot({1, 1});

  ASSERT_TRUE(map.is_frontier({1, 1}));
  const auto step = map.next_step();
  ASSERT_TRUE(step.has_value());
  EXPECT_EQ(*step, (P{2, 1}));
}

TEST(MapMemoryTest, DeltaToDirectionCoversAllAxes)
{
  EXPECT_EQ(mission_explorer::delta_to_direction({1, 1}, {1, 0}), MoveCommand::UP);
  EXPECT_EQ(mission_explorer::delta_to_direction({1, 1}, {1, 2}), MoveCommand::DOWN);
  EXPECT_EQ(mission_explorer::delta_to_direction({1, 1}, {0, 1}), MoveCommand::LEFT);
  EXPECT_EQ(mission_explorer::delta_to_direction({1, 1}, {2, 1}), MoveCommand::RIGHT);
  EXPECT_THROW([[maybe_unused]] const auto direction = mission_explorer::delta_to_direction({1, 1}, {3, 3}),
               std::invalid_argument);
}

}  // namespace
