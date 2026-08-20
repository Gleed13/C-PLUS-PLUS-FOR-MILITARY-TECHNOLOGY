#include "mission_explorer/map_memory.hpp"

#include <array>
#include <deque>
#include <stdexcept>

#include "underground_world/msg/move_command.hpp"

namespace mission_explorer {
namespace {

using underground_world::msg::MoveCommand;

constexpr char kWall = '#';

constexpr std::array<P, 4> kNeighbourDeltas{P{0, -1}, P{0, 1}, P{-1, 0}, P{1, 0}};

}  // namespace

void MapMemory::observe(const int x, const int y, const char type, const int contact_id)
{
  const P position{x, y};
  type_[position] = type;
  contact_id_[position] = contact_id;
}

void MapMemory::set_robot(const P p)
{
  robot_ = p;
}

P MapMemory::robot() const
{
  return robot_;
}

bool MapMemory::known(const P p) const
{
  return type_.find(p) != type_.end();
}

bool MapMemory::passable(const P p) const
{
  const auto iter = type_.find(p);
  return iter != type_.end() && iter->second != kWall;
}

bool MapMemory::is_frontier(const P p) const
{
  if (!passable(p)) {
    return false;
  }

  // Рух 4-зв'язний, тому фронтир рахуємо теж по 4 сусідах.
  for (const auto delta : kNeighbourDeltas) {
    if (!known({p.x + delta.x, p.y + delta.y})) {
      return true;
    }
  }
  return false;
}

std::optional<P> MapMemory::next_step() const
{
  if (!passable(robot_)) {
    return std::nullopt;
  }

  // first_step_[c] - перший хід від робота, який веде у клітинку c.
  std::map<P, P> first_step;
  std::deque<P> queue;
  std::map<P, bool> visited;

  visited[robot_] = true;
  queue.push_back(robot_);

  while (!queue.empty()) {
    const auto current = queue.front();
    queue.pop_front();

    // Власна клітинка робота не є ціллю - інакше він стоятиме на місці вічно.
    if (!(current == robot_) && is_frontier(current)) {
      return first_step.at(current);
    }

    for (const auto delta : kNeighbourDeltas) {
      const P next{current.x + delta.x, current.y + delta.y};
      if (!passable(next) || visited[next]) {
        continue;
      }
      visited[next] = true;
      first_step[next] = (current == robot_) ? next : first_step.at(current);
      queue.push_back(next);
    }
  }

  return std::nullopt;
}

std::uint8_t delta_to_direction(const P from, const P to)
{
  const int dx = to.x - from.x;
  const int dy = to.y - from.y;

  if (dx == 0 && dy == -1) {
    return MoveCommand::UP;
  }
  if (dx == 0 && dy == 1) {
    return MoveCommand::DOWN;
  }
  if (dx == -1 && dy == 0) {
    return MoveCommand::LEFT;
  }
  if (dx == 1 && dy == 0) {
    return MoveCommand::RIGHT;
  }

  throw std::invalid_argument("delta_to_direction: cells are not orthogonal neighbours");
}

}  // namespace mission_explorer
