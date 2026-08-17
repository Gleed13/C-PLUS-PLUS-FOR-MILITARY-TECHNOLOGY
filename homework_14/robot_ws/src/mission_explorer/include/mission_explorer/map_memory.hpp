#pragma once

#include <cstdint>
#include <map>
#include <optional>

namespace mission_explorer {

// Координата клітинки. Окремий тип, щоб алгоритм не залежав від ROS-повідомлень.
struct P {
  int x = 0;
  int y = 0;
};

[[nodiscard]] inline bool operator==(const P lhs, const P rhs)
{
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

[[nodiscard]] inline bool operator<(const P lhs, const P rhs)
{
  return lhs.y != rhs.y ? lhs.y < rhs.y : lhs.x < rhs.x;
}

// Пам'ять про вже побачені клітинки та планувальник наступного кроку.
class MapMemory {
public:
  // Перезапис навмисний: клітинка змінює тип 'C' -> 'x' після знищення контакту.
  void observe(int x, int y, char type, int contact_id);
  void set_robot(P p);

  [[nodiscard]] P robot() const;
  [[nodiscard]] bool known(P p) const;
  [[nodiscard]] bool passable(P p) const;
  [[nodiscard]] bool is_frontier(P p) const;

  // BFS прохідними клітинками від робота. Повертає перший крок до найближчого
  // фронтиру або nullopt, коли досліджувати нічого.
  [[nodiscard]] std::optional<P> next_step() const;

private:
  std::map<P, char> type_;
  std::map<P, int> contact_id_;
  P robot_{};
};

// (0,-1)/(0,1)/(-1,0)/(1,0) -> MoveCommand::UP/DOWN/LEFT/RIGHT
[[nodiscard]] std::uint8_t delta_to_direction(P from, P to);

}  // namespace mission_explorer
