struct InputData {
  float xd_, yd_, zd_;       // drone coordinates
  float xt_, yt_;            // target coordinates
  float attack_speed_;       // meters per second
  float acceleration_path_;  // meters
  char ammo_name_[50];       // NOLINT(cppcoreguidelines-avoid-c-arrays, modernize-avoid-c-arrays, cppcoreguidelines-avoid-magic-numbers,
                             // readability-magic-numbers): fixed-size C string buffer is required by the homework input format.
};

struct OutputData {
  bool is_too_close_to_target_;
  float interm_xd_;  // intermediate x-coordinate of the drone
  float interm_yd_;  // intermediate y-coordinate of the drone
  float fire_x_;     // x-coordinate where the projectile should be dropped
  float fire_y_;     // y-coordinate where the projectile should be dropped
};

struct Ammo {
  const char* ammo_type_;
  float mass_;
  float drag_;
  float lift_;
};

auto get_ammo_params(const char* name, float* mass, float* drag, float* lift) -> bool;

// NOLINTBEGIN(readability-identifier-length): short mathematical coordinate names, time variables and horizontal distance from the task
// statement.
auto calculate_free_fall_time(float zd, float attack_speed, float mass, float drag, float lift, float* t) -> bool;

auto calculate_horizontal_distance(float t, float attack_speed, float mass, float drag, float lift, float* h) -> bool;

auto calculate_output_data(float xd, float yd, float xt, float yt, float acceleration_path, float h, OutputData* output_data) -> bool;
// NOLINTEND(readability-identifier-length)

auto compute_drop_solution(const InputData* input, OutputData* output) -> int;