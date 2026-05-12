#define USE_MATH_DEFINES

#include <iostream>
#include <cmath>
#include <cstring>

#include "ballistics.hpp"

const float kGravity = 9.81F;

// NOLINTBEGIN(cppcoreguidelines-avoid-c-arrays, modernize-avoid-c-arrays, cppcoreguidelines-avoid-magic-numbers): fixed-size compile-time
// ammo table is intentional for this homework.
const Ammo kAmmoTable[] = {{"VOG-17", 0.35F, 0.07F, 0.0F},
                           {"M67", 0.60F, 0.10F, 0.0F},
                           {"RKG-3", 1.20F, 0.10F, 0.0F},
                           {"GLIDING-VOG", 0.45F, 0.10F, 1.0F},
                           {"GLIDING-RKG", 1.40F, 0.10F, 1.0F}};
// NOLINTEND(cppcoreguidelines-avoid-c-arrays, modernize-avoid-c-arrays, cppcoreguidelines-avoid-magic-numbers)

// NOLINTBEGIN(bugprone-easily-swappable-parameters): output parameters are simple float values required by current function design.
auto get_ammo_params(const char* name, float* mass, float* drag, float* lift) -> bool
// NOLINTEND(bugprone-easily-swappable-parameters): output parameters are simple float values required by current function design.
{
  int count = sizeof(kAmmoTable) / sizeof(kAmmoTable[0]);
  for (int i = 0; i < count; i++) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index): iteration over fixed-size C array is intentional here.
    if (strcmp(kAmmoTable[i].ammo_type_, name) == 0) {
      *mass = kAmmoTable[i].mass_;
      *drag = kAmmoTable[i].drag_;
      *lift = kAmmoTable[i].lift_;
      return true;
    }
    // NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)
  }

  return false;
}

// NOLINTBEGIN(bugprone-easily-swappable-parameters, readability-identifier-length): parameters follow the mathematical model used in the
// task.
auto calculate_free_fall_time(float zd, float attack_speed, float mass, float drag, float lift, float* t) -> bool
// NOLINTEND(bugprone-easily-swappable-parameters, readability-identifier-length)
{
  // NOLINTBEGIN(readability-identifier-length, cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers): coefficients and variable
  // names follow the mathematical model used in the task.
  float a = drag * kGravity * mass - 2 * drag * drag * lift * attack_speed;
  float b = -3 * kGravity * mass * mass + 3 * drag * lift * mass * attack_speed;
  float c = 6 * mass * mass * zd;

  std::cout << "Debug: Coefficients of cubic equation: a=" << a << ", b=" << b << ", c=" << c << '\n';

  float p = -b * b / (3 * a * a);
  float q = (2 * b * b * b) / (27 * a * a * a) + c / a;
  // NOLINTEND(readability-identifier-length, cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)

  std::cout << "Debug: Coefficients for depressed cubic: p=" << p << ", q=" << q << '\n';
  if (p >= 0) {
    std::cout << "Error: p >= 0: No real roots" << '\n';
    return false;
  }

  float phi_arg = 3 * q / (2 * p) * std::sqrt(-3 / p);
  if (phi_arg < -1.0F || phi_arg > 1.0F) {
    std::cout << "Error: acos argument is out of bounds: " << phi_arg << '\n';
    return false;
  }

  float phi = std::acos(phi_arg);
  std::cout << "Debug: phi (angle for cosine solution): " << phi << " radians" << '\n';

  // NOLINTBEGIN(bugprone-narrowing-conversions, cppcoreguidelines-narrowing-conversions): result is intentionally stored as float according
  // to task structures.
  *t = 2 * std::sqrt(-p / 3) * cos((phi + 4 * M_PI) / 3) - b / (3 * a);
  // NOLINTEND(bugprone-narrowing-conversions, cppcoreguidelines-narrowing-conversions)
  std::cout << "Debug: Projectile`s free-fall time: " << *t << " seconds" << '\n';
  if (*t <= 0) {
    std::cout << "Error: Non-positive time to target: " << *t << " seconds" << '\n';
    return false;
  }

  return true;
}

// NOLINTBEGIN(readability-identifier-length): short mathematical time and horizontal distance variables.
auto calculate_horizontal_distance(float t, float attack_speed, float mass, float drag, float lift, float* h) -> bool
// NOLINTEND(readability-identifier-length)
{
  // NOLINTBEGIN(readability-identifier-length, cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers): terms in the series
  // expansion of the horizontal distance follow the mathematical model used in the task.
  float t1 = attack_speed * t;
  float t2 = -t * t * drag * attack_speed / (2 * mass);
  float t3 = t * t * t * (6 * drag * kGravity * lift * mass - 6 * drag * drag * (lift * lift - 1) * attack_speed) / (36 * mass * mass);
  float t4 = t * t * t * t *
             (-6 * drag * drag * kGravity * lift * (1 + lift * lift + lift * lift * lift * lift) * mass +
              3 * drag * drag * drag * lift * lift * (1 + lift * lift) * attack_speed +
              6 * drag * drag * drag * lift * lift * lift * lift * (1 + lift * lift) * attack_speed) /
             (36 * (1 + lift * lift) * (1 + lift * lift) * mass * mass * mass);
  float t5 = t * t * t * t * t *
             (3 * drag * drag * drag * kGravity * lift * lift * lift * mass -
              3 * drag * drag * drag * drag * lift * lift * (1 + lift * lift) * attack_speed) /
             (36 * (1 + lift * lift) * mass * mass * mass * mass);
  // NOLINTEND(readability-identifier-length, cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)

  *h = t1 + t2 + t3 + t4 + t5;
  std::cout << "Debug: Projectile trajectory horizontal distance: " << *h << " meters" << '\n';
  if (*h <= 0) {
    std::cout << "Error: Non-positive projectile trajectory horizontal distance: " << *h << " meters" << '\n';
    return false;
  }

  return true;
}

// NOLINTBEGIN(readability-identifier-length): coordinate names from the task statement.
auto calculate_output_data(float xd, float yd, float xt, float yt, float acceleration_path, float h, OutputData* output_data) -> bool
// NOLINTEND(readability-identifier-length)
{
  float distance_to_target = std::sqrt((xt - xd) * (xt - xd) + (yt - yd) * (yt - yd));
  std::cout << "Debug: Distance to target: " << distance_to_target << " meters" << '\n';
  if (distance_to_target <= 0) {
    std::cout << "Error: Non-positive distance to target: " << distance_to_target << " meters" << '\n';
    return false;
  }

  float ratio = (distance_to_target - h) / distance_to_target;
  output_data->is_too_close_to_target_ = h + acceleration_path > distance_to_target;
  if (output_data->is_too_close_to_target_) {
    output_data->interm_xd_ = xt - (xt - xd) * (h + acceleration_path) / distance_to_target;
    output_data->interm_yd_ = yt - (yt - yd) * (h + acceleration_path) / distance_to_target;
  }
  else {
    output_data->interm_xd_ = 0.0F;
    output_data->interm_yd_ = 0.0F;
  }
  output_data->fire_x_ = xd + (xt - xd) * ratio;
  output_data->fire_y_ = yd + (yt - yd) * ratio;

  return true;
}

auto compute_drop_solution(const InputData* input, OutputData* output) -> int
{
  float mass = NAN;
  float drag = NAN;
  float lift = NAN;  // mass in kg, drag and lift coefficients are dimensionless
  float t = NAN;     // NOLINT(readability-identifier-length): projectile free-fall time in seconds.
  float h = NAN;     // NOLINT(readability-identifier-length):projectile trajectory horizontal distance in meters

  // NOLINTBEGIN(cppcoreguidelines-pro-bounds-array-to-pointer-decay): fixed-size C string buffer is used by InputData.
  if (!get_ammo_params(input->ammo_name_, &mass, &drag, &lift)) {
    std::cout << "Error: Unknown ammo type: " << input->ammo_name_ << '\n';
    // NOLINTEND(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    return 1;
  }

  if (!calculate_free_fall_time(input->zd_, input->attack_speed_, mass, drag, lift, &t)) {
    return 1;
  }

  if (!calculate_horizontal_distance(t, input->attack_speed_, mass, drag, lift, &h)) {
    return 1;
  }

  if (!calculate_output_data(input->xd_, input->yd_, input->xt_, input->yt_, input->acceleration_path_, h, output)) {
    return 1;
  }

  return 0;
}