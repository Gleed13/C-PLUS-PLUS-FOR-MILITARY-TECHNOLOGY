#include <iostream>

#include "ballistics.hpp"

auto read_input(InputData* input) -> bool
{
  FILE* f = fopen("input.txt", "r");

  if (f == nullptr) {
    std::cout << "Error: File error" << '\n';
    return false;
  }

  int scanned = fscanf(f,
                       "%f %f %f %f %f %f %f %49s",
                       &input->xd_,
                       &input->yd_,
                       &input->zd_,
                       &input->xt_,
                       &input->yt_,
                       &input->attack_speed_,
                       &input->acceleration_path_,
                       input->ammo_name_);

  fclose(f);

  if (scanned != 8) {
    std::cout << "Error: Invalid input format" << '\n';
    return false;
  }

  return true;
}

auto write_output(const OutputData* output_data) -> bool
{
  FILE* out = fopen("output.txt", "w");

  if (out == nullptr) {
    std::cout << "Error: Cannot create output.txt" << '\n';
    return false;
  }

  if (output_data->is_too_close_to_target_) {
    fprintf(out, "%.3f %.3f %.3f %.3f\n", output_data->interm_xd_, output_data->interm_yd_, output_data->fire_x_, output_data->fire_y_);
  }
  else {
    fprintf(out, "%.3f %.3f\n", output_data->fire_x_, output_data->fire_y_);
  }

  fclose(out);
  return true;
}

auto main() -> int
{
  InputData input{};
  OutputData output{};

  if (!read_input(&input)) {
    return 1;
  }

  if (!compute_drop_solution(&input, &output)) {
    return 1;
  }

  if (!write_output(&output)) {
    return 1;
  }

  return 0;
}