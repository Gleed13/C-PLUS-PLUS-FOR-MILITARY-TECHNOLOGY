#include <iostream>

#include "ballistics.hpp"

auto read_input(InputData* input) -> bool
{
  // NOLINTBEGIN(cppcoreguidelines-owning-memory): C-style file API is used intentionally for this homework task.
  FILE* input_file = fopen("input.txt", "r");

  if (input_file == nullptr) {
    std::cout << "Error: File error" << '\n';
    return false;
  }

  int scanned =
    fscanf(  // NOLINT(cppcoreguidelines-pro-type-vararg): fscanf is used intentionally for simple formatted input in this homework.
      input_file,
      "%f %f %f %f %f %f %f %49s",
      &input->xd_,
      &input->yd_,
      &input->zd_,
      &input->xt_,
      &input->yt_,
      &input->attack_speed_,
      &input->acceleration_path_,
      input->ammo_name_);  // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay): ammo_name_ is required by the current InputData

  fclose(input_file);
  // NOLINTEND(cppcoreguidelines-owning-memory): matching fclose for FILE* opened with fopen above.

  // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers): expected number of fields in input.txt format.
  if (scanned != 8) {
    std::cout << "Error: Invalid input format" << '\n';
    return false;
  }
  // NOLINTEND(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)

  return true;
}

auto write_output(const OutputData* output_data) -> bool
{
  // NOLINTBEGIN(cppcoreguidelines-owning-memory): C-style file API is used intentionally for this homework task.
  FILE* output_file = fopen("output.txt", "w");

  if (output_file == nullptr) {
    std::cout << "Error: Cannot create output.txt" << '\n';
    return false;
  }

  if (output_data->is_too_close_to_target_) {
    fprintf(output_file,  // NOLINT(cppcoreguidelines-pro-type-vararg): fprintf is used intentionally for formatted output in this homework.
            "%.3f %.3f %.3f %.3f\n",
            output_data->interm_xd_,
            output_data->interm_yd_,
            output_data->fire_x_,
            output_data->fire_y_);
  }
  else {
    fprintf(  // NOLINT(cppcoreguidelines-pro-type-vararg): fprintf is used intentionally for formatted output in this homework.
      output_file,
      "%.3f %.3f\n",
      output_data->fire_x_,
      output_data->fire_y_);
  }

  fclose(output_file);
  // NOLINTEND(cppcoreguidelines-owning-memory): matching fclose for FILE* opened with fopen above.
  return true;
}

auto main() -> int
{
  InputData input{};
  OutputData output{};

  if (!read_input(&input)) {
    return 1;
  }

  if (compute_drop_solution(&input, &output) == 1) {
    return 1;
  }

  if (!write_output(&output)) {
    return 1;
  }

  return 0;
}