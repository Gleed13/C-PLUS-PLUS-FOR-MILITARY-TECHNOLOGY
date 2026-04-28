#include <cmath>
#include <iostream>
#include <fstream>

const int kTicksPerRevolution = 1024;
const float kWheelRadius = 0.3;
const float kWheelBase = 1.0;

const float kDistancePerTick = 2 * M_PI * kWheelRadius / kTicksPerRevolution;

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: ugv_odometry <input_path>\n";
        return 1;
    }
    std::ifstream input_file(argv[1]);
    if (!input_file.is_open()) {
        std::cerr << "error: could not open input file\n";
        return 1;
    }

    float x = 0.0, y = 0.0, theta = 0.0;
    long timestamp_ms = 0;
    long fl_ticks = 0, fr_ticks = 0, bl_ticks = 0, br_ticks = 0;
    long prev_fl_ticks = 0, prev_fr_ticks = 0, prev_bl_ticks = 0, prev_br_ticks = 0;
    bool first_line = true;
    while (input_file >> timestamp_ms >> fl_ticks >> fr_ticks >> bl_ticks >> br_ticks) {
        if (first_line) {
            prev_fl_ticks = fl_ticks;
            prev_fr_ticks = fr_ticks;
            prev_bl_ticks = bl_ticks;
            prev_br_ticks = br_ticks;
            first_line = false;
            continue;
        }

        // Step 1
        float d_fl = fl_ticks - prev_fl_ticks;
        float d_fr = fr_ticks - prev_fr_ticks;
        float d_bl = bl_ticks - prev_bl_ticks;
        float d_br = br_ticks - prev_br_ticks;
        // Step 2
        float d_left = (d_fl + d_bl) / 2.0;
        float d_right = (d_fr + d_br) / 2.0;
        // Step 3
        float d_l = d_left * kDistancePerTick;
        float d_r = d_right * kDistancePerTick;
        // Step 4
        float d = (d_l + d_r) / 2.0;
        float d_theta = (d_r - d_l) / kWheelBase;
        // Step 5
        x += d * std::cos(theta + d_theta / 2.0);
        y += d * std::sin(theta + d_theta / 2.0);
        theta += d_theta;
        // Output the current pose
        std::cout << timestamp_ms << " " << x << " " << y << " " << theta << "\n";
        // Assign current ticks to previous ticks for next iteration
        prev_fl_ticks = fl_ticks;
        prev_fr_ticks = fr_ticks;
        prev_bl_ticks = bl_ticks;
        prev_br_ticks = br_ticks;
    }

    return 0;
}
