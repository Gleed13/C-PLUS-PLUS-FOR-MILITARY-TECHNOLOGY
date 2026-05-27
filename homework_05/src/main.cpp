#include "telemetry.hpp"

#include <iostream>

int main(int argc, char** argv) {
    // The executable expects exactly one telemetry log path.
    if (argc != 2) {
        std::cerr << "usage: telemetry_check <input_path>\n";
        return 1;
    }

    Frame frames[MAX_TELEMETRY_FRAMES];
    int frame_count;
    if (!try_read_frames(argv[1], frames, MAX_TELEMETRY_FRAMES, frame_count)) {
        std::cerr << "error: failed to read frames from file: " << argv[1] << '\n';
        return 1;
    }

    Summary summary;
    if (!try_summarize(frames, frame_count, summary)) {
        std::cerr << "error: failed to summarize frames\n";
        return 1;
    }
    print_summary(summary);

    return 0;
}
