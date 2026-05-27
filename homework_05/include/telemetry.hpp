#pragma once

// Fixed-size storage keeps the starter close to the topics from block 1.
const int MAX_TELEMETRY_FRAMES = 128;

// One telemetry sample from the input log.
struct Frame {
    long timestamp_ms;
    int seq;
    double voltage_v;
    double current_a;
    double temperature_c;
    int gps_fix;
    int satellites;
};

// Aggregated values printed by the executable.
struct Summary {
    int frames_total;
    int frames_valid;
    double voltage_min;
    double voltage_max;
    double temperature_avg;
    int low_voltage_frames;
    double frame_rate_hz;
};

// Try to read frames from a whitespace-separated telemetry log, returning false on failure.
bool try_read_frames(const char* path, Frame frames[], int max_frames, int& frame_count);

// Calculates summary values for already parsed frames.
bool try_summarize(const Frame frames[], int frame_count, Summary& summary);

// Prints summary in the stable homework output format.
void print_summary(const Summary& summary);
