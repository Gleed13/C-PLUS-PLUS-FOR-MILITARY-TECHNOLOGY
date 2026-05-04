#include "telemetry.hpp"

#include <climits>
#include <cstdlib>
#include <fstream>
#include <iostream>

// Debugging exercise notes:
// this file intentionally contains four runtime defects.
// The defects are related to malformed input shape, invalid numeric values,
// unsafe time deltas, and empty logs. Exact locations are not marked on purpose.

const int EXPECTED_FIELD_COUNT = 7;
const int MAX_LINE_LENGTH = 256;

int split_line(char line[], char* fields[], int max_fields) {
    int count = 0;
    char* cursor = line;

    while (*cursor != '\0' && count < max_fields) {
        while (*cursor == ' ' || *cursor == '\t' || *cursor == '\n' || *cursor == '\r') {
            *cursor = '\0';
            ++cursor;
        }

        if (*cursor == '\0') {
            break;
        }

        fields[count] = cursor;
        ++count;

        while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t' && *cursor != '\n' &&
               *cursor != '\r') {
            ++cursor;
        }
    }

    return count;
}

bool try_parse_long(const char* text, long& value) {
    char* end = nullptr;
    value = std::strtol(text, &end, 10);

    if (end == text) {
        return false;
    }

    return true;
}

bool try_parse_int(const char* text, int& value) {
    long temp_value;

    if (try_parse_long(text, temp_value)) {
        // Check if long fits into int
        if (temp_value < INT_MIN || temp_value > INT_MAX)
        {
            return false;
        }
        value = static_cast<int>(temp_value);
        return true;
    }

    return false;
}

bool try_parse_double(const char* text, double& value) {
    char* end = nullptr;
    value = std::strtod(text, &end);

    if (end == text) {
        return false;
    }

    return true;
}

bool try_parse_frame(char line[], Frame& frame) {
    char* fields[EXPECTED_FIELD_COUNT] = {};
    const int field_count = split_line(line, fields, EXPECTED_FIELD_COUNT);
    if (field_count != EXPECTED_FIELD_COUNT) {
        std::cerr << "error: expected " << EXPECTED_FIELD_COUNT
                  << " fields, but got " << field_count << '\n';
        return false;
    }

    if (!try_parse_long(fields[0], frame.timestamp_ms)) {
        std::cerr << "error: failed to parse timestamp_ms: " << fields[0] << '\n';
        return false;
    }
    if (!try_parse_int(fields[1], frame.seq)) {
        std::cerr << "error: failed to parse seq: " << fields[1] << '\n';
        return false;
    }
    if (!try_parse_double(fields[2], frame.voltage_v)) {
        std::cerr << "error: failed to parse voltage_v: " << fields[2] << '\n';
        return false;
    }
    if (!try_parse_double(fields[3], frame.current_a)) {
        std::cerr << "error: failed to parse current_a: " << fields[3] << '\n';
        return false;
    }
    if (!try_parse_double(fields[4], frame.temperature_c)) {
        std::cerr << "error: failed to parse temperature_c: " << fields[4] << '\n';
        return false;
    }
    if (!try_parse_int(fields[5], frame.gps_fix)) {
        std::cerr << "error: failed to parse gps_fix: " << fields[5] << '\n';
        return false;
    }
    if (!try_parse_int(fields[6], frame.satellites)) {
        std::cerr << "error: failed to parse satellites: " << fields[6] << '\n';
        return false;
    }
    return true;
}

bool try_compute_frame_rate_hz(const Frame frames[], int frame_count, double& frame_rate_hz) {
    const long elapsed_ms = frames[frame_count - 1].timestamp_ms - frames[0].timestamp_ms;

    if (elapsed_ms == 0) {
        std::cerr << "error: cannot compute frame rate with zero elapsed time\n";
        return false;
    }

    frame_rate_hz = static_cast<double>((frame_count - 1) * 1000 / elapsed_ms);
    return true;
}

bool try_read_frames(const char* path, Frame frames[], int max_frames, int& frame_count) {
    std::ifstream input{path};
    if (!input) {
        std::cerr << "error: failed to open input file: " << path << '\n';
        return false;
    }

    frame_count = 0;
    char line[MAX_LINE_LENGTH];

    while (input.getline(line, MAX_LINE_LENGTH)) {
        if (line[0] == '\0') {
            continue;
        }

        if (frame_count < max_frames) {
            if (!try_parse_frame(line, frames[frame_count])) {
                std::cerr << "error: failed to parse frame at line " << (frame_count + 1) << '\n';
                return false;
            }
            ++frame_count;
        }
    }

    return true;
}

bool try_summarize(const Frame frames[], int frame_count, Summary& summary) {
    if (frame_count == 0) {
        std::cerr << "error: cannot summarize empty frame list\n";
        return false;
    }

    summary.frames_total = frame_count;
    summary.frames_valid = frame_count;
    summary.voltage_min = frames[0].voltage_v;
    summary.voltage_max = frames[0].voltage_v;
    summary.low_voltage_frames = 0;

    double temperature_sum = 0.0;

    for (int i = 0; i < frame_count; ++i) {
        if (frames[i].voltage_v < summary.voltage_min) {
            summary.voltage_min = frames[i].voltage_v;
        }

        if (frames[i].voltage_v > summary.voltage_max) {
            summary.voltage_max = frames[i].voltage_v;
        }

        temperature_sum += frames[i].temperature_c;

        if (frames[i].voltage_v < 22.0) {
            ++summary.low_voltage_frames;
        }
    }

    const int temperature_tenths = static_cast<int>(temperature_sum * 10.0) / frame_count;
    summary.temperature_avg = static_cast<double>(temperature_tenths) / 10.0;
    if (!try_compute_frame_rate_hz(frames, frame_count, summary.frame_rate_hz)) {
        return false;
    }

    return true;
}

void print_summary(const Summary& summary) {
    std::cout << "frames_total " << summary.frames_total << '\n';
    std::cout << "frames_valid " << summary.frames_valid << '\n';
    std::cout << "voltage_min " << summary.voltage_min << '\n';
    std::cout << "voltage_max " << summary.voltage_max << '\n';
    std::cout << "temperature_avg " << summary.temperature_avg << '\n';
    std::cout << "low_voltage_frames " << summary.low_voltage_frames << '\n';
    std::cout << "frame_rate_hz " << summary.frame_rate_hz << '\n';
}