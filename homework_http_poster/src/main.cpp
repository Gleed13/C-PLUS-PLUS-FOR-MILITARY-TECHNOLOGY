#include <chrono>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include <httplib.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace {

constexpr const char* kHost = "cppmiltech.com.ua";
constexpr int kPort = 80;
constexpr const char* kApiKey = "dz12-vX7mK4qT9r2w";
constexpr const char* kStudentId = "1040";
constexpr int kMaxAttempts = 5;
constexpr int kRetryPauseSeconds = 1;
constexpr int kTimeoutSeconds = 2;

struct TestResult {
    std::string testId;
    std::string status;
    int attempts = 0;
};

std::string MakeTestId(int index) {
    char buf[8];
    std::snprintf(buf, sizeof(buf), "T%02d", index);
    return buf;
}

bool ReadSimulationFile(const std::string& path, json& outSimulation) {
    std::ifstream file(path);
    if (!file.is_open()) {
        return false;
    }
    file >> outSimulation;
    return true;
}

httplib::Client MakeClient() {
    httplib::Client cli(kHost, kPort);
    cli.set_connection_timeout(kTimeoutSeconds, 0);
    cli.set_read_timeout(kTimeoutSeconds, 0);
    return cli;
}

bool VerifyResultOnServer(httplib::Client& cli, const std::string& testId) {
    const std::string path = "/api/dz12/results/" + testId + "/" + kStudentId;
    httplib::Headers headers = {{"x-api-key", kApiKey}};
    auto res = cli.Get(path, headers);
    if (!res) {
        std::printf("[WARN] %s: GET verification failed (no response)\n", testId.c_str());
        return false;
    }
    if (res->status != 200) {
        std::printf("[WARN] %s: GET verification returned status %d\n", testId.c_str(), res->status);
        return false;
    }
    json body = json::parse(res->body, nullptr, false);
    if (body.is_discarded() || !body.value("found", false)) {
        std::printf("[WARN] %s: GET verification body did not confirm result\n", testId.c_str());
        return false;
    }
    return true;
}

TestResult PostSingleResult(httplib::Client& cli, const std::string& testId, const json& simulation) {
    TestResult result;
    result.testId = testId;

    json payload = {
        {"studentId", kStudentId},
        {"testId", testId},
        {"simulation", simulation},
    };
    const std::string body = payload.dump();

    httplib::Headers headers = {{"x-api-key", kApiKey}};

    for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
        result.attempts = attempt;

        auto res = cli.Post("/api/dz12/results", headers, body, "application/json");

        if (!res) {
            std::printf("[LOG] %s attempt %d: timeout/no response\n", testId.c_str(), attempt);
            if (attempt < kMaxAttempts) {
                std::this_thread::sleep_for(std::chrono::seconds(kRetryPauseSeconds));
            }
            continue;
        }

        if (res->status == 200 || res->status == 201) {
            std::printf("[LOG] %s attempt %d: server returned %d\n", testId.c_str(), attempt, res->status);
            result.status = VerifyResultOnServer(cli, testId) ? "success" : "success (verify failed)";
            return result;
        }

        if (res->status == 400) {
            std::printf("[LOG] %s attempt %d: bad request (400): %s\n", testId.c_str(), attempt, res->body.c_str());
            result.status = "failed (400 bad request)";
            return result;
        }

        if (res->status == 401) {
            std::printf("[LOG] %s attempt %d: unauthorized (401)\n", testId.c_str(), attempt);
            result.status = "failed (401 unauthorized)";
            return result;
        }

        if (res->status == 503) {
            std::printf("[LOG] %s attempt %d: service unavailable (503)\n", testId.c_str(), attempt);
            if (attempt < kMaxAttempts) {
                std::this_thread::sleep_for(std::chrono::seconds(kRetryPauseSeconds));
            }
            continue;
        }

        std::printf("[LOG] %s attempt %d: unexpected status %d\n", testId.c_str(), attempt, res->status);
        result.status = "failed (unexpected status " + std::to_string(res->status) + ")";
        return result;
    }

    result.status = "failed (max attempts reached)";
    return result;
}

void PrintReport(const std::vector<TestResult>& results) {
    std::printf("\n=== Report ===\n");
    std::printf("%-6s %-30s %s\n", "Test", "Status", "Attempts");
    for (const auto& r : results) {
        std::printf("%-6s %-30s %d\n", r.testId.c_str(), r.status.c_str(), r.attempts);
    }
}

}  // namespace

int main() {
    std::vector<TestResult> results;
    httplib::Client cli = MakeClient();

    for (int i = 1; i <= 10; ++i) {
        const std::string testId = MakeTestId(i);
        const std::string path = "data/" + std::to_string(i) + "/simulation.json";

        json simulation;
        if (!ReadSimulationFile(path, simulation)) {
            std::printf("[LOG] %s: no local simulation.json found, skipping\n", testId.c_str());
            continue;
        }

        results.push_back(PostSingleResult(cli, testId, simulation));
    }

    PrintReport(results);
    return 0;
}
