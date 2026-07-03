#pragma once

#include <string>

#include "interfaces/ICheckerController.hpp"

class MockCheckerController final : public ICheckerController {
public:
    ~MockCheckerController() override;
    MockCheckerController(const std::string& chip_path, const int start_line, const int drop_line);
    void init() override;
    void start() override;
    void drop() override;
    void cleanup() override;
private:
    std::string chip_path_;
    int start_line_;
    int drop_line_;
};