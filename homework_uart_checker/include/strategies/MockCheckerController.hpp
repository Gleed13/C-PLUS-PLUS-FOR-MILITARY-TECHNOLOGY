#pragma once

#include <string>

#include "interfaces/ICheckerController.hpp"

class MockCheckerController final : public ICheckerController {
public:
    ~MockCheckerController() override;
    MockCheckerController(const std::string& chip_path, const unsigned start_line, const unsigned drop_line);
    void init() override;
    void start() override;
    void drop() override;
    void cleanup() override;
private:
    std::string chip_path_;
    const unsigned start_line_;
    const unsigned drop_line_;
};