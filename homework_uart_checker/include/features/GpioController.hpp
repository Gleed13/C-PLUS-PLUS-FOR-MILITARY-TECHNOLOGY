#pragma once

#include <gpiod.hpp>
#include "interfaces/ICheckerController.hpp"

class GpioController final : public ICheckerController {
public:
    ~GpioController() override;
    GpioController(const std::string& chip_path, const int start_line, const int drop_line);
    void init() override;
    void start() override;
    void drop() override;
    void cleanup() override;

private:
    const int start_line_;
    const int drop_line_;
    std::string chip_path_;
    std::optional<gpiod::line_request> request_;

    void ensureInitialized() const;
};