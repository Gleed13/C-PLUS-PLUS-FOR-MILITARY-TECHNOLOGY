#pragma once

#include <gpiod.hpp>
#include "interfaces/ICheckerController.hpp"

class GpioController final : public ICheckerController {
public:
    ~GpioController() override;
    GpioController(const std::string& chip_path, const unsigned start_line, const unsigned drop_line);
    void init() override;
    void start() override;
    void drop() override;
    void cleanup() override;

private:
    const unsigned start_line_;
    const unsigned drop_line_;
    std::string chip_path_;
    std::optional<gpiod::line_request> request_;

    void ensureInitialized() const;
};