#pragma once

class ICheckerController {
public:
    virtual ~ICheckerController();
    virtual void init() = 0;
    virtual void start() = 0;
    virtual void drop() = 0;
    virtual void cleanup() = 0;
};