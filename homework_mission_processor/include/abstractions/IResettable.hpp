#pragma once

class IResettable {
public:
    virtual void reset() = 0;
    virtual ~IResettable() = default;
};