#include "models/Target.hpp"

class ITargetProvider {
public:
    virtual int    getTargetCount() = 0;
    virtual Target getTarget(int index) = 0;
    virtual ~ITargetProvider() = default;
};