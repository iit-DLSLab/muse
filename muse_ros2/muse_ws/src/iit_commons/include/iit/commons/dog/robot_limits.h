#ifndef IIT_DOG_LIMITS_H
#define IIT_DOG_LIMITS_H

#include "declarations.h"

namespace iit{

namespace dog {

class LimitsBase
{
public:
    virtual void setMaxEffort(const JointState & max_effort) = 0;

    virtual JointState getTorqueLimits(const JointState & q) = 0;

};

}
}

#endif
