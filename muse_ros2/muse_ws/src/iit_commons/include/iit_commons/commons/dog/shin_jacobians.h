#ifndef IIT_COMMONS_DOG_SHIN_JACOBIANS_H_
#define IIT_COMMONS_DOG_SHIN_JACOBIANS_H_

#include "declarations.h"
#include "leg_data_map.h"

namespace iit {
namespace dog {

/**
 * An interface for the Jacobian of the shin of quadrupeds.
 */
class ShinJacobians
{
public:
    virtual ~ShinJacobians() {}


    /**
     * @brief getShinJacobian
     * @param q
     * @param contact_point the contact point along the shin. If 0, it returns
     * the Jacobian at the knee, if equal to shin length, it returns the
     * Jacobian at the foot
     * @param leg
     * @return
     */
    virtual FootJac getShinJacobian(const JointState& q,
                                    const double& contact_point,
                                    const LegID& leg) = 0;

};


}
}


#endif
