#ifndef IIT_COMMONS_DOG_FEET_JACOBIANS_H_
#define IIT_COMMONS_DOG_FEET_JACOBIANS_H_

#include "declarations.h"
#include "leg_data_map.h"

namespace iit {
namespace dog {

/**
 * An interface for the Jacobian of the feet of quadrupeds.
 */
class FeetJacobians
{
public:
    virtual ~FeetJacobians() {};

    /**
     * \name Foot Jacobian getters
     * These functions shall return the 3x3 Jacobian that multiplied by the
     * velocity of the leg joints yields the linear velocity of the foot,
     * expressed in the base reference frame.
     *
     * There are five getters, one for each leg (LF, RF, LH, RH) and one that
     * takes the identifier of the leg of interest.
     */
    ///@{
    virtual FootJac getFootJacobianLF(const JointState& q) = 0;
    virtual FootJac getFootJacobianRF(const JointState& q) = 0;
    virtual FootJac getFootJacobianLH(const JointState& q) = 0;
    virtual FootJac getFootJacobianRH(const JointState& q) = 0;
    virtual FootJac getFootJacobian(const JointState& q, const LegID& leg) = 0;
    /**
     * @brief overload to get the Jacobian on a point of the foot sphere
     * @param q
     * @param leg
     * @param foot_x
     * @param foot_y
     * @return
     */
    virtual FootJac getFootJacobian(const iit::dog::JointState& q,
                                    const iit::dog::LegID& leg,
                                    const double& foot_x,
                                    const double& foot_y) = 0;
    ///@}

    /**
     * \name Foot Jacobian getters
     * These functions shall return the 3x3 Jacobian that multiplied by the
     * velocity of the leg joints yields the angular velocity of the foot,
     * expressed in the base reference frame.
     *
     * There are five getters, one for each leg (LF, RF, LH, RH) and one that
     * takes the identifier of the leg of interest.
     */
    ///@{
    virtual FootJac getAngularFootJacobianLF(const JointState& q) = 0;
    virtual FootJac getAngularFootJacobianRF(const JointState& q) = 0;
    virtual FootJac getAngularFootJacobianLH(const JointState& q) = 0;
    virtual FootJac getAngularFootJacobianRH(const JointState& q) = 0;
    virtual FootJac getAngularFootJacobian(const JointState& q, const LegID& leg) = 0;
    /**
     * @brief overload to get the Jacobian on a point of the foot sphere
     * @param q
     * @param leg
     * @param foot_x
     * @param foot_y
     * @return
     */
    /*virtual FootJac getAngularFootJacobian(const iit::dog::JointState& q,
                                    const iit::dog::LegID& leg,
                                    const double& foot_x,
                                    const double& foot_y) = 0;*/
    ///@}

};


}
}


#endif
