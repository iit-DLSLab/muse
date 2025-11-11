#ifndef IIT_COMMONS_DOG_FORWARD_KINEMATICS_H_
#define IIT_COMMONS_DOG_FORWARD_KINEMATICS_H_


#include "declarations.h"
#include "leg_data_map.h"

namespace iit {
namespace dog {

/**
 * A forward kinematics interface for quadrupeds.
 */
class ForwardKinematics
{
public:
    virtual ~ForwardKinematics() {}

    virtual Vector3d getFootPosLF(const JointState& q) = 0;
    virtual Vector3d getFootPosRF(const JointState& q) = 0;
    virtual Vector3d getFootPosLH(const JointState& q) = 0;
    virtual Vector3d getFootPosRH(const JointState& q) = 0;
    virtual Vector3d getFootPos  (const JointState& q, const LegID& leg) = 0;
    inline virtual dog::LegDataMap<Vector3d> getFeetPos  (const JointState& q)
    {
        dog::LegDataMap<dog::Vector3d> feetPos(Vector3d::Zero());
        feetPos[dog::LF] = getFootPosLF(q);
        feetPos[dog::RF] = getFootPosRF(q);
        feetPos[dog::LH] = getFootPosLH(q);
        feetPos[dog::RH] = getFootPosRH(q);
        return feetPos;
    }
    virtual Matrix3d getFootOrientation(const JointState& q, const LegID& leg) = 0;
    virtual Vector3d getShinPos(const JointState& q,
                                const double& contact_pos,
                                const LegID& leg) = 0;


};


}
}


#endif
