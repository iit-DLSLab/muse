#ifndef IIT_COMMONS_DOG_TEST_KINEMATICS_H_
#define IIT_COMMONS_DOG_TEST_KINEMATICS_H_

#include <iostream>
#include <iit/rbd/rbd.h>
#include <iit/rbd/utils.h>

#include "forward_kinematics.h"
#include "inverse_kinematics.h"
#include "leg_data_map.h"

namespace iit {
namespace dog {


class TestK
{
public:
    struct Result
    {
        JointState q;
        JointState q_ik;
        bool equal;
        LegDataMap<Vector3d> feet;
    };
public:

    TestK(ForwardKinematics& fwdkin, InverseKinematics& invkin)
        : fk(fwdkin), ik(invkin)
    {
    };

    ~TestK() {};

    void testFK_IK(const JointState& q, Result& res)
    {
        res.q = q;
        res.feet[LF] = fk.getFootPosLF(q);
        res.feet[RF] = fk.getFootPosRF(q);
        res.feet[LH] = fk.getFootPosLH(q);
        res.feet[RH] = fk.getFootPosRH(q);

        ik.getJointPosition(res.feet, res.q_ik);

        rbd::Utils::CwiseAlmostZeroOp<double> approx(1E-4);
        res.equal = ( (res.q-res.q_ik).transpose().unaryExpr(approx).sum() == 0.0 );

    }

private:
    ForwardKinematics& fk;
    InverseKinematics& ik;
};


}
}


#endif
