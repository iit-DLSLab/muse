#ifndef IIT_COMMONS_DOG_TEST_FORWARD_KINEMATICS_H_
#define IIT_COMMONS_DOG_TEST_FORWARD_KINEMATICS_H_

#include <iostream>
#include <iit/rbd/rbd.h>

#include "forward_kinematics.h"
#include "leg_data_map.h"

namespace iit {
namespace dog {

class TestFK
{

public:

    TestFK(ForwardKinematics& fwdkin) : fk(fwdkin)
    {
    };

    ~TestFK() {};

    void printPositions(const JointState& q)
    {
        std::cout << "LF: " << fk.getFootPosLF(q).transpose() << std::endl;
        std::cout << "RF: " << fk.getFootPosRF(q).transpose() << std::endl;
        std::cout << "LH: " << fk.getFootPosLH(q).transpose() << std::endl;
        std::cout << "RH: " << fk.getFootPosRH(q).transpose() << std::endl;

        for (auto leg : {LF,RF,LH,RH}) {
            std::cout << fk.getFootPos(q, leg).transpose() << "  ,  ";
        }
        std::cout << std::endl;
    }

private:
    ForwardKinematics& fk;
};


}
}


#endif
