
#ifndef _IIT_COMMONS_DOG_BODYLENGTH_H_
#define _IIT_COMMONS_DOG_BODYLENGTH_H_


namespace iit {
namespace dog {

/**
 * This interface shall be used by application code to retrieve any of the
 * most important length measures of a dog robot.
 * @deprecated This interface has been replaced by dog::KinDynParams
 */
class RobotLengths
{
public:
    virtual ~RobotLengths() {};

    virtual double getHAA_x() = 0;
    virtual double getHAA_y() = 0;
    virtual double getHAA_z() = 0;

    virtual double getDist_HAA_HFE() = 0;
    virtual double getDist_HFE_KFE() = 0;
    virtual double getFoot_x() = 0;
    virtual double getFoot_y() = 0;
    virtual double getFoot_z() = 0;
};


}
}

#endif
