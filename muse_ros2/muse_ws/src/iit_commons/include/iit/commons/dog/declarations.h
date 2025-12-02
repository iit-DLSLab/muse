#ifndef _IIT_COMMONS_DOG_JOINT_ID_DECLARATIONS_H_
#define _IIT_COMMONS_DOG_JOINT_ID_DECLARATIONS_H_

#include <iit/rbd/rbd.h>

namespace iit {
namespace dog {

static const int JointSpaceDimension = 12;
static const int jointsCount = 12;
static const int jointsLegCount = 3; //number of joints per leg
static const int contactConstrCount = 3; //number of constraint for each contact point

//index to define the joint state
static const int baseJoints = 0;
static const int activeJoints = 6;

static const int fbjointsCount = 18; //TODO extension tp centaur
typedef Eigen::Matrix<double, fbjointsCount, 1> Column18d;//TODO extension tp centaur
typedef Column18d FloatingBaseJointState;

/** The total number of rigid bodies of this robot, including the base */
static const int linksCount  = 13;

typedef Eigen::Matrix<double, 12, 1> Column12d;
typedef Column12d JointState;
typedef Eigen::Array<bool, 3, 1> LegJointBool;

typedef iit::rbd::Vector3d Vector3d;
typedef iit::rbd::Matrix33d Matrix3d;
typedef iit::rbd::PlainMatrix<double, 3, 3 > FootJac;

typedef Eigen::Vector3d LegJointState;


enum JointIdentifiers {
    LF_HAA = 0
    , LF_HFE
    , LF_KFE
    , RF_HAA
    , RF_HFE
    , RF_KFE
    , LH_HAA
    , LH_HFE
    , LH_KFE
    , RH_HAA
    , RH_HFE
    , RH_KFE
};


enum LinkIdentifiers {
    TRUNK = 0
    , LF_HIPASSEMBLY
    , LF_UPPERLEG
    , LF_LOWERLEG
    , RF_HIPASSEMBLY
    , RF_UPPERLEG
    , RF_LOWERLEG
    , LH_HIPASSEMBLY
    , LH_UPPERLEG
    , LH_LOWERLEG
    , RH_HIPASSEMBLY
    , RH_UPPERLEG
    , RH_LOWERLEG
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {LF_HAA,LF_HFE,LF_KFE,RF_HAA,RF_HFE,RF_KFE,LH_HAA,LH_HFE,LH_KFE,RH_HAA,RH_HFE,RH_KFE};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {TRUNK,LF_HIPASSEMBLY,LF_UPPERLEG,LF_LOWERLEG,RF_HIPASSEMBLY,RF_UPPERLEG,RF_LOWERLEG,LH_HIPASSEMBLY,LH_UPPERLEG,LH_LOWERLEG,RH_HIPASSEMBLY,RH_UPPERLEG,RH_LOWERLEG};



}
}
#endif
