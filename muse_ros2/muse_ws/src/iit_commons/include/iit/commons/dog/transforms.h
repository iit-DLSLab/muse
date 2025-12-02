#ifndef IIT_COMMONS_DOT_MOTION_TRANSFORMS_H__
#define IIT_COMMONS_DOT_MOTION_TRANSFORMS_H__

#include <iit/rbd/TransformsBase.h>
#include <iit/commons/dog/declarations.h>
#include <iit/rbd/rbd.h>

namespace iit {

namespace dog {

typedef typename iit::rbd::PlainMatrix<double, 6, 6> SpatialTransformPlain ;
typedef typename iit::rbd::PlainMatrix<double, 4, 4> HomogeneousTransformPlain ;

enum class DestFrame {TRUNK = 0, LF_LOWERLEG, RF_LOWERLEG, LH_LOWERLEG, RH_LOWERLEG,
                      LF_FOOT, RF_FOOT, LH_FOOT, RH_FOOT,
                      LF_HIPASSEMBLY, RF_HIPASSEMBLY, LH_HIPASSEMBLY, RH_HIPASSEMBLY,
                      LF_UPPERLEG, RF_UPPERLEG, LH_UPPERLEG, RH_UPPERLEG};

enum class OriginFrame {TRUNK = 0,
                        LF_HAA, LF_HFE, LF_KFE,
                        RF_HAA, RF_HFE, RF_KFE,
                        LH_HAA, LH_HFE, LH_KFE,
                        RH_HAA, RH_HFE, RH_KFE,
                        LF_LOWERLEG, RF_LOWERLEG, LH_LOWERLEG, RH_LOWERLEG,
                        LF_HIPASSEMBLY_COM, RF_HIPASSEMBLY_COM, LH_HIPASSEMBLY_COM, RH_HIPASSEMBLY_COM,
                        LF_HIPASSEMBLY, RF_HIPASSEMBLY, LH_HIPASSEMBLY, RH_HIPASSEMBLY,
                        LF_UPPERLEG_COM,RF_UPPERLEG_COM,LH_UPPERLEG_COM,RH_UPPERLEG_COM,
                        LF_UPPERLEG,RF_UPPERLEG,LH_UPPERLEG,RH_UPPERLEG,
                        LF_LOWERLEG_COM,RF_LOWERLEG_COM,LH_LOWERLEG_COM,RH_LOWERLEG_COM,
                        LF_SHIN,RF_SHIN,LH_SHIN,RH_SHIN,
                        LF_FOOT,RF_FOOT,LH_FOOT,RH_FOOT};

class HomogeneousTransformsBase {
public:
    virtual ~HomogeneousTransformsBase() {}

    virtual HomogeneousTransformPlain getTransform(const JointState& q,
                                                        const OriginFrame& orig,
                                                        const DestFrame& dest) = 0;
};

class MotionTransformsBase {
public:
    virtual ~MotionTransformsBase() {}

    virtual SpatialTransformPlain getTransform(const JointState& q,
                                                        const OriginFrame& orig,
                                                        const DestFrame& dest) = 0;
};


class ForceTransformsBase {
public:
    virtual ~ForceTransformsBase() {}

    virtual SpatialTransformPlain getTransform(const JointState& q,
                                                        const OriginFrame& orig,
                                                        const DestFrame& dest) = 0;
};

}

}

#endif
