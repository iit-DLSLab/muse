#ifndef IIT_COMMONS_DOG_INERTIA_PROPERTIES_H__
#define IIT_COMMONS_DOG_INERTIA_PROPERTIES_H__

#include <iit/rbd/rbd.h>

#include "declarations.h"
#include "leg_data_map.h"


namespace iit {
namespace dog {

class InertiaPropertiesBase {
public:
    virtual ~InertiaPropertiesBase() {}

    virtual const iit::rbd::Vector3d& getTrunkCOM() = 0;
    virtual double getTrunkMass()  = 0;

    virtual double getHipAssemblyMass(const LegID& leg) const = 0;
    virtual double getUpperLegMass(const LegID& leg) const = 0;
    virtual double getLowerLegMass(const LegID& leg) const = 0;

    virtual Vector3d getHipAssemblyCOM(const LegID& leg) const = 0;
    virtual Vector3d getUpperLegCOM(const LegID& leg) const = 0;
    virtual Vector3d getLowerLegCOM(const LegID& leg) const = 0;

    virtual double getTotalMass() const = 0;
    /**
     * @brief getWholeBodyCOM  computes the Center Of Mass (COM) position of
     * the whole robot, in  base coordinates.
     * @param q the joint status vector describing the configuration of the robot
     * @return the position of the Center Of Mass of the whole robot, expressed
               in base coordinates
     */
    virtual Vector3d getWholeBodyCOM(const JointState& q) = 0;

};
}
}

#endif
