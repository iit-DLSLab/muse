#ifndef IIT_COMMONS_SL_RBD_CONVERSION_H_
#define IIT_COMMONS_SL_RBD_CONVERSION_H_

#include <iit/rbd/rbd.h>
#include <SL.h>


/*!
 * \file
 * \brief This file contains conversion functions for twists and wrenches,
 * between SL data structures and the data types defined in <tt>iit::rbd</tt>
 */

namespace iit {
namespace commons {
namespace SL {

static const double sizeOfThreeDoubles = 3 * sizeof(double);

/**
 * \name iit-RBD-conversion
 * Conversion functions for twists and wrenches,
 * between SL data structures and the data types defined in <tt>iit::rbd</tt>
 */
///@{
inline void baseAccelToSL(
    const iit::rbd::VelocityVector& base_a,
    SL_Cstate& base_state,
    SL_quat& base_quat)
{
    std::memcpy( &(base_state.xdd[_X_]), &(base_a.data()[iit::rbd::LX]), sizeOfThreeDoubles);
    std::memcpy( &(base_quat .add[_X_]), &(base_a.data()[iit::rbd::AX]), sizeOfThreeDoubles);
}

inline void baseAccelFromSL(
    const SL_Cstate& base_state,
    const SL_quat& base_quat,
    iit::rbd::VelocityVector& base_a)
{
    std::memcpy( &(base_a.data()[iit::rbd::LX]), &(base_state.xdd[_X_]), sizeOfThreeDoubles);
    std::memcpy( &(base_a.data()[iit::rbd::AX]), &(base_quat .add[_X_]), sizeOfThreeDoubles);
}


inline void baseVelToSL(
    const iit::rbd::VelocityVector& base_v,
    SL_Cstate& base_state,
    SL_quat& base_quat)
{
    std::memcpy( &(base_state.xd[_X_]), &(base_v.data()[iit::rbd::LX]), sizeOfThreeDoubles);
    std::memcpy( &(base_quat .ad[_X_]), &(base_v.data()[iit::rbd::AX]), sizeOfThreeDoubles);
}

inline void baseVelFromSL(
    const SL_Cstate& base_state,
    const SL_quat& base_quat,
    iit::rbd::VelocityVector& base_v)
{
    std::memcpy( &(base_v.data()[iit::rbd::LX]), &(base_state.xd[_X_]), sizeOfThreeDoubles);
    std::memcpy( &(base_v.data()[iit::rbd::AX]), &(base_quat .ad[_X_]), sizeOfThreeDoubles);
}


inline void wrenchToSL_uext(const iit::rbd::ForceVector& wrench, SL_uext& ux)
{
    std::memcpy( &(ux.f[_X_]), &(wrench.data()[iit::rbd::LX]), sizeOfThreeDoubles);
    std::memcpy( &(ux.t[_X_]), &(wrench.data()[iit::rbd::AX]), sizeOfThreeDoubles);
}


inline void wrenchFromSL_uext(const SL_uext& ux, iit::rbd::ForceVector& wrench)
{
    std::memcpy( &(wrench.data()[iit::rbd::LX]), &(ux.f[_X_]), sizeOfThreeDoubles);
    std::memcpy( &(wrench.data()[iit::rbd::AX]), &(ux.t[_X_]), sizeOfThreeDoubles);
}
///@}


}
}
}


#endif /* IIT_COMMONS_SL_RBD_CONVERSION_H_ */
