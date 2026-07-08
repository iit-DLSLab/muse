/*
 * euler_angles.h
 *
 *  Created on: Dec 3, 2013
 *      Author: Marco Frigerio
 */

#ifndef IIT_COMMONS_EULER_ANGLES_H_
#define IIT_COMMONS_EULER_ANGLES_H_

#include <assert.h>

#include <Eigen/Core>

#include <iit/rbd/StateDependentMatrix.h>

#include "common.h"
#include "eigen_traits.h"
#include "transforms_internal.h"


/**
 * \file
 * Facilities related to rotation matrices and Euler angles.
 */

/**
 * \defgroup rotations Rotations and Orientation
 */

/**
 * \defgroup eulerangles Euler angles
 * Facilities related to Euler angles.
 *
 * \ingroup rotations
 */

namespace iit {
namespace eulerang {
namespace internal {


#define MATRIX_TYPE_NAME(xxx) Type_S##_X_E##xxx

#define MATRIX_INV_TYPE_NAME(xxx) Type_E##xxx##_X_S


/*
 * This "convention traits" is just a way to associate a value of
 * Convention (ie a convention of Euler angles) to the existing matrix
 * types in iit::commons::eulerang::HomogeneousTransforms
 */
#define CONVENTION_TRAITS(xxx,XXX) \
    template <> \
    struct Traits<RotationMatrices::MATRIX_TYPE_NAME(xxx)> \
    { \
        static constexpr Convention convention = Convention::XXX; \
    }; \
    template <> \
    struct Traits<RotationMatrices::MATRIX_INV_TYPE_NAME(xxx)> \
    { \
        static constexpr Convention convention = Convention::XXX; \
    };

template<typename M>
struct Traits {};

CONVENTION_TRAITS(xyz, XYZ)
CONVENTION_TRAITS(xzy, XZY)
CONVENTION_TRAITS(yxz, YXZ)
CONVENTION_TRAITS(yzx, YZX)
CONVENTION_TRAITS(zxy, ZXY)
CONVENTION_TRAITS(zyx, ZYX)

CONVENTION_TRAITS(xyx, XYX)
CONVENTION_TRAITS(xzx, XZX)

CONVENTION_TRAITS(yxy, YXY)
CONVENTION_TRAITS(yzy, YZY)

CONVENTION_TRAITS(zxz, ZXZ)
CONVENTION_TRAITS(zyz, ZYZ)


#undef CONVENTION_TRAITS

} // namespace internal


/**
 *
 */
template<typename M>
constexpr Convention getConvention(const M& m) {
    return internal::Traits<M>::convention;
}



/**
 * A container of types of the rotation matrices for Euler-angles rotations.
 *
 * \tparam convention one of the 12 possible Euler-angles conventions, as an
 *     item of the enum eulerang::Convention
 *
 * The convention for all the rotation matrices is that of intrinsic rotations,
 * that is, the elemental rotations happen about the axes of the moving frame.
 *
 * Usage:
 * \code
 *   typedef eulerang::Convention Conv;
 *   eulerang::Transform<Conv::XYZ>::Base_R_Rotated Rxyz;
 *   eulerang::Transform<Conv::XYZ>::Rotated_R_Base RIxyz;
 *   //...
 * \endcode
 * These lines create two rotation matrices for the convention XYZ, one being
 * the inverse (transpose) of the other. \c Base_R_Rotated is the type of matrix
 * that, when left multiplied with a column vector, transforms coordinates of
 * the rotated frame to the original frame. \c Rotated_R_Base is obviously the
 * inverse. This second type is provided for convenience, one can always use
 * the transpose of an instance of the first type.
 *
 * These types model state-dependent matrices, that is matrices which can be
 * given a state variable to update their internal state. In this case the state
 * is a value for each of the three rotations, in the form of an object of type
 * eulang::angles_state. So for instance:
 * \code
 *   eulerang::angles_state state;
 *   state.a1 = ((double)std::rand())/RAND_MAX;
 *   state.a2 = ((double)std::rand())/RAND_MAX;
 *   state.a3 = ((double)std::rand())/RAND_MAX;
 *
 *   std::cout << Rxyz(state) * RIxyz(state) << std::endl; // should be the identity
 * \endcode
 *
 * \ingroup eulerangles
 */
template<Convention convention>
struct Transform {};


#define EULERANGLES_SPECIALIZATION(XXX,xxx) \
    template <> \
    struct Transform<Convention::XXX> \
    { \
        typedef internal::RotationMatrices::MATRIX_TYPE_NAME(xxx) Base_R_Rotated; \
        typedef internal::RotationMatrices::MATRIX_INV_TYPE_NAME(xxx) Rotated_R_Base; \
    };

EULERANGLES_SPECIALIZATION(XYZ, xyz)
EULERANGLES_SPECIALIZATION(XZY, xzy)
EULERANGLES_SPECIALIZATION(YXZ, yxz)
EULERANGLES_SPECIALIZATION(YZX, yzx)
EULERANGLES_SPECIALIZATION(ZXY, zxy)
EULERANGLES_SPECIALIZATION(ZYX, zyx)

EULERANGLES_SPECIALIZATION(XYX, xyx)
EULERANGLES_SPECIALIZATION(XZX, xzx)

EULERANGLES_SPECIALIZATION(YXY, yxy)
EULERANGLES_SPECIALIZATION(YZY, yzy)

EULERANGLES_SPECIALIZATION(ZXZ, zxz)
EULERANGLES_SPECIALIZATION(ZYZ, zyz)


#undef EULERANGLES_SPECIALIZATION
#undef MATRIX_TYPE_NAME

}
}



#endif
