/*
 * eigen_traits.h
 *
 *  Created on: Dec 9, 2013
 *      Author: Marco Frigerio
 */

#ifndef ITT_COMMONS_EULERANGLES_EIGEN_TRAITS_H_
#define ITT_COMMONS_EULERANGLES_EIGEN_TRAITS_H_

#include <Eigen/Core>
#include <iit/rbd/eigen_traits.h>

#include "transforms_internal.h"

namespace Eigen {
namespace internal {

namespace euler_angles = iit::eulerang; // just a shorter alias

template<typename M>
struct traits< euler_angles::internal::RotationMatrix<M> >
    : public traits< iit::rbd::RotationTransformBase<
          typename euler_angles::angles_state, M> > {};

#define EIGEN_TRAITS(xxx) \
    template<> \
    struct traits<euler_angles::internal::RotationMatrices::Type_S##_X_E##xxx> : \
    public traits<euler_angles::internal::RotationMatrix<euler_angles::internal::RotationMatrices::Type_S##_X_E##xxx> > {}; \
    template<> \
    struct traits<euler_angles::internal::RotationMatrices::Type_E##xxx##_X_S> : \
    public traits<euler_angles::internal::RotationMatrix<euler_angles::internal::RotationMatrices::Type_E##xxx##_X_S> > {};

EIGEN_TRAITS(xyz)
EIGEN_TRAITS(xzy)
EIGEN_TRAITS(yxz)
EIGEN_TRAITS(yzx)
EIGEN_TRAITS(zxy)
EIGEN_TRAITS(zyx)

EIGEN_TRAITS(xyx)
EIGEN_TRAITS(xzx)

EIGEN_TRAITS(yxy)
EIGEN_TRAITS(yzy)

EIGEN_TRAITS(zxz)
EIGEN_TRAITS(zyz)

#undef EIGEN_TRAITS

}
}


#endif /* ITT_COMMONS_EULERANGLES_EIGEN_TRAITS_H_ */
