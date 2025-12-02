/*
 * basics.h
 *
 *  Created on: Dec 11, 2013
 *      Author: Marco Frigerio
 */

#ifndef IIT_COMMONS_EULER_BASICS_H_
#define IIT_COMMONS_EULER_BASICS_H_

namespace iit {
/**
 * Facilities related to Euler angles, like rotation matrices
 * \ingroup eulerangles
 */
namespace eulerang {


/**
 * All the 12 possible combinations of three consecutive rotations.
 * Note that, for a unique specification of a rotation, one must also specify
 * whether each elemental rotation happens about the rotated axis (aka intrinsic
 * rotations) or about the fixed axes (extrinsic rotations).
 *
 * \ingroup eulerangles
 */
enum class Convention {
    XYZ, XZY,    YXZ, YZX,    ZXY, ZYX,
    XYX, XZX,    YXY, YZY,    ZXZ, ZYZ
};

/**
 * A container for the value of three Euler angles.
 * Note that any value of this struct is meaningful if interpreted in relation
 * to a \ref Convention "convention of Euler angles".
 *
 * \ingroup eulerangles
 */
struct angles_state {
    double a1; ///< the amount of rotation about the first axis
    double a2;
    double a3;
};



}
}

#endif
