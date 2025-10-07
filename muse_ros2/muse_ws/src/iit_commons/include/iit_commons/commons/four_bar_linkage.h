#ifndef IIT_COMMONS_FOURBARLINKAGE_H_
#define IIT_COMMONS_FOURBARLINKAGE_H_

#include <math.h>

namespace iit {
namespace commons {


struct FBLConsts
{
    double akh; ///!< fixed angle between the bar and the line connecting hip-knee joints
    double cj;
    double jk;

    // The linkage bars
    double ak;
    double bk;
    double ar;
    double br;
    double angle_KB_lleg_xaxis;
    double cyl_retracted;
    double actuator_rod_ends_dist_at_cal_position;
};

/**
 * Geometry of the four-bar linkage used in some of iit DLS lab robots
 */
struct FBLStatus
{
    /**
     * The status angle, as in the mechanism documentation within the iit-DLS lab.
     * Ranges from 0 to pi radians.
     */
	double theta;
	/**
	 * The total extension of the linear actuator that moves the linkage.
	 * In meters, always positive.
	 */
	double extension;
	/**
     * The transmission ratio (previously known as "level arm") computed analitically, the velocity gain of the mechanism. This number
	 * multiplied by the joint velocity gives the linear velocity of the
     * linear actuator, multiplied by the force it gives the joint torque
	 */
	double jac;

    /**
     * The transmission ratio (previously known as "level arm") computed numerically, the velocity gain of the mechanism. This number
     * multiplied by the joint velocity gives the linear velocity of the
     * linear actuator, multiplied by the force it gives the joint torque
     */
    double jac_num;


	/**
	 * Default constructor
	 */
	FBLStatus(const FBLConsts&);

	/**
	 * Computes the geometry (linear extension and transmission ratio)
	 * given the mechanism angle.
	 *
	 * \param th the joint angle (in radians) as in the mechanism documentation
	 */
	void setAngle(double th) {
		theta = th;
		updateGeometry();
	}

	/**
	 * Computes the mechanism angle (\c theta) given the linear actuator extension.
	 * It does *NOT* compute the lever.
	 * \param l the total current length of the actuator
	 */
	void setExtension(double l);

    void computeNumericJacobian(double q);


    /**
     * Converts a knee joint angle to the input angle used to calculate the four
     * bar linkage geometry.
     *
     * \param angle the joint angle (in radians), in robot conventions; that is, the value
     *    returned by the I/O component of the robot
     * \param whichOne the identifier of the leg whose knee is of interest
     *
     * Currently, the conversion from the robot joint angle q, to the angle as
     * defined in the documentation of the mechanism, is simply PI-|q|, and it
     * applies to any knee.
     */
    inline double kneeAngleToFBLAngle(double angle)
    {
        return M_PI - fabs(angle) + angle_KB_lleg_xaxis;
    }

    inline double FBLAngleToKneeAngle(double angle)
    {
        return - (M_PI - angle + angle_KB_lleg_xaxis);
    }


private:
	void updateGeometry();
private:
	FBLConsts consts;
	// other constants that can be computed from the input values
	double ak2;
	double bk2;
	double ar2;
	double br2;
	double ck;
	double ck2;
	double ckh;
	double cka;
	double ca;
	double ca2;
	double cak;
    double angle_KB_lleg_xaxis;
    double cyl_retracted;
    double actuator_rod_ends_dist_at_cal_position;
};





}
}


#endif
