#ifndef IIT_CONTROL_VIRTUALMODEL_HPP_
#define IIT_CONTROL_VIRTUALMODEL_HPP_


#include <iit/rbd/rbd.h>
#include <Eigen/Dense>
#include <iit/commons/planning/planning.h>

namespace iit {
namespace control {

/**
 * @brief Creates linear and angular virtual model accelerations based on a
 *        difference between a desired and a current state.
 * @authors Michele Focchi (michele.focchi@iit.it)
 * Marco Camurri (marco.camurri@iit.it)
 * @date Dec 16, 2014
 */
class VirtualModel {
public:
    typedef iit::rbd::ForceVector Gains;
    typedef iit::rbd::ForceVector Wrench;
    typedef iit::rbd::InertiaMatrixDense InertiaMatrix;
    typedef iit::planning::Point3d PointState;
    typedef Eigen::Vector3d Vector3d;

    enum GainType{PROPORTIONAL=0, DERIVATIVE};

    /**
     * @brief VirtualModel default constructor.
     */
    VirtualModel();

    /**
     * @brief setGains sets the proportional and derivative gains for the
     * wrench tracking
     * @param Kp_in a 6D rbd::ForceVector representing the proportional gain
     * @param Kd_in a 6D rbd::ForceVector representing the derivative gain
     */
    void setGains(const  Gains & Kp_in, const  Gains & Kd_in);



    /**
     * @brief overload of setGains(const Gains & Kp_in, const Gains & Kd_in)
     * with single scalar values
     * @param[in] lx_p linear proportional gain, direction x
     * @param[in] ly_p linear proportional gain, direction y
     * @param[in] lz_p linear proportional gain, direction z
     * @param[in] lx_d linear derivative gain, direction x
     * @param[in] ly_d linear derivative gain, direction y
     * @param[in] lz_d linear derivative gain, direction z
     * @param[in] ax_p angular proportional gain, direction x
     * @param[in] ay_p angular proportional gain, direction y
     * @param[in] az_p angular proportional gain, direction z
     * @param[in] ax_d angular derivative gain, direction x
     * @param[in] ay_d angular derivative gain, direction y
     * @param[in] az_d angular derivative gain, direction z
     * @sa setGains(const  Gains & Kp_in, const  Gains & Kd_in)
     */
    void setGains(double lx_p, double ly_p, double lz_p,
                  double lx_d, double ly_d, double lz_d,
                  double ax_p, double ay_p, double az_p,
                  double ax_d, double ay_d, double az_d);
    /**
     * @brief setGain set a specific scalar value of one of the gains
     * @param[in] index which component of the gain to set (e.g., rbd::AX for angular
     * part, direction X)
     * @param[in] type VirtualModel::PROPORTIONAL or VirtualModel::DERIVATIVE
     * @param[in] value to be set for the selected gain
     */
    void setGain(rbd::Coords6D index, GainType type, double value);

    /**
     * @brief getGains
     * @param[out] Kp_out
     * @param[out] Kd_out
     */
    void getGains(Gains & Kp_out, Gains & Kd_out);

    /**
     * @brief Calculates necessary body wrench to move the body from actual to des.
     * @param[in] des_pos desired position(x)/velocity(xd)/acceleration(xdd)  from motion plan.
     * @param[in] des_orient desired orientation(x)/velocity(xd)/acceleration(xdd)  from motion plan.
     * @param[in] actual_pos actual position(x)/velocity(xd)/acceeleration(xdd) from state estimation.
     * @param[in] actual_orient actual orientation(x) and angular velocity (xd)
     *  expressed in the world frame.
     *
     * @return A Wrench (a.k.a. rbd::ForceVector) expressing the wrench to be
     * applied where the virtual model is attached in order to close the position
     * and velocity control loop, without the feedforward term.
    */
    Wrench getFeedBackWrench(const PointState & des_pos,
                             const PointState & actual_pos,
                             const PointState & des_orient,
                             const PointState & actual_orient,
                             const double &robot_mass,
                             const Wrench &wrench_error = Wrench::Zero());
    /**
     * @brief getFeedForwardWrench computes the feedforward term of the wrench.
     * @param[in] des_lin_acc desired linear acceleration
     * @param[in] des_euler_rate_dot desired Euler rate derivatives
     * @param[in] des_ang_pos desired angular position, expressed as Euler angles
     * with ZYX convention
     * @param[in] des_euler_rate desired Euler rates
     * @param[in] mass robot mass
     * @param[in] inertia_matrix expressed in a frame aligned with the base
     * frame
     * @return the feedforward term of the virtual model wrench
     */
    Wrench getFeedForwardWrench(const Vector3d& des_lin_acc,
                                const Vector3d& des_euler_rate_dot,
                                const Vector3d& des_ang_pos,
                                const Vector3d& des_euler_rate,
                                const double& mass,
                                const InertiaMatrix& inertia_matrix);

    /**
     * @brief getTotalWrench Calculates necessary body wrench to move the body
     * from actual to des with FFWd action.
     * @param[in] des_lin_state desired linear position (x), velocity(xd) and
     * acceleration(xdd) from motion plan, expressed in the world frame.
     * @param[in] actual_lin_state actual linear position(x), velocity(xd) and
     * acceleration(xdd) from motion plan, expressed in the world frame.
     * @param[in] des_ang_state desired angular position(x), expressed ad Euler
     * angles with ZYX convention; desired Euler rate (xd); and
     * desired euler rate derivatives (xdd) from motion plan.
     * @param[in] actual_ang_state actual angular angular position(x), expressed ad Euler
     * angles with ZYX convention; actual angular velocity (xd), expressed in
     * the base frame; and actual angular acceleration (xdd), expressed in the
     * base frame.
     * @param[in] robot_mass total robot mass, expressed in kg
     * @param[in] inertia_matrix Inertia matrix for angular and linear
     * (only robot mass) acceleration.
     * ROBOT::dyn::InertiaMatrix Ic = jsim(q).getWholeBodyInertia();
     * @param[in] wrench_error
     * @return A Wrench (a.k.a. rbd::ForceVector) expressing the wrench to be
     * applied where the virtual model is attached in order to close the position
     * and velocity control loop
     * @attention The desired angular velocity and acceleration are expressed
     * as Euler rates and Euler rates derivatives, whereas the corresponding
     * actual quantities are expressed as angular velocities and acceleration
     * in the base frame!
     */
    Wrench getTotalWrench(const PointState & des_lin_state,
                          const PointState & actual_lin_state,
                          const PointState & des_ang_state,
                          const PointState & actual_ang_state,
                          const double& robot_mass,
                          const InertiaMatrix& inertia_matrix,
                          const Wrench &wrench_error = Wrench::Zero());

    /**
     * @brief setWrenchErrorThreshold sets the wrench threshold after which
     * the feedback control action starts reducing the gains for safety.
     * By default, this is set to 400.
     * @param wrenchErrorTh
     */
    void setWrenchErrorThreshold(const Wrench& wrenchErrorTh);

private:
    Gains Kp_;
    Gains Kp_limited_;
    Gains Kd_;
    Gains Kd_limited_;
    bool gravityCompensation_;
    Wrench wrenchErrorThreshold_;
};

} // namespace control
} // namespace iit


#endif /* IIT_CONTROL_VIRTUALMODEL_HPP_ */

