#include <iit/commons/control/VirtualModel.hpp>
#include <iit/commons/geometry/rotations.h>
#include <iit/rbd/InertiaMatrix.h>

namespace iit {
namespace control {


VirtualModel::VirtualModel() :
           Kp_(Wrench::Zero()),
           Kd_(Wrench::Zero()),
           wrenchErrorThreshold_(Wrench::Constant(400.0))
{
}


void VirtualModel::setGains(const Gains &Kp_in, const Gains &Kd_in) {
    this->Kp_ = Kp_in;
    this->Kd_ = Kd_in;
}


void VirtualModel::setGains(double lx_p, double ly_p, double lz_p,
              double lx_d, double ly_d, double lz_d,
              double ax_p, double ay_p, double az_p,
              double ax_d, double ay_d, double az_d)
{
    Wrench Kp;
    Wrench Kd;

    // Proportional  gain, linear part
    Kp(rbd::LX) = lx_p;
    Kp(rbd::LY) = ly_p;
    Kp(rbd::LZ) = lz_p;

    // Proportional  gain, angular part
    Kp(rbd::AX) = ax_p;
    Kp(rbd::AY) = ay_p;
    Kp(rbd::AZ) = az_p;

    // Derivative  gain, linear part
    Kd(rbd::LX) = lx_d;
    Kd(rbd::LY) = ly_d;
    Kd(rbd::LZ) = lz_d;

    // Derivative  gain, angular part
    Kd(rbd::AX) = ax_d;
    Kd(rbd::AY) = ay_d;
    Kd(rbd::AZ) = az_d;

    setGains(Kp,Kd);
}


void VirtualModel::setGain(rbd::Coords6D index, GainType type, double value)
{
    if (type == PROPORTIONAL){
        this->Kp_(index)= value;
    }
    if (type == DERIVATIVE){
        this->Kd_(index)= value;
    }
}

void VirtualModel::getGains(Gains & Kp_out, Gains & Kd_out)
{
    Kp_out = Kp_;
    Kd_out = Kd_;
}


//this function computes the wrench (forces) generated by a PD attractor (in the world frame)
//des/actual_pos is supposed in the world frame
//des orient are euler angles and derivatives /
//actual_orient.xd is the angular velocity expressed in the base frame
VirtualModel::Wrench VirtualModel::getFeedBackWrench(const PointState & des_lin_state,
                                                     const PointState & actual_lin_state,
                                                     const PointState & des_ang_state,
                                                     const PointState & actual_ang_state,
                                                     const double & robot_mass,
                                                     const Wrench& wrench_error)
{

    //these generalized moments and forces are in the world frame
    Wrench wrench_out;
    Eigen::Matrix3d  R = commons::rpyToRot(actual_ang_state.x);

    //gracefully degrade performances
    rbd::Vector6D wrench_error_base;
    Wrench scaleGains = Wrench::Ones();

    // project the wrench error in the base frame
    wrench_error_base.segment(rbd::AX,3) = R * wrench_error.segment(rbd::AX,3);
    wrench_error_base.segment(rbd::LX,3) = R * wrench_error.segment(rbd::LX,3);

    scaleGains = rbd::Vector6D::Constant(1.0) -
            (wrench_error_base.cwiseAbs()).cwiseProduct(wrenchErrorThreshold_.cwiseInverse());

    // clip  between 0 and 1
    scaleGains = (scaleGains.array()>0.0).select(scaleGains.array(), 0.0);

    Kp_limited_ = scaleGains.cwiseProduct(Kp_);
    Kd_limited_ = scaleGains.cwiseProduct(Kd_);

    //linear position
    wrench_out.segment(rbd::LX,3) = R.transpose() *
            Kp_limited_.segment(rbd::LX,3).asDiagonal() * R *
            (des_lin_state.x - actual_lin_state.x) +
            R.transpose()* Kd_limited_.segment(rbd::LX,3).asDiagonal() * R *
            (des_lin_state.xd - actual_lin_state.xd);

    // orientation (omega is expressed in the base frame)
    // the orientation vector is returned either in the base_frame or
    // des_base_frame so it should be rotated as well to have the wrench into
    // the world frame
    //we need to transform the euler rates and euler_rates derivatives in omega omega_dot
    //compute matrix that transform euler rates into othogonal (Euclidean) frame
    Eigen::Matrix3d Jomega = commons::rpyToEar(des_ang_state.x);

    //new
    wrench_out.segment(rbd::AX,3) = R.transpose() *
            (Kp_limited_.segment(rbd::AX,3).asDiagonal() *
             commons::computeOrientError(des_ang_state.x, actual_ang_state.x) +
             Kd_limited_.segment(rbd::AX,3).asDiagonal() *(Jomega*des_ang_state.xd - actual_ang_state.xd));
             // Rt map into the moments in world frame cause omega
             // is defined in base frame


    //apply wrenchmask
    return wrench_out;


}

VirtualModel::Wrench VirtualModel::getFeedForwardWrench(const Vector3d& des_lin_acc,
                                                        const Vector3d& des_euler_rate_dot,
                                                        const Vector3d& des_ang_pos,
                                                        const Vector3d& des_euler_rate,
                                                        const double& mass,
                                                        const InertiaMatrix& inertia_matrix){
    Wrench wrench_out(Wrench::Zero());
    Eigen::Matrix3d  R = commons::rpyToRot(des_ang_pos);
    //linear term
    //multiply by the robot mass
    wrench_out.segment(rbd::LX,3) = mass * des_lin_acc;



    //matrix to transform euler rates into world frame
    Eigen::Matrix3d J_omega = commons::rpyToEarInv(des_ang_pos);
    //matrix to  transform euler rates derivatives into into world frame
    Eigen::Matrix3d J_omega_dot = commons::rpyToEarInv_dot(des_ang_pos, des_euler_rate);
    //compute desired omega dot in base frame
    Vector3d des_omega_dot = R * (J_omega * des_euler_rate_dot + J_omega_dot*des_euler_rate);

    //angular term (Ic is supposed in the base frame)
    //new
    //multiply by the trunk inertia I is supposed in the base frame)
    wrench_out.segment(rbd::AX,3) = R.transpose() *
           inertia_matrix.block(rbd::AX, rbd::AX,3,3) * des_omega_dot;


    return wrench_out;
}





//this function adds the ffwd accel term that will be multiplied by
//mass in the linear case and inertia tensor in the angular case
//the angular acceleration actual_orient.xdd is expressed in the base frame the linear one actual_pos.xdd is in the world frame

/**
 * @brief computes the total wrench at the base, including the feedforward and
 * the feedback terms.
 * @param des_pos
 * @param actual_pos
 * @param des_orient
 * @param actual_orient
 * @param mass
 * @param inertia_matrix
 * @param wrench_error
 * @return
 * @sa VirtualModel::getFeedForWardWrench() and VirtualModel::getFeedBackWrench()
 */
VirtualModel::Wrench VirtualModel::getTotalWrench(const PointState & des_pos,
                                                  const PointState & actual_pos,
                                                  const PointState & des_orient,
                                                  const PointState & actual_orient,
                                                  const double & mass,
                                                  const InertiaMatrix& inertia_matrix,
                                                  const Wrench& wrench_error)
{

    //new
    Wrench wrench_out =  getFeedForwardWrench(des_pos.xdd,
                                              des_orient.xdd,
                                              des_orient.x,
                                              des_orient.xd,
                                              mass, inertia_matrix);

    wrench_out += getFeedBackWrench(des_pos, actual_pos, des_orient,
                                    actual_orient, mass, wrench_error);

    return wrench_out;
}

void VirtualModel::setWrenchErrorThreshold(const Wrench& wrench_error_threshold){
    this->wrenchErrorThreshold_ = wrench_error_threshold;
}

} // namespace control
} // namespace iit

