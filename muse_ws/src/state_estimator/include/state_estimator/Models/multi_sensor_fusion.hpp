#ifndef KF_MULTI_SENSOR_FUSION_HPP
#define KF_MULTI_SENSOR_FUSION_HPP

#include "state_estimator/Observers/EKF.hpp"
#include <unsupported/Eigen/MatrixFunctions>

namespace state_estimator
{

class KFMultiSensorFusion : public EKFBase<double,6,3,9,6,9,6,6>
{
public:
  KFMultiSensorFusion(double t0,
                      const Eigen::Matrix<double,6,1> &xhat0,
                      const Eigen::Matrix<double,6,6> &P0,
                      const Eigen::Matrix<double,6,6> &Q,
                      const Eigen::Matrix<double,9,9> &R)
    : EKFBase(t0, xhat0, P0, Q, R),
      R_base_(R)
  {
  }

  ~KFMultiSensorFusion() = default;

  void predict(double t, const Eigen::Matrix<double,3,1> &u) override
  {
    dt_ = t - this->t_prev;
    if (dt_ < 1e-6) {
      this->t_prev = t;
      return;
    }

    fx_ = calc_f(t, this->xhat, u);
    F_ = calc_F(t, this->xhat, u);

    for (int i = 0; i < 3; ++i) {
      G_(i, i) = -1.0;
    }

    M_.setZero();
    M_.block<6,6>(0, 0) = F_ * dt_;
    M_.block<6,6>(0, 6) = G_ * dt_;

    s_ = M_.exp();
    Phi_ = s_.block<6,6>(0, 0);
    Gamma_ = s_.block<6,6>(0, 6);

    this->P = Phi_ * this->P * Phi_.transpose() + Gamma_ * this->Q * Gamma_.transpose();
    this->xhat = this->xhat + fx_ * dt_;
    this->fixP();

    this->t_prev = t;
  }

  void update(double t, const Eigen::Matrix<double,9,1> &z) override
  {
    yhat_ = calc_h(t, this->xhat);
    H_ = calc_H(t, this->xhat);

    const Eigen::Matrix<double,9,9> S = H_ * this->P * H_.transpose() + this->R;
    K_ = this->P * H_.transpose() * S.inverse();

    this->xhat = this->xhat + K_ * (z - yhat_);
    this->P = (this->I - K_ * H_) * this->P;
    this->fixP();
  }

  void update_proprio(double t, const Eigen::Matrix<double,3,1> &z_vel_world)
  {
    (void)t;
    yhat_vel_ = this->xhat.tail<3>();
    H_vel_.setZero();
    H_vel_.block<3,3>(0, 3) = Eigen::Matrix3d::Identity();

    const Eigen::Matrix3d R_leg = this->R.block<3,3>(0, 0);
    const Eigen::Matrix3d S = H_vel_ * this->P * H_vel_.transpose() + R_leg;
    K_vel_ = this->P * H_vel_.transpose() * S.inverse();

    this->xhat = this->xhat + K_vel_ * (z_vel_world - yhat_vel_);
    this->P = (this->I - K_vel_ * H_vel_) * this->P;
    this->fixP();
  }

  void setMeasurementCovariances(int stance_count)
  {
    this->R = R_base_;

    double leg_scale = 1.0;
    if (stance_count <= 0) {
      leg_scale = 20.0;
    } else if (stance_count == 1) {
      leg_scale = 4.0;
    }

    this->R.block<3,3>(0, 0) *= leg_scale;
  }

protected:
  Eigen::Matrix<double,6,1> calc_f(double /*t*/, const Eigen::Matrix<double,6,1> &x, const Eigen::Matrix<double,3,1> &u) override
  {
    Eigen::Matrix<double,6,1> out;
    out << x.tail<3>(), u;
    return out;
  }

  Eigen::Matrix<double,6,6> calc_F(double /*t*/, const Eigen::Matrix<double,6,1> & /*x*/, const Eigen::Matrix<double,3,1> & /*u*/) override
  {
    Eigen::Matrix<double,6,6> out = Eigen::Matrix<double,6,6>::Zero();
    out(0, 3) = 1.0;
    out(1, 4) = 1.0;
    out(2, 5) = 1.0;
    return out;
  }

  Eigen::Matrix<double,9,1> calc_h(double /*t*/, const Eigen::Matrix<double,6,1> &x) override
  {
    Eigen::Matrix<double,9,1> out;
    out.segment<3>(0) = x.tail<3>();
    out.segment<3>(3) = x.head<3>();
    out.segment<3>(6) = x.tail<3>();
    return out;
  }

  Eigen::Matrix<double,9,6> calc_H(double /*t*/, const Eigen::Matrix<double,6,1> & /*x*/) override
  {
    Eigen::Matrix<double,9,6> out = Eigen::Matrix<double,9,6>::Zero();
    out.block<3,3>(0, 3) = Eigen::Matrix3d::Identity();
    out.block<3,3>(3, 0) = Eigen::Matrix3d::Identity();
    out.block<3,3>(6, 3) = Eigen::Matrix3d::Identity();
    return out;
  }

private:
  double dt_{0.0};

  Eigen::Matrix<double,6,1> fx_{Eigen::Matrix<double,6,1>::Zero()};
  Eigen::Matrix<double,6,6> F_{Eigen::Matrix<double,6,6>::Zero()};
  Eigen::Matrix<double,6,6> G_{Eigen::Matrix<double,6,6>::Identity()};
  Eigen::Matrix<double,12,12> M_{Eigen::Matrix<double,12,12>::Zero()};
  Eigen::Matrix<double,12,12> s_{Eigen::Matrix<double,12,12>::Zero()};
  Eigen::Matrix<double,6,6> Phi_{Eigen::Matrix<double,6,6>::Zero()};
  Eigen::Matrix<double,6,6> Gamma_{Eigen::Matrix<double,6,6>::Zero()};

  Eigen::Matrix<double,9,1> yhat_{Eigen::Matrix<double,9,1>::Zero()};
  Eigen::Matrix<double,9,6> H_{Eigen::Matrix<double,9,6>::Zero()};
  Eigen::Matrix<double,6,9> K_{Eigen::Matrix<double,6,9>::Zero()};

  Eigen::Matrix<double,3,1> yhat_vel_{Eigen::Matrix<double,3,1>::Zero()};
  Eigen::Matrix<double,3,6> H_vel_{Eigen::Matrix<double,3,6>::Zero()};
  Eigen::Matrix<double,6,3> K_vel_{Eigen::Matrix<double,6,3>::Zero()};

  Eigen::Matrix<double,9,9> R_base_{Eigen::Matrix<double,9,9>::Identity()};
};

} // namespace state_estimator

#endif // KF_MULTI_SENSOR_FUSION_HPP
