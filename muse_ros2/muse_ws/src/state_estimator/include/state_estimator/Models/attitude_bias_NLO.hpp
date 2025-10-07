#ifndef ATTITUDEBIASNLO_HPP
#define ATTITUDEBIASNLO_HPP

#include "state_estimator/Observers/NLO.hpp"

namespace state_estimator {

	class AttitudeBiasNLO : public NLO<double,7,3,6> 
	{
	public:
		AttitudeBiasNLO(double t0, const Eigen::Vector7d &xhat0, 
						const Eigen::Vector3d &f_n, const Eigen::Vector3d &m_n, 
						double ki, double kp) 
		: NLO(t0,xhat0,ki,kp) 
		{
			Eigen::Vector4d q = xhat0.head(4);
			this->R = calc_Rquat(q);
			this->f_n = f_n;
			this->m_n = m_n;
		}

		virtual void predict(double t, const Eigen::Vector3d &u) override {}

		virtual void update(double t, const Eigen::Vector6d &z) override {}

		virtual void update(double t, const Eigen::Vector3d &u, const Eigen::Vector6d &z) override 
		{ 
			Eigen::Vector3d omega = u;
			Eigen::Vector3d f_b = z.head(3);
			f_b = f_b/f_b.norm();
			Eigen::Vector3d m_b = z.tail(3);
			m_b = m_b/m_b.norm();
			double dt = t - this->t_prev;
			this->t_prev = t;		

			double delta=1;
			Eigen::Vector3d v1b = f_b/std::max(f_b.norm(),delta);
			Eigen::Vector3d v1n = this->f_n/std::max(f_n.norm(),delta);
			Eigen::Vector3d v2b = m_b/m_b.norm();
			Eigen::Vector3d v2n = this->m_n/m_n.norm();

			Eigen::Vector3d V2b = calc_SSM(v1b)*v2b; V2b = V2b/V2b.norm();
			Eigen::Vector3d V2n = calc_SSM(v1n)*v2n; V2n = V2n/V2n.norm();
			Eigen::Vector3d V3b = calc_SSM(v1b)*calc_SSM(v1b)*v2b; V3b = V3b/V3b.norm();
			Eigen::Vector3d V3n = calc_SSM(v1n)*calc_SSM(v1n)*v2n; V3n = V3n/V3n.norm();

			Eigen::Vector3d b = this->xhat.tail(3);

			Eigen::Matrix3d J = (v1n-this->R*v1b)*v1b.transpose() + (V2n-this->R*V2b)*V2b.transpose() + (V3n-this->R*V3b)*V3b.transpose();

			Eigen::Matrix3d Rs = this->R;
		
			for (int i=0;i<9;i++) 
			{
				if (abs(Rs(i)>1))
				{
					Rs(i)=Rs(i)/abs(Rs(i));
				}
			}

			Eigen::Matrix3d tmp = Rs.transpose()*J;

			Eigen::Matrix3d stmp = 0.5*(tmp-tmp.transpose());
			Eigen::Vector3d vex; vex << 0.5*(stmp(2,1)-stmp(1,2)), 0.5*(stmp(0,2)-stmp(2,0)), 0.5*(stmp(1,0)-stmp(0,1));

			Eigen::Vector3d db = -this->ki*vex;

			Eigen::Matrix3d Rminus = this->R + dt*this->R*calc_SSM(omega-b);
			this->R = Rminus + dt*this->kp*J;
			b = b + dt*this->kp*db;

			Eigen::Vector4d q = calc_euler2q(calc_R2euler(this->R)); //This guarantees norm(q)=1

			xhat << q, b;
		
			this->xhat = xhat;
		}

	protected:
		Eigen::Matrix3d R;
		Eigen::Vector3d f_n;
		Eigen::Vector3d m_n;

	}; // end class AttitudeBiasNLO

} //namespace state_estimator

#endif
