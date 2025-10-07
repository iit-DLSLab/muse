#ifndef ATTITUDEBIASXKF_HPP
#define ATTITUDEBIASXKF_HPP

#include "state_estimator/Observers/XKF.hpp"
#include "state_estimator/Models/attitude_bias_NLO.hpp"
#include <unsupported/Eigen/MatrixFunctions>
#include <gtest/gtest_prod.h>

namespace state_estimator {

	class AttitudeBiasXKF : public XKF<double,7,3,6,7,6,6,6> 
	{
	public:
		AttitudeBiasXKF(double t0, const Eigen::Vector7d &xhat0, 
						const Eigen::Matrix6d &P0, const Eigen::Matrix6d &Q, const Eigen::Matrix6d &R, 
						const Eigen::Vector3d &f_n, const Eigen::Vector3d &m_n, 
						double ki, double kp) 
		: XKF<double,7,3,6,7,6,6,6>(t0,xhat0,P0,Q,R) 
		{
			q_bar_prev = xhat0.head(4);
			this->nlo = new AttitudeBiasNLO(t0,xhat0,f_n,m_n,ki,kp);
			this->f_n = f_n;
			this->m_n = m_n;
			this->q_bar_prev << xhat0.head(4);
			this->x_bar = xhat0;
		}

		~AttitudeBiasXKF() 
		{
			if (this->nlo!=nullptr) delete(this->nlo);
		}

		//Time update
		virtual void predict(double t, const Eigen::Vector3d &u) override 
		{
			double dt = t-this->t_prev;

			Eigen::Vector7d f = calc_f(t,this->x_bar,u);
			Eigen::Matrix7d Fx = calc_F(t,this->x_bar,u);
			Eigen::Vector7d xdot = f + Fx*(this->xhat-this->x_bar);

			Eigen::Matrix6d F = calc_F(t,this->xhat,this->x_bar,u);
			Eigen::Matrix6d G = calc_G(t,this->xhat,this->x_bar,u);

			Eigen::Matrix<double,12,12> M;
			M.block(0,0,6,6) = F*dt;
			M.block(0,6,6,6) = G*dt;
			M.block(6,0,6,12) = Eigen::Matrix<double,6,12>::Zero();
			Eigen::Matrix<double,12,12> s = M.exp();
			Eigen::Matrix<double,6,6> Phi = s.block(0,0,6,6);
			Eigen::Matrix<double,6,6> Gamma = s.block(0,6,6,6);
			this->P = Phi*this->P*Phi.transpose() + Gamma*this->Q*Gamma.transpose();  //Zero order hold

			this->xhat = this->xhat + xdot*dt;

			this->renormalizeQ();

			this->fixP();

			this->t_prev=t;

		}

		//Measurement update
		virtual void update(double t, const Eigen::Vector6d &zc) override 
		{
			Eigen::Vector6d z;
			z.block(0,0,3,1) =  zc.head(3)/zc.head(3).norm();
			z.block(3,0,3,1) =  zc.tail(3)/zc.tail(3).norm();

			Eigen::Vector7d xbar = this->x_bar;

			Eigen::Matrix3d R_bar = calc_Rquat(this->x_bar.head(4));
			Eigen::Vector3d f_bar = R_bar.transpose()*this->f_n;
			Eigen::Vector3d m_bar = R_bar.transpose()*this->m_n;

			Eigen::Vector6d y_tilde; y_tilde = z;
			Eigen::Vector6d zbar;
			zbar.block(0,0,3,1) = R_bar.transpose()*this->f_n/this->f_n.norm();
			zbar.block(3,0,3,1) = R_bar.transpose()*this->m_n/this->m_n.norm();

			Eigen::Matrix<double,4,3> Psi_bar = calc_Psi(this->x_bar.head(4));
			Eigen::Matrix<double,4,3> Psi_hat = calc_Psi(this->xhat.head(4));

			Eigen::Matrix<double,6,7> Hz = calc_H(t,this->x_bar);
			Eigen::Matrix6d H = calc_H(t,this->xhat, this->x_bar);

			Eigen::Matrix6d K = P*H.transpose()*(H*P*H.transpose()+this->R).inverse();

			Eigen::Matrix<double,7,6> M; //Mapping from additive to multiplicate error state
			M.block(0,0,4,3) = Psi_bar+Psi_hat;
			M.block(0,3,4,3) = Eigen::Matrix<double,4,3>::Zero();
			M.block(4,0,3,3) = Eigen::Matrix<double,3,3>::Zero();
			M.block(4,3,3,3) = Eigen::Matrix<double,3,3>::Identity();
			Eigen::Vector7d hb = this->xhat-this->x_bar;
			y_tilde = z-zbar-Hz*hb;
			Eigen::Vector7d z_tilde; z_tilde = M*K*y_tilde;

			this->xhat = this->xhat + z_tilde;
			this->renormalizeQ();
			this->P = (Eigen::Matrix6d::Identity()-K*H)*this->P;
			this->fixP();
		}


		virtual void update(double t, const Eigen::Vector3d &u, const Eigen::Vector6d &z) override 
		{
			(static_cast<AttitudeBiasNLO*>(this->nlo))->update(t,u,z);
			this->x_bar = this->nlo->getX();

			// Ensure sign of qbar, StoJohFos18 (12)
			checkQSign(); 

			//reset rule, StoJohFos18 Lemma 5
			if (this->x_bar.head(4).transpose()*this->xhat.head(4) < 0.001) {
				this->xhat = this->x_bar;
			}
			
			Observer::update(t,u,z); //call predict and update
		}

		Eigen::Vector7d calc_f(double t, const Eigen::Vector7d &x, const Eigen::Vector3d &u) 
		{

			Eigen::Vector4d q = x.head(4);
			Eigen::Vector3d b = x.tail(3);

			Eigen::Matrix<double,4,4> Omega = calc_Omega(u-b);

			Eigen::Vector4d qdot = 0.5*Omega*q;
			Eigen::Vector3d bdot = Eigen::Vector3d::Zero();

			Eigen::Vector7d xdot;
			xdot << qdot, bdot;
			return xdot;
		}

		Eigen::Vector6d calc_h(double t, const Eigen::Vector7d &x) 
		{
			Eigen::Vector6d h;
			Eigen::Vector4d q = x.head(4);
			Eigen::Vector3d b = x.tail(3);

			Eigen::Matrix3d Rnb = calc_Rquat(q);
			Eigen::Matrix3d Rbn = Rnb.transpose();
		
			Eigen::Vector3d y_1 = Rbn*this->f_n/this->f_n.norm();
			Eigen::Vector3d y_2 = Rbn*this->m_n/this->m_n.norm();
			h << y_1, y_2;
			return h;
		}

		Eigen::Matrix7d calc_F(double t, const Eigen::Vector7d &x, const Eigen::Vector3d &u) 
		{
			Eigen::Matrix7d F;
			Eigen::Vector3d omega = u;
			Eigen::Vector4d q = x.head(4);
			Eigen::Vector3d b = x.tail(3);

			Eigen::Matrix<double,4,3> Xi = calc_Xi(q);
			Eigen::Matrix<double,4,4> Omega = calc_Omega(omega-b);
			F.block(0,0,4,4) = 0.5*Omega;
			F.block(0,4,4,3) = -0.5*Xi;
			F.block(4,0,3,7) = Eigen::Matrix<double,3,7>::Zero();
			return F;
		}

		Eigen::Matrix6d calc_F(double t, const Eigen::Vector7d &xhat, const Eigen::Vector7d &xbar, const Eigen::Vector3d &u) 
		{
			Eigen::Matrix6d F;
			Eigen::Vector4d qhat = xhat.head(4);
			Eigen::Vector3d bhat = xhat.tail(3);
			Eigen::Vector4d qbar = xbar.head(4);
			Eigen::Vector3d bbar = xbar.tail(3);
			Eigen::Vector3d omega = u;

			Eigen::Matrix3d R_bar = calc_Rquat(qbar);
			Eigen::Matrix<double,4,3> Psi_bar = calc_Psi(qbar);
			Eigen::Matrix<double,4,3> Psi_hat = calc_Psi(qhat);
			Eigen::Matrix<double,4,4> Omega_b = calc_Omega(bhat-bbar);
			Eigen::Vector4d dq = calc_qmult(calc_quatinv(qhat),qbar);
			Eigen::Vector3d du = dq.tail(3)/(1+dq(0));
			Eigen::Matrix3d D1 = Eigen::Matrix3d::Identity()-calc_SSM(du);
			Eigen::Matrix3d F1 = (Psi_hat+Psi_bar).transpose()*Omega_b*Psi_hat/(4*(1+dq(0)));
			Eigen::Matrix3d F2 = -D1*R_bar/4;
			F.block(0,0,3,3) = F1;
			F.block(0,3,3,3) = F2;
			F.block(3,0,3,6) = Eigen::Matrix<double,3,6>::Zero();

			return F;
		}

		Eigen::Matrix6d calc_G(double t, const Eigen::Vector7d &xhat, const Eigen::Vector7d &xbar, const Eigen::Vector3d &u) 
		{
			Eigen::Matrix6d G;
			Eigen::Vector4d qhat = xhat.head(4);
			Eigen::Vector3d bhat = xhat.tail(3);
			Eigen::Vector4d qbar = xbar.head(4);
			Eigen::Vector3d bbar = xbar.tail(3);
			Eigen::Vector3d omega = u;

			Eigen::Matrix3d R_bar = calc_Rquat(qbar);
			Eigen::Vector4d dq = calc_qmult(calc_quatinv(qhat),qbar);
			Eigen::Vector3d du = dq.tail(3)/(1+dq(0));
			Eigen::Matrix3d D1 = Eigen::Matrix3d::Identity()-calc_SSM(du);
			G.block(0,0,3,3) = -0.25*D1*R_bar;
			G.block(0,3,3,3) = Eigen::Matrix3d::Zero();
			G.block(3,0,3,3) = Eigen::Matrix3d::Zero();
			G.block(3,3,3,3) = Eigen::Matrix3d::Identity();
	
			return G;
		}

		Eigen::Matrix6d calc_H(double t, const Eigen::Vector7d &xhat, const Eigen::Vector7d &xbar) 
		{
			Eigen::Matrix6d H;
			Eigen::Vector4d qhat = xhat.head(4);
			Eigen::Vector4d qbar = xbar.head(4);

			Eigen::Matrix3d R_bar = calc_Rquat(qbar);
			Eigen::Vector3d fn = this->f_n/this->f_n.norm();
			Eigen::Vector3d mn = this->m_n/this->m_n.norm();
			Eigen::Vector4d dq = calc_qmult(calc_quatinv(qhat),qbar);
			Eigen::Vector3d du = dq.tail(3)/(1+dq(0));
			Eigen::Matrix3d D1 = Eigen::Matrix3d::Identity()-calc_SSM(du);
			Eigen::Matrix3d D = D1*(1+dq(0));
			H.block(0,0,3,3) = 2*R_bar.transpose()*calc_SSM(fn)*D.transpose();
			H.block(3,0,3,3) = 2*R_bar.transpose()*calc_SSM(mn)*D.transpose();
			H.block(0,3,6,3) = Eigen::Matrix<double,6,3>::Zero();
			return H;
		}

		virtual Eigen::Matrix<double,6,7> calc_H(double t, const Eigen::Vector7d &x) 
		{
			Eigen::Matrix<double,6,7> H;
			Eigen::Vector4d q = x.head(4);
			Eigen::Matrix3d R = calc_Rquat(q);
			Eigen::Matrix3d Rt = R.transpose();
			Eigen::Vector3d f = Rt*this->f_n/this->f_n.norm();
			Eigen::Vector3d m = Rt*this->m_n/this->m_n.norm();
			Eigen::Matrix<double,4,3> Xi = calc_Xi(q);
			H.block(0,0,3,4) = 2*calc_SSM(f)*Xi.transpose();
			H.block(3,0,3,4) = 2*calc_SSM(m)*Xi.transpose();
			H.block(0,4,6,3) = Eigen::Matrix<double,6,3>::Zero();
			return H;
		}

		void checkQSign()
		{
			Eigen::Vector4d q_bar = this->x_bar.head(4);
			Eigen::Vector3d b_bar = this->x_bar.tail(3);
			if (this->q_bar_prev.transpose()*q_bar < 0) 
			{
				this->x_bar << -q_bar, b_bar;
			}
			this->q_bar_prev = this->x_bar.head(4);
		}

		inline Eigen::Matrix<double,4,3> calc_Xi(const Eigen::Vector4d &q) 
		{
			Eigen::Matrix<double,4,3> xi;
			xi << -q(1), -q(2), -q(3), q(0)*Eigen::Matrix3d::Identity()+calc_SSM(q.tail(3));
			return xi;
		}

		inline Eigen::Matrix<double,4,3> calc_Psi(const Eigen::Vector4d &q) 
		{
            Eigen::Matrix<double,4,3> psi;
			Eigen::Matrix<double,3,3> tmp = q(0)*Eigen::Matrix3d::Identity()-calc_SSM(q.tail(3));
			psi << -q(1), -q(2), -q(3),
					tmp(0,0), tmp(0,1),tmp(0,2),
					tmp(1,0), tmp(1,1),tmp(1,2),
					tmp(2,0), tmp(2,1),tmp(2,2);
			return psi;
		}

		inline Eigen::Matrix<double,4,4> calc_Omega(const Eigen::Vector3d &w) 
		{
			Eigen::Matrix<double,4,4> omega;
			omega << 0, -w(0), -w(1), -w(2), w, -calc_SSM(w);
			return omega;
		}

		Eigen::Vector4d q_bar_prev;
		Eigen::Vector7d x_bar;
		Eigen::Vector3d f_n;
		Eigen::Vector3d m_n;

	public:

		void renormalizeQ() {
			Eigen::Vector4d q = this->xhat.head(4)/this->xhat.head(4).norm();
			for (int i=0;i<4;i++)
			{
				this->xhat(i) = q(i);
			}
		}

	}; // end class AttitudeBiasXKF

} // end namespace state_estimator

#endif