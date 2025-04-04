#ifndef OBSERVER_HPP
#define OBSERVER_HPP

// #include <Eigen/Dense>
#include <math.h>
#include "state_estimator/EigenExtra.hpp"
#include "state_estimator/Estimator.hpp"

namespace state_estimator {

template <typename T, unsigned int N, unsigned int M, unsigned int O> class Observer : public Estimator<T,N> {

public:
	Observer(T t0, const Eigen::Matrix<T,N,1> &xhat0) {
		this->t_prev=t0;
		this->xhat=xhat0;
                Estimator<T,N>::name_ = std::string("Observer");
	}

	virtual void predict(T t, const Eigen::Matrix<T,M,1> &u)=0;

	virtual void update(T t, const Eigen::Matrix<T,O,1> &z)=0;

	virtual void update(T t, const Eigen::Matrix<T,M,1> &u, const Eigen::Matrix<T,O,1> &z) {
		this->predict(t,u);
		this->update(t,z);
	}

	virtual void updateGen(double t, Eigen::MatrixXd &u, Eigen::MatrixXd &z) override {
		Eigen::Matrix<T,M,1> u2 = u;
		Eigen::Matrix<T,O,1> z2 = z;
		update(t,u2,z2);	
	}

	virtual unsigned int getNumStates() const override { return N; }
	virtual unsigned int getNumMeasurments() const override { return M; }
	virtual unsigned int getNumOutputs() const override { return O; }

    Eigen::Matrix<T,N,1> getX() const {
        return xhat; }

	virtual Eigen::MatrixXd getXgen() const override {
		Eigen::MatrixXd ret;
		ret = this->xhat;
		return ret;
	}



	T getT() const { return t_prev; }

	virtual std::string DebugString() const {
		std::stringstream ss;
		ss << "===Observer===" << std::endl;
		ss << "t=" << this->t_prev << std::endl;
		ss << "xhat=" << this->xhat.transpose() << std::endl;
		return ss.str();
	}


protected:
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW ?
	Eigen::Matrix<T,N,1> xhat;
	T t_prev;

};

} //namespace state_estimator


#endif

