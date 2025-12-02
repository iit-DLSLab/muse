#ifndef XKF_HPP
#define XKF_HPP

#include "EKF.hpp"
#include "NLO.hpp"

namespace state_estimator {

template <typename T, int N1, int N2, int N3, int N4, int N5, int N6, int N7>
class XKF : public EKFBase<T,N1,N2,N3,N4,N5,N6,N7> {

public:
	XKF(T t0, const Eigen::Matrix<T,N1,1> &xhat0, const Eigen::Matrix<T,N6,N6> &P0, const Eigen::Matrix<T,N7,N7> &Qi, const Eigen::Matrix<T,N5,N5> &Ri) :
	EKFBase<T,N1,N2,N3,N4,N5,N6,N7>(t0,xhat0,P0,Qi,Ri) {
		this->nlo = nullptr;
               Estimator<T,N1>::name_ = std::string("XKF");
	}

	Eigen::Matrix<T,N1,1> getNLOX() const {
		if (nlo) return this->nlo->getX(); 
		else return Eigen::Matrix<T,N1,1>::Zero();
	}

	virtual std::string DebugString() const override {
		std::stringstream ss;
		ss << "===eXogenous Kalman Filter===" << std::endl;
		ss << "t=" << this->t_prev << std::endl;
		ss << "xhat=" << this->xhat.transpose() << std::endl;
		return ss.str();
	}



protected:
	NLO<T,N1,N2,N3> *nlo;
};

} //namespace state_estimator

#endif
