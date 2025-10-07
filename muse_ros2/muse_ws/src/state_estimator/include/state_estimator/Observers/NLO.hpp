#ifndef NLO_HPP
#define NLO_HPP

#include "Observer.hpp"

namespace state_estimator {

template <typename T, int N, int M, int O>
class NLO : public Observer<T,N,M,O> {

public:
	NLO(T t0, const Eigen::Matrix<T,N,1> &xhat0, T ki, T kp) : Observer<T,N,M,O> (t0,xhat0) {
                Estimator<T,N>::name_ = std::string("NLO");
		this->ki=ki;
		this->kp=kp;
	}

	virtual std::string DebugString() {
		std::stringstream ss;
		ss << "===Nonlinear Observer===" << std::endl;
		ss << "t=" << this->t_prev << std::endl;
		ss << "xhat=" << this->xhat.transpose() << std::endl;
		return ss.str();
	}


protected:
	T ki;
	T kp;
};

} //namespace state_estimator

#endif

