/*
 * control.h
 *
 *  Created on: May 21, 2013
 *      Author: mfrigerio
 */

#ifndef IIT_COMMONS_CONTROL_H_
#define IIT_COMMONS_CONTROL_H_

#include <iostream>

namespace iit {
namespace commons {

/**
 * TODO!!
 * @param x
 * @param xprev
 * @param dt
 * @param filterCoeff
 * @return
 */
inline double filteredDerivative(
        const double& x,
        const double& xprev,
        const double& dt,
        const double& filterCoeff) {
    return (1-filterCoeff) * xprev + filterCoeff * (x - xprev)/dt;
}


namespace ctrl {



typedef struct pidgains{
        double p;
        double i;
        double d;
        pidgains(double val) : p(val), i(val), d(val) {}
        pidgains(double pg, double ig, double dg) : p(pg), i(ig), d(dg) {}
        pidgains() : p(0), i(0), d(0) {}
} pid_gains;



typedef struct pidterms {
    double P;
    double I;
    double D;
    pidterms(double val) : P(val), I(val), D(val) {}
    pidterms() : P(0), I(0), D(0) {}

    double sumTerms() const { return P+I+D; }
    //TODO define other operators
} pid_terms;




typedef struct piderrs {
    double current;
    double previous;

    piderrs(double val) : current(val), previous(val) {}
    piderrs() : current(0), previous(0) {}

    /**
     * Sets the previous error with the value of the current one
     */
    void updatePrevious() { previous = current; }
    //TODO define other operators
} pid_errors;



/**
 * A simple class for PID control of a single variable.
 * Instances of this class are immutable. The state is limited to the timing
 * constants required to compute the integral term and to filter the derivative
 * term.
 */
class PIDControl {

public:
    PIDControl(double ts, double tf);
    PIDControl(const PIDControl&);

    double pid( //TODO the name should tell the type of controller (there are variants of PID)
            const pid_gains& gains,
            const pid_errors& err,
            pid_terms& previousTerms);
    void resetIntegrator(pid_terms& previousTerms);

    double getTs()    const { return Ts; }
    double getTf()    const { return Tf; }
    double getAlpha() const { return alpha; }

private:
    double Ts; ///< filter time constant TODO: be more precise, ask Michele
    double Tf; ///< filter time constant TODO: be more precise, ask Michele
    double alpha;
};


inline PIDControl::PIDControl(double ts, double tf) :
        Ts(ts), Tf(tf), alpha(Ts/(Tf+Ts)) {}

inline PIDControl::PIDControl(const PIDControl& copyMe) :
        Ts(copyMe.Ts), Tf(copyMe.Tf), alpha(copyMe.alpha) {}

inline double PIDControl::pid(
        const pid_gains& gains,
        const pid_errors& err,
        pid_terms& previousTerms)
{
    // Proportional term
    previousTerms.P = gains.p * err.current;
    // Integral term
    previousTerms.I = previousTerms.I + gains.i * Ts * (err.previous + err.current)/2;
    // Derivative term with first order filter (filter constant time Tf)
    previousTerms.D = (1-alpha)*previousTerms.D + alpha * gains.d * (err.current - err.previous )/Ts;

    return previousTerms.sumTerms();
}

inline void PIDControl::resetIntegrator(pid_terms& previousTerms)
{
    // Reset Integral term
    previousTerms.I = 0.0;
}





inline std::ostream& operator<< (std::ostream& out, const pid_gains& g) {
    return out << "p: " << g.p << " i: " << g.i << " d: " << g.d;
}



} // namespace 'control'

}

}


#endif /* IIT_COMMONS_CONTROL_H_ */
