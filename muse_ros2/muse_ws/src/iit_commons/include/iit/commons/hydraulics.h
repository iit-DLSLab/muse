#ifndef IIT_COMMONS_HYDRAULICS_H
#define IIT_COMMONS_HYDRAULICS_H

#include <iostream>
#include <math.h>


namespace iit {
namespace hydr {


struct ValvePort {
    enum Port { A=0, B=1};
    static const int _PORTS_COUNT = 2;
};


inline int sign(double val) {
	return val>0? +1 : -1;
}

typedef struct pressures {
    double ports[ValvePort::_PORTS_COUNT];
    double ps;
    double pt;

    pressures() : ps(0.0), pt(0.0) {
        ports[ValvePort::A] = 0.0;
        ports[ValvePort::B] = 0.0;
    }
} Pressures;


typedef struct pipelineProperties {
    double oilBulkMod; // Pa
    /**
     * The total volume of the hoses connected to the actuator.
     */
    double volume; // m³
} PipelineProperties;


typedef struct motorProperties {

    double volumetricDisp; // m³/rad
    double totVolume;      // m³
    /**
     * This value should include any part of the internal volume
     * of the actuator that is not covered during motion due to
     * e.g. an external mechanical end-stop. This value accounts only for the
     * unused volume of ONE chamber (and it assumes the two chambers have
     * symmetric unused volumes).
     */
  //  double unusedVolume;   // m³
    /*
     * To calculate non symmetric unused volumes
     * */
    double unusedVolume[ValvePort::_PORTS_COUNT]; // m³
} MotorProperties;

typedef struct cylinderProperties {
    double pistonArea[ValvePort::_PORTS_COUNT];     // m²
    double stroke;  // m
} CylinderProperties;



typedef struct valveProperties {
	/**
	 * The valve sizing gain.
	 * Flow / (pressure drop * input signal to the valve)
	 * See Boaventura's PhD thesis 2013
	 */
	double gain;
	double gain_f;
	double gain_high;
	double gain_low;
	double transition_point;
	double dt;
	double alpha;

	void updateValveProperties(double uv)
	{
//		if ((uv >=transition_point)|| (uv <=-transition_point))
//		{
//			gain = gain_high;
//		}else {
//			gain = gain_low;
//		}

		double offset = 0.18;
		double band = 30.0;
		double sigmoid = 1.0/(1.0 +  exp(-band*(fabs(uv)- offset)));
		alpha = dt/(dt + 1/(2*M_PI*100.0));
		gain = gain_low *(1 - sigmoid) + gain_high *sigmoid;
		gain_f = alpha*gain + (1-alpha)*gain_f;
	}
} ValveProperties;



typedef struct actuatorState {
    double q; //TODO caller need to normalize this from the actuator perspective (q=0 -> the chamber leading to positive torque is empty. E.g. for a cylinder this typically means cylinder completely retracted)
    double qd;
    ValvePort::Port activePort;
} ActuatorState;


enum VoltageSign {
    Vpos=1, Vneg=-1
};

/** Simply two identifiers for positive and negative force */
enum ForceSign {U_POS=0, U_NEG=1};

struct PortToVoltageSign {
    VoltageSign map[ValvePort::_PORTS_COUNT];

    /**
     * Associates the given port to the given sign, and automatically
     * sets the opposite sign for the other port
     */
    void set(ValvePort::Port port, VoltageSign sign);
};

/**
 * A container to associate a hydraulic port with the sign of force.
 */
struct ForceSignToPort {
    ValvePort::Port ports[2];
    /**
     *  Associates a valve port to the sign of the joint force.
     *  The other port is automatically assigned to the opposite sign.
     */
    inline void set(ForceSign tausign, ValvePort::Port p) {
        ports[tausign] = p;
        ForceSign other_sign = (tausign == U_POS? U_NEG : U_POS);
        ports[other_sign] = (p == ValvePort::A? ValvePort::B : ValvePort::A);
    }
    inline ValvePort::Port get(ForceSign tausign) const {
        return ports[tausign];
    }
    /*!
     *
     * The other port is automatically assigned to the opposite one.
     */
    inline void setPositiveForcePort(ValvePort::Port p) {
        ports[U_POS] = p;
        ports[U_NEG] = (p==ValvePort::A? ValvePort::B : ValvePort::A);
    }
    /*!
     *
     * The other port is automatically assigned to the opposite one.
     */
    inline void setNegativeForcePort(ValvePort::Port p) {
        ports[U_NEG] = p;
        ports[U_POS] = (p==ValvePort::A? ValvePort::B : ValvePort::A);
    }
    inline ValvePort::Port getPositiveForcePort() const {
        return ports[U_POS];
    }
    inline ValvePort::Port getNegativeForcePort() const {
        return ports[U_NEG];
    }
};


typedef struct hydDynTerms {
	double f;
	double g;

	hydDynTerms(double val) : f(val), g(val) {}
	hydDynTerms() : f(0.0), g(0.0) {}
} HydDynTerms;


typedef struct linearizationTerms {
    double uvc; ///< velocity compensation term
    double upc; ///< pressure compensation term

    linearizationTerms(double val) : uvc(val), upc(val) {}
    linearizationTerms() : uvc(0.0), upc(0.0) {}
} LinearizationTerms;

typedef struct linearizationGains {
    double kvc; ///< velocity compensation gain
    double kpc; ///< pressure compensation gain

    linearizationGains(double val) : kvc(val), kpc(val) {}
    linearizationGains(double vc, double pc) : kvc(vc), kpc(pc) {}
    linearizationGains() : kvc(0.0), kpc(0.0) {}
    linearizationGains(const linearizationGains& rhs) : kvc(rhs.kvc), kpc(rhs.kpc) {}
} LinearizationGains;

//TODO
typedef struct hydraulicState {
    double             force;
    double             forceRef;
//TODO
    ActuatorState      actuator;
    HydDynTerms        hydDynTerms;
    LinearizationTerms linTerms;
    Pressures          pressures;
} HydraulicState;

/**
 *
 * @param props
 * @param portA
 * @param portB
 * @param hydrForce the force measured by the load cell (it ignores the friction forces).
 *                  The value must be positive when the cylinder is pushing
 *                  (i.e. the active port is the 'portA' argument)
 * @param pressures
 */
inline void cylinderChamberPressures(
        const CylinderProperties& props, ValvePort::Port portA,
        ValvePort::Port portB, double hydrForce, Pressures& pressures)
{
    double Aa = props.pistonArea[portA];
    double Ab = props.pistonArea[portB];

    pressures.ports[portA] = (hydrForce + Ab * (pressures.ps - pressures.pt)) / (Aa + Ab);
    pressures.ports[portB] = pressures.ps - pressures.pt - pressures.ports[portA];
}


/**
 * TODO!!
 */
inline void motorChamberPressures(
        const MotorProperties& props,
        ValvePort::Port portA,
        ValvePort::Port portB,
        double tau,
        Pressures& pressures)
{
    pressures.ports[portA] = (tau + props.volumetricDisp * (pressures.ps - pressures.pt)) / (2*props.volumetricDisp);
    pressures.ports[portB] = pressures.ps - pressures.pt - pressures.ports[portA];
}


/**
 * Computes the feedback-linearization terms of the hydraulic dynamics,
 * given data about the dynamics state and the linearization gains.
 * TODO parameters
 */
inline void computeLinearizationTerms(
    const HydDynTerms& hydTerms,
    const LinearizationGains& gains,
    LinearizationTerms& output)
{
    output.upc = gains.kpc / hydTerms.g;
    output.uvc = gains.kvc * hydTerms.f * output.upc;
}

/**
 * Applies the feedback-linearization terms of the hydraulic dynamics
 * to a given control command.
 * TODO the semantic of such control command
 */
inline double applyLinearizationTerms(
        const LinearizationTerms& terms, const double& currentCommand)
{
    return (currentCommand * terms.upc) - terms.uvc;
}

/**
 * Applies the feedback-linearization terms of the hydraulic dynamics
 * to a given control command.
 * TODO the semantic of such control command
 */
inline double applyLinearizationTerms(
		ValveProperties& valProps,
        const LinearizationTerms& terms,
        const double& currentCommand)
{
	double uv = (currentCommand * terms.upc) - terms.uvc;
	//update valve gain
	valProps.updateValveProperties(uv);
    return uv;
}

void feedback_linearization_motor(
    const PipelineProperties&,
    const MotorProperties&,
    const ValveProperties&,
    const ActuatorState&,
    const Pressures&,
    ValvePort::Port positiveTorquePort,
    const PortToVoltageSign& portToVolt,
    double& f, double& g);

void feedback_linearization_cylinder(
    const PipelineProperties&,
    const CylinderProperties&,
    const ValveProperties&,
    const ActuatorState&,
    const Pressures&,
    ValvePort::Port positiveTorquePort,
    const PortToVoltageSign& portToVolt,
    double& f, double& g);



}
}




#endif /* IIT_HYD_COMMONS_H */
