#include <cmath>

#include <iit/commons/hydraulics.h>

void iit::hydr::PortToVoltageSign::set(ValvePort::Port port, VoltageSign sign)
{
    map[port] = sign;
    ValvePort::Port otherPort = (port==ValvePort::A? ValvePort::B : ValvePort::A);
    VoltageSign     otherSign = (sign==VoltageSign::Vpos? VoltageSign::Vneg : VoltageSign::Vpos);
    map[ otherPort ] = otherSign;
}

void iit::hydr::feedback_linearization_motor(
    const PipelineProperties& pipeline,
    const MotorProperties& actProps,
    const ValveProperties& valProps,
    const ActuatorState& actState,
    const Pressures& press,
    ValvePort::Port posUPort,
	[[maybe_unused]] const PortToVoltageSign& portToVolt,
    double& f, double& g)
{
    double Ps = press.ps;
    double Pt = press.pt;

    ValvePort::Port negUPort = static_cast<ValvePort::Port>((static_cast<int>(posUPort)+1)%2);

    // Let's call Pa the pressure of the chamber leading to positive torque
    double Pa = press.ports[posUPort];
    double Pb = press.ports[(static_cast<int>(posUPort)+1)%2];

    // Pipeline volumes
    double Va, Vb;
    Va = pipeline.volume + actProps.unusedVolume[posUPort] + actState.q * actProps.volumetricDisp;
    Vb = pipeline.volume - actProps.unusedVolume[negUPort] + actProps.totVolume - actState.q * actProps.volumetricDisp;

	double DeltaPa, DeltaPb;
    // The active port is the one that determines a positive torque
    if(actState.activePort == posUPort) { // Port A, in the naming convention of this code
        DeltaPa = Ps - Pa;
        DeltaPb = Pb - Pt;
    } else { // Port B
        DeltaPb = Ps - Pb;
        DeltaPa = Pa - Pt;
    }
    g = pipeline.oilBulkMod * actProps.volumetricDisp * valProps.gain_f * (
              (  std::sqrt( std::fabs(DeltaPa) ) ) / Va +
              (  std::sqrt( std::fabs(DeltaPb) ) ) / Vb
            );

    f = - pipeline.oilBulkMod * actProps.volumetricDisp * actProps.volumetricDisp *
              ( 1/Va + 1/Vb) * actState.qd;
}


void iit::hydr::feedback_linearization_cylinder(
    const PipelineProperties& pipeline,
    const CylinderProperties& actProps,
    const ValveProperties& valProps,
    const ActuatorState& actState,
    const Pressures& press,
    ValvePort::Port posUPort,
    [[maybe_unused]] const PortToVoltageSign& portToVolt,
    double& f, double& g)
{
    double Ps = press.ps;
    double Pt = press.pt;

    ValvePort::Port negUPort = static_cast<ValvePort::Port>((static_cast<int>(posUPort)+1)%2);

    // Let's call Pa the pressure of the chamber leading to positive torque
    double Pa = press.ports[posUPort];
    double Pb = press.ports[negUPort];

    // Pipeline volumes
    double Aa = actProps.pistonArea[posUPort];
    double Ab = actProps.pistonArea[negUPort];
    double Va, Vb;
    Va = pipeline.volume + actState.q * Aa;
    Vb = pipeline.volume + (actProps.stroke - actState.q) * Ab;

    double DeltaPa, DeltaPb;
    // The active port is the one that determines a positive torque
    if(actState.activePort == posUPort) { // Port A, in the naming convention of this code
        DeltaPa = Ps - Pa;
        DeltaPb = Pb - Pt;
    } else { // Port B
        DeltaPb = Ps - Pb;
        DeltaPa = Pa - Pt;
    }
    g = pipeline.oilBulkMod * valProps.gain_f * (
              ( std::sqrt( std::fabs(DeltaPa) ) ) * (Aa / Va) +
              ( std::sqrt( std::fabs(DeltaPb) ) ) * (Ab / Vb)
            );

    f = - pipeline.oilBulkMod * ( Aa*Aa/Va + Ab*Ab/Vb) * actState.qd;
}


