#ifndef IIT_DOG_KIN_DYN_PARAMS_H_
#define IIT_DOG_KIN_DYN_PARAMS_H_

namespace iit {
namespace dog {


class KinDynParams
{

public:
    virtual void resetDefaults() = 0;

    //getters
    virtual double getValue_robot_total_mass() const = 0;
    virtual double getValue_trunk_mass() const = 0;

    virtual double getValue_trunk_com_x() const = 0;
    virtual double getValue_trunk_com_y() const = 0;
    virtual double getValue_trunk_com_z() const = 0;

    virtual double getValue_trunk_Ix() const = 0;
    virtual double getValue_trunk_Iy() const = 0;
    virtual double getValue_trunk_Iz() const = 0;
    virtual double getValue_trunk_Ixy() const = 0;
    virtual double getValue_trunk_Ixz() const = 0;
    virtual double getValue_trunk_Iyz() const = 0;


    virtual void setValue_robot_total_mass(double val) = 0;
    virtual void setValue_trunk_mass(double val)  = 0;

    virtual void setValue_trunk_com_x(double val)  = 0;
    virtual void setValue_trunk_com_y(double val)  = 0;
    virtual void setValue_trunk_com_z(double val)  = 0;

    virtual void setValue_trunk_Ix(double val)  = 0;
    virtual void setValue_trunk_Iy(double val)  = 0;
    virtual void setValue_trunk_Iz(double val)  = 0;
    virtual void setValue_trunk_Ixy(double val)  = 0;
    virtual void setValue_trunk_Ixz(double val)  = 0;
    virtual void setValue_trunk_Iyz(double val)  = 0;

    virtual double getValue_foot_x() const = 0;

    virtual void setValue_foot_x(double val) = 0;

    virtual double getValue_LF_shin() const = 0;
    virtual double getValue_RF_shin() const = 0;
    virtual double getValue_LH_shin() const = 0;
    virtual double getValue_RH_shin() const = 0;

    virtual void setValue_LF_shin(double val) = 0;
    virtual void setValue_RF_shin(double val) = 0;
    virtual void setValue_LH_shin(double val) = 0;
    virtual void setValue_RH_shin(double val) = 0;

    virtual double getValue_haa_x() const = 0;
    virtual double getValue_haa_y() const = 0;
    virtual double getValue_haa_z() const = 0;

    virtual double getValue_haa_hfe() const = 0;
    virtual double getValue_upper_leg() const = 0;
    virtual double getValue_lower_leg() const = 0;

    virtual void setValue_haa_x(double val) = 0;
    virtual void setValue_haa_y(double val) = 0;
    virtual void setValue_haa_hfe(double val) = 0;
    virtual void setValue_upper_leg(double val) = 0;
    virtual void setValue_lower_leg(double val) = 0;
};


}
}
#endif
