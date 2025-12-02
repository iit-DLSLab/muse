#include <iit/commons/yaml_config/yaml_config.h>

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <fstream>

using namespace iit;
using namespace std;

ostream& operator<<(ostream& out, hydr::ValvePort::Port p ) {
    return ( out << ( p==hydr::ValvePort::A  ? "A" : "B") );
}

int main() {

    try {
        YAML::Node doc = YAML::LoadFile("../src/yaml_config/example.yaml");

        hydr::PortToVoltageSign signmap;
        hydr::ForceSignToPort forcemap;

        signmap = doc["Valve1"]["command"].as<hydr::PortToVoltageSign>();
        cout << "Valve 1, electrical command:" << endl;
        cout << "\tA: " << signmap.map[hydr::ValvePort::A] << endl;
        cout << "\tB: " << signmap.map[hydr::ValvePort::B] << endl;

        forcemap = doc["Valve1"]["actuator_force"].as<hydr::ForceSignToPort>();
        cout << "Valve 1, actuator force:" << endl;
        cout << "\tPositive force: port " << forcemap.get(hydr::U_POS) << endl;
        cout << "\tNegative force: port " << forcemap.get(hydr::U_NEG) << endl;

        commons::ctrl::pid_gains pid;
        pid = doc["PID"].as<commons::ctrl::pid_gains>();
        cout << "PID p:" << pid.p <<endl;
        cout << "PID d:" << pid.d << endl;
        cout << "PID i:" << pid.i <<endl;

    } catch(YAML::Exception& e) {
        std::cerr << std::endl;
        std::cerr << " *** WARNING: Exception during YAML parsing...: " << e.what() << std::endl;
        std::cerr << std::endl << std::endl;
    }
}



