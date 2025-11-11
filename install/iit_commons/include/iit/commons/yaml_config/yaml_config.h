/*
 * hydraulics.h
 *
 *  Created on: Jul 7, 2014
 *      Author: mfrigerio
 */

#ifndef IIT_COMMONS_YAMLCONFIG_H_
#define IIT_COMMONS_YAMLCONFIG_H_

#include <yaml-cpp/yaml.h>

#include <iit/commons/hydraulics.h>
#include <iit/commons/control.h>

namespace YAML {

template<>
struct convert<iit::hydr::ValvePort::Port> {
    static Node encode(const iit::hydr::ValvePort::Port& port) {
      //Node node;
      throw std::runtime_error("Encode for iit::hydr::ValvePort::Port not defined");
      //return node;
    }
    static bool decode(const Node& node, iit::hydr::ValvePort::Port& port) {
      if(!node.IsDefined()) {
        return false;
      }
      std::string p;
      p = node["port"].as<std::string>();
      if(p.compare("A") == 0) {
          port = iit::hydr::ValvePort::A;
      } else if(p.compare("B") == 0) {
          port = iit::hydr::ValvePort::B;
      } else {
          throw std::runtime_error("Invalid value for configuration item 'port' : " + p);
      }
      return true;
    }
};

template<>
struct convert<iit::hydr::VoltageSign> {
    static Node encode(const iit::hydr::VoltageSign& sign) {
      //Node node;
      throw std::runtime_error("Encode for iit::hydr::VoltageSign not defined");
      //return node;
    }
    static bool decode(const Node& node, iit::hydr::VoltageSign& sign) {
      if(!node.IsDefined()) {
        return false;
      }
      std::string v;
      v = node["sign"].as<std::string>();
      if(v.compare("Vpos") == 0) {
          sign = iit::hydr::VoltageSign::Vpos;
      } else if(v.compare("Vneg") == 0) {
          sign = iit::hydr::VoltageSign::Vneg;
      } else {
          throw std::runtime_error("Invalid value for configuration item 'sign' : " + v);
      }
      return true;
    }
};

template<>
struct convert<iit::hydr::PortToVoltageSign> {
    static Node encode(const iit::hydr::PortToVoltageSign& map) {
      //Node node;
      throw std::runtime_error("Encode for iit::hydr::PortToVoltageSign not defined");
      //return node;
    }
    static bool decode(const Node& node, iit::hydr::PortToVoltageSign& map) {
      if(!node.IsDefined()) {
        return false;
      }
      iit::hydr::VoltageSign sign;
      iit::hydr::ValvePort::Port port;
      sign = node.as<iit::hydr::VoltageSign>();
      port = node.as<iit::hydr::ValvePort::Port>();
      map.set(port, sign);
      return true;
    }
};

template<>
struct convert<iit::hydr::ForceSign> {
    static Node encode(const iit::hydr::ForceSign& sign) {
      //Node node;
      throw std::runtime_error("Encode for iit::hydr::ForceSign not defined");
      //return node;
    }
    static bool decode(const Node& node, iit::hydr::ForceSign& sign) {
      if(!node.IsDefined()) {
        return false;
      }
      std::string v;
      v = node["sign"].as<std::string>();
      if(v.compare("Fpos") == 0) {
          sign = iit::hydr::ForceSign::U_POS;
      } else if(v.compare("Fneg") == 0) {
          sign = iit::hydr::ForceSign::U_NEG;
      } else {
          throw std::runtime_error("Invalid value for configuration item 'sign' (force sign): " + v);
      }
      return true;
    }
};

template<>
struct convert<iit::hydr::ForceSignToPort> {
    static Node encode(const iit::hydr::ForceSignToPort& map) {
      //Node node;
      throw std::runtime_error("Encode for iit::hydr::ForceSignToPort not defined");
      //return node;
    }
    static bool decode(const Node& node, iit::hydr::ForceSignToPort& map) {
      if(!node.IsDefined()) {
        return false;
      }
      iit::hydr::ForceSign sign;
      iit::hydr::ValvePort::Port port;
      sign = node.as<iit::hydr::ForceSign>();
      port = node.as<iit::hydr::ValvePort::Port>();
      map.set(sign, port);
      return true;
    }
};

template<>
struct convert<iit::hydr::LinearizationGains> {
    static Node encode(const iit::hydr::LinearizationGains& g) {
      //Node node;
      throw std::runtime_error("Encode for iit::hydr::LinearizationGains not defined");
      //return node;
    }
    static bool decode(const Node& node, iit::hydr::LinearizationGains& g) {
      if(!node.IsDefined()) {
        return false;
      }
      g.kvc = node["kvc"].as<double>();
      g.kpc = node["kpc"].as<double>();
      return true;
    }
};

template<>
struct convert<iit::commons::ctrl::pid_gains> {
    static Node encode(const iit::commons::ctrl::pid_gains& g) {
      //Node node;
      throw std::runtime_error("Encode for iit::commons::ctrl::pid_gains not defined");
      //return node;
    }
    static bool decode(const Node& node, iit::commons::ctrl::pid_gains& g) {
      if(!node.IsDefined()) {
        return false;
      }
      g.p = node["p"].as<double>();
      g.i = node["i"].as<double>();
      g.d = node["d"].as<double>();
      return true;
    }
};

} // namespace 'YAML'

#endif /* include guard */
