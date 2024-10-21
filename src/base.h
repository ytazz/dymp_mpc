#pragma once

#pragma warning(disable: 4251)
#pragma warning(disable: 4275)

#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <Eigen/Eigen>

namespace dymp{
namespace mpc{

inline void ReadInt(int& v, const YAML::Node& node){
    if(!node.IsDefined() || !node.IsScalar())
        return;

    v = node.as<int>();
}

inline void ReadBool(bool& v, const YAML::Node& node){
    if(!node.IsDefined() || !node.IsScalar())
        return;

    v = node.as<bool>();
}

inline void ReadDouble(double& v, const YAML::Node& node){
    if(!node.IsDefined() || !node.IsScalar())
        return;

    v = node.as<double>();
}

inline void ReadString(std::string& s, const YAML::Node& node){
    if(!node.IsDefined() || !node.IsScalar())
        return;

    s = node.as<std::string>();
}

inline void ReadVector2(Eigen::Vector2d& v, const YAML::Node& node){
    if(!node.IsDefined() || !node.IsSequence() || node.size() != 2)
        return;

    v[0] = node[0].as<double>();
    v[1] = node[1].as<double>();
}

inline void ReadVector3(Eigen::Vector3d& v, const YAML::Node& node){
    if(!node.IsDefined() || !node.IsSequence() || node.size() != 3)
        return;

    v[0] = node[0].as<double>();
    v[1] = node[1].as<double>();
    v[2] = node[2].as<double>();
}

inline void ReadMatrix3(Eigen::Matrix3d& m, const YAML::Node& node){
    if(!node.IsDefined() || !node.IsSequence() || node.size() != 9)
        return;

    m(0, 0) = node[0].as<double>();
    m(0, 1) = node[1].as<double>();
    m(0, 2) = node[2].as<double>();
    m(1, 0) = node[3].as<double>();
    m(1, 1) = node[4].as<double>();
    m(1, 2) = node[5].as<double>();
    m(2, 0) = node[6].as<double>();
    m(2, 1) = node[7].as<double>();
    m(2, 2) = node[8].as<double>();
}

inline void ReadVectorInt(std::vector<int>& v, const YAML::Node& node){
    if(!node.IsDefined() || !node.IsSequence())
        return;

    for(auto&& elem : node)
        v.push_back(elem.as<int>());
}

inline void ReadVectorDouble(std::vector<double>& v, const YAML::Node& node){
    if(!node.IsDefined() || !node.IsSequence())
        return;

    for(auto&& elem : node)
        v.push_back(elem.as<double>());
}

inline void ReadVectorString(std::vector<std::string>& v, const YAML::Node& node){
    if(!node.IsDefined() || !node.IsSequence())
        return;

    for(auto&& elem : node)
        v.push_back(elem.as<std::string>());
}

}
}
