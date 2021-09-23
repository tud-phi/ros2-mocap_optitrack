#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include "mocap_optitrack_interfaces/msg/rigid_body_array.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>

class InverseKinematics
{
private:
    //Private methods
    Eigen::Matrix3f quatToRotm(float qx, float qy, float qz, float qw) const;
    int getRingPosition(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr &msg, int nRB, int ID) const;

    //Private attributes
public:
    //Public methods
    InverseKinematics();

    Eigen::VectorXf getConfiguration(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr &msg, 
                          std::vector<long int> &ring_ids,
                          std::vector<double> &ls,
                          std::vector<double> &ds,
                          std::vector<double> &Ls) const;
};

#endif