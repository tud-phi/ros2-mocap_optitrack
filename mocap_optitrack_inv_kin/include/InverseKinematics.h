#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include "mocap_optitrack_interfaces/msg/rigid_body_array.hpp"
#include "mocap_optitrack_interfaces/msg/rigid_body.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>

class InverseKinematicsNode;

class InverseKinematics
{
protected:
    Eigen::Matrix3f quatToRotm(float qx, float qy, float qz, float qw) const;
    int getRingPosition(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr &msg, int nRB, int ID) const;
    std::vector<mocap_optitrack_interfaces::msg::RigidBody> getSorteredBodies(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr &msg, std::vector<long int> &ring_ids) const;
    //
    InverseKinematicsNode* IKNode;
public:
    //Public methods
    InverseKinematics(InverseKinematicsNode* IKNode_);
    //
    virtual Eigen::VectorXf getConfiguration(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr &msg, 
                                             std::vector<long int> &ring_ids,
                                             std::vector<double> &ls,
                                             std::vector<double> &ds,
                                             std::vector<double> &Ls) const = 0;
};

#endif