#ifndef INVERSEKINEMATICS2D_H
#define INVERSEKINEMATICS2D_H

#include <InverseKinematics.h>

class InverseKinematics2D: public InverseKinematics
{
private:
    //Private methods
    Eigen::Matrix3f quatToRotm(float qx, float qy, float qz, float qw) const;
    int getRingPosition(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr &msg, int nRB, int ID) const;
    std::vector<mocap_optitrack_interfaces::msg::RigidBody> getSorteredBodies(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr &msg, std::vector<long int> &ring_ids) const;

public:
    //Public methods
    InverseKinematics2D(InverseKinematicsNode* IKNode_);

    Eigen::VectorXf getConfiguration(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr &msg, 
                          std::vector<long int> &ring_ids,
                          std::vector<double> &ls,
                          std::vector<double> &ds,
                          std::vector<double> &Ls) const;
};

#endif