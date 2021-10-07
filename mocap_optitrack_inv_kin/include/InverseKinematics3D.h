#ifndef INVERSEKINEMATICS3D_H
#define INVERSEKINEMATICS3D_H

#include <InverseKinematics.h>

class InverseKinematics3D: public InverseKinematics
{
public:
    //Public methods
    InverseKinematics3D(InverseKinematicsNode* IKNode_);

    Eigen::VectorXf getConfiguration(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr &msg, 
                          std::vector<long int> &ring_ids,
                          std::vector<double> &ls,
                          std::vector<double> &ds,
                          std::vector<double> &Ls) const;
};

#endif