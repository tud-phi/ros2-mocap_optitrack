#ifndef INVERSEKINEMATICS2D_H
#define INVERSEKINEMATICS2D_H

#include <InverseKinematics.h>

class InverseKinematics2D: public InverseKinematics
{
public:
    //Public methods
    InverseKinematics2D(InverseKinematicsNode* IKNode_);
    //
    Eigen::VectorXf getConfiguration(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr &msg, 
                          std::vector<long int> &ring_ids,
                          std::vector<double> &ls,
                          std::vector<double> &ds,
                          std::vector<double> &Ls) const;
};

#endif