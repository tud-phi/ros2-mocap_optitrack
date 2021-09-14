#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include "mocap_optitrack_interfaces/msg/rigid_body_array.hpp"

class InverseKinematics
{
private:
    //Private methods
    //Private attributes
public:
    //Public methods
    InverseKinematics();

    void getConfiguration(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr &msg, 
                          std::vector<long int> &ring_ids,
                          std::vector<double> &ring_ls,
                          std::vector<double> &ring_ds) const;
};

#endif