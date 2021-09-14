#include <InverseKinematics.h>


InverseKinematics::InverseKinematics()
{
}

/*Main method to get the configuration vector*/
void InverseKinematics::getConfiguration(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr &msg, 
                          std::vector<long int> &ring_ids,
                          std::vector<double> &ring_ls,
                          std::vector<double> &ring_ds) const
{
    int nRB = (int) msg->rigid_bodies.size();
    
}