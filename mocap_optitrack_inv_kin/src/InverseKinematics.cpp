#include <InverseKinematics.h>

InverseKinematics::InverseKinematics()
{
}

/*Main method to get the configuration vector*/
Eigen::VectorXf InverseKinematics::getConfiguration(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr &msg, 
                          std::vector<long int> &ring_ids,
                          std::vector<double> &ls,
                          std::vector<double> &ds,
                          std::vector<double> &Ls) const
{
    //Allocate the vector of configuration variables to be returned
    Eigen::VectorXf q(ring_ids.size()*3);

    //Perform the inverse kinematics
    int nRB = (int) msg->rigid_bodies.size();
    int pos;
    for (auto& i : ring_ids)
    {
        pos = this->getRingPosition(msg, nRB, i);
        if (pos == -1){printf("Error: ring not found.\n");}else{
            //Compute the inverse kinematics
            
            //TODO : Remove the object since it is not required anymore
        }
    }
    //Return the configuration
    return q;
}

int InverseKinematics::getRingPosition(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr &msg, int nRB, int ID) const
{
    int i;
    for (i = 0; i < nRB; i++)
    {
        if ( msg->rigid_bodies[i].id == ID) return i;
    }
    //ID not found
    printf("ID [%d] not found in the message.\n", ID);
    return -1;
}

//Get rotation matrix associated to quaternion representation
Eigen::Matrix3f InverseKinematics::quatToRotm(float qx, float qy, float qz, float qw) const
{
  Eigen::Matrix3f R;
  R(0,0) = 2*(pow(qw,2)+pow(qx,2))-1;
  R(0,1) = 2*(qx*qy-qw*qz);
  R(0,2) = 2*(qx*qz+qw*qy);
  R(1,0) = 2*(qx*qy+qw*qz);
  R(1,1) = 2*(pow(qw,2)+pow(qy,2))-1;
  R(1,2) = 2*(qy*qz-qw*qx);
  R(2,0) = 2*(qx*qz-qw*qy);
  R(2,1) = 2*(qy*qz+qw*qx);
  R(2,2) = 2*(pow(qw,2)+pow(qz,2))-1;
  return R;
}