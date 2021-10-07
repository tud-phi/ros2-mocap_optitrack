#include <InverseKinematics.h>
#include <InverseKinematicsNode.h>

InverseKinematics::InverseKinematics(InverseKinematicsNode* IKNode_)
{
    this->IKNode = IKNode_;
}

//Method that gets the index position from the message msg given the length of the message and the ID of the ring in the motion capture system.
int InverseKinematics::getRingPosition(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr &msg, int nRB, int ID) const
{
    int i;
    for (i = 0; i < nRB; i++)
    {
        if ( msg->rigid_bodies[i].id == ID) return i;
    }
    //ID not found
    RCLCPP_ERROR(this->IKNode->get_logger(), "ID [%d] not found in the message.\n", ID);
    return -1;
}


//Returns the vector of rigid bodies sorted by the ring IDs in the configuration file
std::vector<mocap_optitrack_interfaces::msg::RigidBody> InverseKinematics::getSorteredBodies(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr &msg, std::vector<long int> &ring_ids) const
{
    std::vector<mocap_optitrack_interfaces::msg::RigidBody> RBs;
    int i = 0;
    int nRB = (int) msg->rigid_bodies.size();
    for (auto& pos : ring_ids)
    {
        //Get the position of current ring
        i = this->getRingPosition(msg, nRB, pos);
        if (i == -1){RCLCPP_ERROR(this->IKNode->get_logger(), "Ring not found.\n");}else{//TO DO: implement the error as a ROS2 log
            //Compute the inverse kinematics
            RBs.push_back(msg->rigid_bodies[i]);
        }
    }
    return RBs;
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