#include <InverseKinematics.h>
#include <InverseKinematicsNode.h>
#include <InverseKinematics2D.h>

#include <cmath>
#include <limits>
#include <type_traits>

InverseKinematics2D::InverseKinematics2D(InverseKinematicsNode* IKNode_):InverseKinematics(IKNode_)
{
}


/*Main method to get the configuration vector*/
Eigen::VectorXf InverseKinematics2D::getConfiguration(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr &msg, 
                          std::vector<long int> &ring_ids,
                          std::vector<double> &ls,
                          std::vector<double> &ds,
                          std::vector<double> &Ls) const
{
    //Unused paramters, just to avoid warning
    (void) ls;
    (void) ds;
    /*Variables initialization*/
    //Sort the rigid bodies in the message so that its all consistent with the parameters provided thrpugh the yaml file
    std::vector<mocap_optitrack_interfaces::msg::RigidBody> RBs = getSorteredBodies(msg, ring_ids);
    //
    int i;
    int nRB = (int) RBs.size();//number of rigid bodies
    //
    // Configuration vector to be returned
    Eigen::VectorXf q(nRB);
    //
    // Rotation matrix from ring (i) frame to ring (i-1) frame
    Eigen::Matrix3f R_i_1_ri; R_i_1_ri << 1,0,0,0,1,0,0,0,1;
    //Transformation matrix from ring (i-1) to robot base frame
    Eigen::Matrix4f T_0_i_1; T_0_i_1 << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
    //Auxiliary variables for local computations
    double qi, ci, si;
    //Epsilon quantity for limit computations, i.e. around 0
    float eps = std::numeric_limits<float>::epsilon();
    //
    /*Run the 3D inverse kinematics*/
    for (i = 0; i < nRB; i++ )
    {
        R_i_1_ri = T_0_i_1.block<3,3>(0,0).transpose()*this->quatToRotm(RBs[i].pose_stamped.pose.orientation.x,
                                                                        RBs[i].pose_stamped.pose.orientation.y,
                                                                        RBs[i].pose_stamped.pose.orientation.z,
                                                                        RBs[i].pose_stamped.pose.orientation.w);
        /*Compute the configuration*/
        //Compute the scaling factor for the current segment
        //
        q(i)   = acos(R_i_1_ri(1,1));
        //Update the transformation matrix
        if(std::abs(q(i)) < eps)
        {
            qi = eps;
        }else{
            qi    = q(i);
        }
        ci = cos(qi);
        si = sin(qi);
        //
        T_0_i_1 = T_0_i_1*(Eigen::MatrixXf(4,4) << ci , -si, 0, Ls[i]*si/qi,
                                                   si ,  ci, 0, Ls[i]*(1-ci)/qi,
                                                     0,   0, 1, 0,
                                                     0,   0, 0, 1).finished();
    }
    //Log the configuration vector
    RCLCPP_DEBUG(this->IKNode->get_logger(), "Configuration vector : \n");
    RCLCPP_DEBUG(this->IKNode->get_logger(), (static_cast<std::ostringstream&&>(std::ostringstream() << q)).str().c_str());
    //
    //Return the configuration
    return q;
}

//Method that gets the index position from the message msg given the length of the message and the ID of the ring in the motion capture system.
int InverseKinematics2D::getRingPosition(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr &msg, int nRB, int ID) const
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
std::vector<mocap_optitrack_interfaces::msg::RigidBody> InverseKinematics2D::getSorteredBodies(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr &msg, std::vector<long int> &ring_ids) const
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
Eigen::Matrix3f InverseKinematics2D::quatToRotm(float qx, float qy, float qz, float qw) const
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