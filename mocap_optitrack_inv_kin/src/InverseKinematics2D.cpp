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
    Eigen::Matrix2f R_i_1_ri; R_i_1_ri << 1,0,0,1;
    //Transformation matrix from ring (i-1) to robot base frame
    Eigen::Matrix3f T_0_i_1; T_0_i_1 << 1,0,0,0,1,0,0,0,1;
    //Auxiliary variable that represents a 3D rotation matrix, used for conversion from 3D rotation to 2D
    Eigen::Matrix3f R3D; R3D << 1,0,0,0,1,0,0,0,1;
    //Auxiliary variable that represents a 2D rotation matrix, used for conversion from 3D rotation to 2D
    Eigen::Matrix2f R2D; R2D << 1,0,0,1;
    //Auxiliary variables for local computations
    double qi, ci, si;
    //Epsilon quantity for limit computations, i.e. around 0
    float eps = std::numeric_limits<float>::epsilon();
    //
    /*Run the 3D inverse kinematics*/
    for (i = 0; i < nRB; i++ )
    {
        R3D = this->quatToRotm(RBs[i].pose_stamped.pose.orientation.x,
                               RBs[i].pose_stamped.pose.orientation.y,
                               RBs[i].pose_stamped.pose.orientation.z,
                               RBs[i].pose_stamped.pose.orientation.w);
        //It is assumed that the motion is occurring in the XZ plane of the motion capture system
        R2D << R3D(0,0), R3D(0,2), R3D(2,0), R3D(2,2);
        R_i_1_ri = T_0_i_1.block<2,2>(0,0).transpose()*R2D;
        /*Compute the configuration*/
        //
        std::cout << R_i_1_ri << std::endl;
        //q(i) = atan(R_i_1_ri(0,1)/R_i_1_ri(1,1));
        q(i) = (R_i_1_ri(0,1) < 0) ? acos(R_i_1_ri(1,1)) : -acos(R_i_1_ri(1,1));
        //
        //Update the transformation matrix
        qi = (std::abs(q(i)) < eps) ? ((q(i) < 0) ? -eps: eps) : q(i);
        //
        ci = cos(qi);
        si = sin(qi);
        T_0_i_1 = T_0_i_1*(Eigen::MatrixXf(3,3) <<  ci, -si, Ls[i]*si/qi,
                                                    si,  ci, Ls[i]*(1-ci)/qi,
                                                     0,   0, 1).finished();
    }
    //Log the configuration vector
    RCLCPP_DEBUG(this->IKNode->get_logger(), "Configuration vector : \n");
    RCLCPP_DEBUG(this->IKNode->get_logger(), (static_cast<std::ostringstream&&>(std::ostringstream() << q)).str().c_str());
    //
    //Return the configuration
    return q;
}
