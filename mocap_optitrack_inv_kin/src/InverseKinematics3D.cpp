#include <InverseKinematics.h>
#include <InverseKinematicsNode.h>
#include <InverseKinematics3D.h>

#include <cmath>
#include <limits>
#include <type_traits>

InverseKinematics3D::InverseKinematics3D(InverseKinematicsNode* IKNode_):InverseKinematics(IKNode_)
{
}


/*Main method to get the configuration vector*/
Eigen::VectorXf InverseKinematics3D::getConfiguration(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr &msg, 
                          std::vector<long int> &ring_ids,
                          std::vector<double> &ls,
                          std::vector<double> &ds,
                          std::vector<double> &Ls) const
{
    /*Variables initialization*/
    //Sort the rigid bodies in the message so that its all consistent with the parameters provided thrpugh the yaml file
    std::vector<mocap_optitrack_interfaces::msg::RigidBody> RBs = getSorteredBodies(msg, ring_ids);
    //
    int i, k = 0;
    int nRB = (int) RBs.size();//number of rigid bodies
    //
    // Configuration vector to be returned
    Eigen::VectorXf q(nRB*3);
    //
    // Rotation matrix from ring (i) frame to ring (i-1) frame
    Eigen::Matrix3f R_i_1_ri; R_i_1_ri << 1,0,0,0,1,0,0,0,1;
    // Position of the origin of ring (i) expressed in ring (i-1) frame
    Eigen::Vector3f t_i_1_ri; t_i_1_ri << 0,0,0;
    //Transformation matrix from ring (i-1) to robot base frame
    Eigen::Matrix4f T_0_i_1; T_0_i_1 << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
    double D_c_ri, delta_L_ri, Delta_x_ri, Delta_y_ri, L_factor, Delta_i, Delta_i2, ci, si, scf;
    //Epsilon quantity for limit computations, i.e. around 0. 
    //Adjust this parameter if the configuration is attaing unrelastic values.
    float eps = pow(10,3)*std::numeric_limits<float>::epsilon();
    //
    /*Run the 3D inverse kinematics*/
    for (i = 0; i < nRB; i++ )
    {
        R_i_1_ri = T_0_i_1.block<3,3>(0,0).transpose()*this->quatToRotm(RBs[i].pose_stamped.pose.orientation.x,
                                                                        RBs[i].pose_stamped.pose.orientation.y,
                                                                        RBs[i].pose_stamped.pose.orientation.z,
                                                                        RBs[i].pose_stamped.pose.orientation.w);
        t_i_1_ri = (T_0_i_1.inverse()*(Eigen::VectorXf(4) << RBs[i].pose_stamped.pose.position.x, RBs[i].pose_stamped.pose.position.y, RBs[i].pose_stamped.pose.position.z, 1).finished()).head(3);
        /*Compute the configuration*/
        //Compute the scaling factor for the current segment
        L_factor = Ls[i]/ls[i];
        //Approximate 1 with 1-eps to run the limit (straight configuration)
        if(std::abs(R_i_1_ri(2,2)) >= 1)
        {
            R_i_1_ri(2,2) = R_i_1_ri(2,2) + eps*((R_i_1_ri(2,2) > 0) ? -1 : 1);
        }
        delta_L_ri = t_i_1_ri(2)*(acos(R_i_1_ri(2,2)))/(sin(acos(R_i_1_ri(2,2))))-ls[i];
        //
        D_c_ri = ds[i]/(ls[i]+delta_L_ri)*(pow(acos(R_i_1_ri(2,2)), 2)/(1-R_i_1_ri(2,2)));
        Delta_x_ri = t_i_1_ri(0)*D_c_ri;
        Delta_y_ri = t_i_1_ri(1)*D_c_ri;
        //
        q(k)   = L_factor*Delta_x_ri;
        q(k+1) = L_factor*Delta_y_ri;
        q(k+2) = L_factor*delta_L_ri;
        //Update the transformation matrix
        Delta_i2= pow(q(k),2) + pow(q(k+1),2);
        //If Delta_i2 is too close to zero approximate with eps to compute the limit
        if(Delta_i2 < eps)
        {
            Delta_i2 = eps;
        }
        Delta_i = sqrt(Delta_i2);
        ci      = cos(Delta_i/ds[i]);
        si      = sin(Delta_i/ds[i]);
        scf     = ds[i]*(Ls[i]+q(k+2))/Delta_i2;
        T_0_i_1 = T_0_i_1*(Eigen::MatrixXf(4,4) << 1+(pow(q(k),2)/Delta_i2)*(ci-1), (q(k)*q(k+1)/Delta_i2*(ci-1))    , q(k)/Delta_i*si  , scf*q(k)*(1-ci),
                                                   (q(k)*q(k+1)/Delta_i2*(ci-1))  , 1+(pow(q(k+1),2)/Delta_i2)*(ci-1), q(k+1)/Delta_i*si, scf*q(k+1)*(1-ci),
                                                   -q(k)/Delta_i*si               , -q(k+1)/Delta_i*si               , ci               , scf*Delta_i*si,
                                                   0                              , 0                                , 0                , 1).finished();
        //Update the iterator for the configuration vector
        k += 3;
    }
    //Log the configuration vector
    RCLCPP_DEBUG(this->IKNode->get_logger(), "Configuration vector : \n");
    RCLCPP_DEBUG(this->IKNode->get_logger(), (static_cast<std::ostringstream&&>(std::ostringstream() << q)).str().c_str());
    //
    //Return the configuration
    return q;
}
