#include <InverseKinematics.h>
#include <cmath>


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
    //Sort the rigid bodies in the message so that its all consistent with the parameters
    std::vector<mocap_optitrack_interfaces::msg::RigidBody> RBs = getSorteredBodies(msg, ring_ids);

    /*Run the inverse kinematics*/

    //Initialize the variables
    int i, k = 0;
    int nRB = (int) RBs.size();//number of rigid bodies
    //Allocate the vector of configuration variables to be returned
    Eigen::VectorXf q(nRB*3);
    //Auxiliary variables
    Eigen::Matrix3f R_i_1_ri;
    Eigen::Vector3f t_i_1_ri;
    Eigen::Matrix4f T_0_i_1;
    T_0_i_1 << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;//Initialize to the identity matrix. TODO: clean the code and make with Identity function of eigen
    double D_c_ri, delta_L_ri, Delta_x_ri, Delta_y_ri, L_factor, Delta_i, Delta_i2, ci, si, scf;
    
    Eigen::Matrix4f T_prova;T_prova << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;

    //Run the IK
    for (i = 0; i < nRB; i++ )
    {
        //T_prova.block<3,3>(0,0) = this->quatToRotm(RBs[i].pose_stamped.pose.orientation.x,
        //                                            RBs[i].pose_stamped.pose.orientation.y,
        //                                            RBs[i].pose_stamped.pose.orientation.z,
        //                                            RBs[i].pose_stamped.pose.orientation.w);
        //T_prova.block<3,1>(0,3) << RBs[i].pose_stamped.pose.position.x, RBs[i].pose_stamped.pose.position.y, RBs[i].pose_stamped.pose.position.z;
        //std::cout << T_prova << std::endl;
        R_i_1_ri = T_0_i_1.block<3,3>(0,0).transpose()*this->quatToRotm(RBs[i].pose_stamped.pose.orientation.x,
                                                                        RBs[i].pose_stamped.pose.orientation.y,
                                                                        RBs[i].pose_stamped.pose.orientation.z,
                                                                        RBs[i].pose_stamped.pose.orientation.w);
        t_i_1_ri = (T_0_i_1.inverse()*(Eigen::VectorXf(4) << RBs[i].pose_stamped.pose.position.x, RBs[i].pose_stamped.pose.position.y, RBs[i].pose_stamped.pose.position.z, 1).finished()).head(3);
        std::cout << R_i_1_ri << std::endl;
        std::cout << t_i_1_ri << std::endl;
        //Compute the configuration
        L_factor = Ls[i]/ls[i];
        //
        //delta_L_ri = t_i_1_ri(2)*(ds[i]*acos(R_i_1_ri(2,2)))/(sin(acos(R_i_1_ri(2,2))))-ls[i];
        delta_L_ri = t_i_1_ri(2)*(acos(R_i_1_ri(2,2)))/(sin(acos(R_i_1_ri(2,2))))-ls[i];
        //std::cout << delta_L_ri << std::endl;
        //
        D_c_ri = ds[i]/(ls[i]+delta_L_ri)*(pow(acos(R_i_1_ri(2,2)), 2)/(R_i_1_ri(2,2)-1));
        Delta_x_ri = t_i_1_ri(0)*D_c_ri;
        Delta_y_ri = t_i_1_ri(1)*D_c_ri;
        //
        q(k)   = L_factor*Delta_x_ri;
        q(k+1) = L_factor*Delta_y_ri;
        q(k+2) = L_factor*delta_L_ri;
        //Update the transformation matrix
        Delta_i2= pow(q(k),2) + pow(q(k+1),2);
        Delta_i = sqrt(Delta_i2);
        ci      = cos(Delta_i/ds[i]);
        si      = sin(Delta_i/ds[i]);
        scf     = ds[i]*(Ls[i]+q(k+2))/Delta_i2;
        T_0_i_1 = T_0_i_1*(Eigen::MatrixXf(4,4) << 1+(pow(q(k),2)/Delta_i2)*(ci-1), (q(k)*q(k+1)/Delta_i2*(ci-1))    , q(k)/Delta_i*si  , scf*q(k)*(1-ci),
                                                   (q(k)*q(k+1)/Delta_i2*(ci-1))  , 1+(pow(q(k+1),2)/Delta_i2)*(ci-1), q(k+1)/Delta_i*si, scf*q(k+1)*(1-ci),
                                                   -q(k)/Delta_i*si               , -q(k+1)/Delta_i*si               , ci               , scf*Delta_i*si,
                                                   0                              , 0                                , 0                , 1).finished();
        //Update the itertor for the configuration vector
        k += 3;
    }
    std::cout << "Configuration vector : " << std::endl;
    std::cout << q << std::endl;

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
        if (i == -1){printf("Error: ring not found.\n");}else{//TO DO: implement the error as a ROS2 log
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