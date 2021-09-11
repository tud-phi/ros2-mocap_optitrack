#include <WorldToBase.h>
#include <stdio.h>
#include <Eigen/Dense>
 
using namespace Eigen;
using namespace std;

WorldToBase::WorldToBase(): Node("world_to_base")
{

  //Declare the parameters of the node
  this->declare_parameter<float>("base_qx", 0.0);
  this->declare_parameter<float>("base_qy", 0.0);
  this->declare_parameter<float>("base_qz", 0.0);
  this->declare_parameter<float>("base_qw", 1.0);
  this->declare_parameter<float>("initial_offset_x", 0.0);
  this->declare_parameter<float>("initial_offset_y", -0.19);
  this->declare_parameter<float>("initial_offset_z", 0.0);
  this->declare_parameter<int>("base_id", 0);

  //Event handler for RigidBody messages
  this->subscription_ = this->create_subscription<mocap_optitrack_interfaces::msg::RigidBodyArray>(
    "rigid_body_topic", 10, std::bind(&WorldToBase::rigid_body_topic_callback, this, _1));
}

//Callback to receive rigid body messages
void WorldToBase::rigid_body_topic_callback(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr msg) const
{
  //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  // Transform the pose of all the rigid bodies from the frame of the motion capture system to the base frame of the robot
  transformPose(msg);
}

// Method that transforms the pose of the rigid bodies expressed in the motion capture system into the base frame of the robot
void WorldToBase::transformPose(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr msg) const
{
  Vector3f P, P_base;//P_base is the position of the robot base recorded by Motive
  Vector4f P_1;
  Matrix3f R, R_base;//R_base is the orientation of the robot base recoded by Motive
  Matrix4f T_0_B;//transformation matrix from the motion capture system to the robot base frame

  P_base << 0,0,0;
  R_base << 0,0,0,0,0,0,0,0,0;
  T_0_B  << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1;
  P_1 << 0,0,0,1;

  int i,i_base = -1;
  int nRB = (int) msg->rigid_bodies.size();

  //Retreive the ID of the base frame
  int BASE_STREAMING_ID = -1;
  this->get_parameter("base_id", BASE_STREAMING_ID);

  printf("BASE ID : %d\n", BASE_STREAMING_ID);
  
  // Get the pose of the base of the robot recorded by Motive
  for (i = 0; i < nRB; i++)
  {
      if (msg->rigid_bodies[i].id == BASE_STREAMING_ID)
      {
        i_base = i;
        //Store the position of the robot base and the orientation
        P_base << msg->rigid_bodies[i].pose_stamped.pose.position.x, msg->rigid_bodies[i].pose_stamped.pose.position.y, msg->rigid_bodies[i].pose_stamped.pose.position.z;
        R_base = this->quatToRotm(msg->rigid_bodies[i].pose_stamped.pose.orientation.x,
                                  msg->rigid_bodies[i].pose_stamped.pose.orientation.y,
                                  msg->rigid_bodies[i].pose_stamped.pose.orientation.z,
                                  msg->rigid_bodies[i].pose_stamped.pose.orientation.w);
        break;
      }
  }

  //Now that we have the pose of the robot base, compute the transformation matrix from the Motive frame to the robot base frame
  //TO MODIFY : retreive the parameters!
  //T_0_B.block<3,3>(0,0) = this->R_0_B;
  //T_0_B.block<3,1>(0,3) = P_base+this->offset;
  
  T_0_B.block<3,3>(0,0) = R_base;
  T_0_B.block<3,1>(0,3) = P_base;

  //std::cout << T_0_B << std::endl;

  //The rigid body associated to the base was not found
  if(i_base == -1)
  {
    printf("Rigid body of the base not found!\n\n");
    //TODO : decide what to do in this case, we just continue the process
  }

  // Iterate through all the rigid bodies
  for (i = 0; i < nRB; i++)
  {
      //Print some information
      printf("ID : %ld\n", msg->rigid_bodies[i].id);
      //Print the time stamp of the message
      printf("Time stamp : %d(s)---%d(ns)\n", msg->rigid_bodies[i].pose_stamped.header.stamp.sec, msg->rigid_bodies[i].pose_stamped.header.stamp.nanosec);

      // Transform first the position
      P << msg->rigid_bodies[i].pose_stamped.pose.position.x, msg->rigid_bodies[i].pose_stamped.pose.position.y, msg->rigid_bodies[i].pose_stamped.pose.position.z;
      R = this->quatToRotm(msg->rigid_bodies[i].pose_stamped.pose.orientation.x,
                           msg->rigid_bodies[i].pose_stamped.pose.orientation.y,
                           msg->rigid_bodies[i].pose_stamped.pose.orientation.z,
                           msg->rigid_bodies[i].pose_stamped.pose.orientation.w);
      P_1.segment(0,3) = P;//store in P_1 the current position of the marker
      //Compute the position of the point in the robot base frame
      std::cout << T_0_B.inverse()*P_1 << std::endl;
      //std::cout << T_0_B*P_1 << std::endl;
      //Store the position of the point in the message

      //TODO : Process the orientation...
  }
}

//Save in R the rotation represented by the unit quaternion [qx,qy,qz,qw]
Eigen::Matrix3f WorldToBase::quatToRotm(float qx, float qy, float qz, float qw) const
{
  Matrix3f R;
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

// Main method
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WorldToBase>());
    rclcpp::shutdown();
    return 0;
}