#include <WorldToBase.h>

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <stdexcept>
#include <stdio.h>

 
using namespace Eigen;
using namespace std;

WorldToBase::WorldToBase(): Node("world_to_base")
{
  // Declare the parameters of the node
  this->declare_parameter<float>("base_qx", -0.7071068);
  this->declare_parameter<float>("base_qy", 0.0);
  this->declare_parameter<float>("base_qz", 0.0);
  this->declare_parameter<float>("base_qw",  0.7071068);
  this->declare_parameter<float>("initial_offset_x", 0.0);
  this->declare_parameter<float>("initial_offset_y", -0.19);
  this->declare_parameter<float>("initial_offset_z", 0.0);
  this->declare_parameter<int>("base_id", 0);
  this->declare_parameter<std::string>("sub_topic", "rigid_body_topic");
  this->declare_parameter<std::string>("pub_topic", "rigid_body_baseframe_topic");
  //
  // Subscribe to the topic for RigidBody messages
  std::string sub_topic_;
  this->get_parameter("sub_topic", sub_topic_);
  char* sub_topic = (char*) malloc(sub_topic_.length()*sizeof(char));
  strcpy(sub_topic, sub_topic_.c_str());
  this->subscription_ = this->create_subscription<mocap_optitrack_interfaces::msg::RigidBodyArray>(
    sub_topic, 10, std::bind(&WorldToBase::rigid_body_topic_callback, this, _1));
  //
  // Publisher definition
  std::string pub_topic_;
  this->get_parameter("pub_topic", pub_topic_);
  char* pub_topic = (char*) malloc(pub_topic_.length()*sizeof(char));
  strcpy(pub_topic, pub_topic_.c_str());
  this->publisher_ = this->create_publisher<mocap_optitrack_interfaces::msg::RigidBodyArray>(pub_topic, 10);
  //
  // Log info about creation
  RCLCPP_INFO(this->get_logger(), "Created world to base node.\n");
}

// Callback to receive rigid body messages
void WorldToBase::rigid_body_topic_callback(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr msg) const
{
  // Transform the pose of all the rigid bodies from the frame of the motion capture system to the base frame of the robot
  transformPoseAndSend(msg);
}

// Method that transforms the pose of the rigid bodies expressed in the motion capture system into the base frame of the robot
void WorldToBase::transformPoseAndSend(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr msg) const
{
  /* Variables declaration */
  // Position of the robot base recorded by the motion capture system
  Vector3f t_W_0bar; t_W_0bar << 0,0,0;
  // Orientation of the robot base recorded by the motion capture system
  Matrix3f R_W_0tilde; R_W_0tilde << 1,0,0,0,1,0,0,0,1;
  // Transformation matrix from the world frame to the \bar{0} frame (e.g. mo-cap markers on the robot base holder)
  Matrix4f T_W_0bar; T_W_0bar  << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
  // Transformation matrix from the \bar{0} frame (e.g. mo-cap markers on the robot base holder) to the base frame
  Matrix4f T_0bar_0; T_0bar_0  << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
  // Transformation matrix from the world frame (e.g. motion capture frame) to the robot base frame and inverse
  Matrix4f T_W_0, T_0_W; T_W_0  << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1; T_0_W = T_W_0;
  // Transformation matrix from the robot base frame to the (generic) rigid body frame
  Matrix4f T_0_B; T_0_B  << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
  // Transformation matrix from the world frame to the (generic) rigid body frame as initialised by MoCap (\tilde{B})
  Matrix4f T_W_Btilde; T_W_Btilde  << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
  // Transformation matrix the rigid body frame as initialised by the MoCap to be aligned with the world frame on creation 
  // to the rigid body frame initialised to be aligned with the base frame in straight configuration
  Matrix4f T_Btilde_B; T_Btilde_B  << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
  //
  int i,i_base = -1;//i is the iterator, i_base is the index position in the RigidBodyArray 
  int nRB = (int) msg->rigid_bodies.size();
  int BASE_STREAMING_ID = -1; // Motive ID of the robot base
  // Retrieve the ID of the base frame as ROS2 parameter
  this->get_parameter("base_id", BASE_STREAMING_ID);
  RCLCPP_INFO(this->get_logger(), "BASE ID : %d\n", BASE_STREAMING_ID);
  // Retrieve the rotation between world and base frame as ROS2 parameters
  float base_qx, base_qy, base_qz, base_qw, initial_offset_x, initial_offset_y, initial_offset_z; 
  this->get_parameter("base_qx", base_qx);
  this->get_parameter("base_qy", base_qy);
  this->get_parameter("base_qz", base_qz);
  this->get_parameter("base_qw", base_qw);
  // Retrieve the translation between the markers on the base holder and the base of the proximal end of the robotic arm
  this->get_parameter("initial_offset_x", initial_offset_x);
  this->get_parameter("initial_offset_y", initial_offset_y);
  this->get_parameter("initial_offset_z", initial_offset_z);
  //
  Eigen::Vector4f xi_0_B; // vector that represents the unit quaternion of the rigid body in the robot base frame
  mocap_optitrack_interfaces::msg::RigidBodyArray msg_r;//message to send over the ROS2 network with the modified poses
  //
  /* Get the pose of the base of the robot recorded by the motion capture system */
  for (i = 0; i < nRB; i++)
  {
      if (msg->rigid_bodies[i].id == BASE_STREAMING_ID)
      {
        i_base = i;
        // Store the position and orientation of the robot base in the world frame
        t_W_0bar << msg->rigid_bodies[i].pose_stamped.pose.position.x, msg->rigid_bodies[i].pose_stamped.pose.position.y, msg->rigid_bodies[i].pose_stamped.pose.position.z;
        R_W_0tilde = this->quatToRotm(msg->rigid_bodies[i].pose_stamped.pose.orientation.x,
                                      msg->rigid_bodies[i].pose_stamped.pose.orientation.y,
                                      msg->rigid_bodies[i].pose_stamped.pose.orientation.z,
                                      msg->rigid_bodies[i].pose_stamped.pose.orientation.w);
        break;
      }
  }
  // The rigid body associated to the base was not found
  if(i_base == -1)
  {
    RCLCPP_ERROR(this->get_logger(), "Rigid body of the base not found.\n");
    throw std::runtime_error("Could not transform pose estimates from world to robot base frame as did not find rigid body representing the base holder.");
  }
  // Store the rotation from the world frame to the base frame
  T_W_0bar.block<3,3>(0,0) = quatToRotm(base_qx, base_qy, base_qz, base_qw);
  // Store the translation from the world frame to the \Bar{0} frame (markers attached to base holder)
  T_W_0bar.block<3,1>(0,3) = t_W_0bar;
  // Store the translation from the \Bar{0} frame (markers attached to base holder) to the base of the proximale segment (e.g. base frame)
  T_0bar_0.block<3,1>(0,3) << initial_offset_x, initial_offset_y, initial_offset_z;
  // Compute the resulting transformation  matrix from the world frame to the robot base frame
  T_W_0 = T_W_0bar*T_0bar_0;
  // Log transfromation matrix from the world frame to the robot base frame
  RCLCPP_DEBUG(this->get_logger(), "Transformation matrix from world frame to robot base frame: \n"); 
  RCLCPP_DEBUG(this->get_logger(), (static_cast<std::ostringstream&&>(std::ostringstream() << T_W_0)).str().c_str()); 
  //
  /* Get the transformation matrix from the robot base frame to the world frame by inverting T_W_0 */
  T_0_W.block<3,3>(0,0) = T_W_0.block<3,3>(0,0).transpose();
  T_0_W.block<3,1>(0,3) = -T_0_W.block<3,3>(0,0)*T_W_0.block<3,1>(0,3);
  // RCLCPP_DEBUG(this->get_logger(), "Transformation matrix from the robot base frame to the world frame: \n"); 
  // RCLCPP_DEBUG(this->get_logger(), (static_cast<std::ostringstream&&>(std::ostringstream() << T_0_W)).str().c_str()); 

  // Compute the rotation of the rigid body frame from \tilde{B} to B
  // e.g. we switch from a y-up initialised body frame to z-up initialised body frame
  T_Btilde_B.block<3,3>(0,0) = T_W_0.block<3,3>(0,0);

  //
  /* Compute the pose of each rigid body in the robot base frame */
  for (i = 0; i < nRB; i++)
  {
      // Print some information
      RCLCPP_DEBUG(this->get_logger(), "ID : %ld\n", msg->rigid_bodies[i].id);
      RCLCPP_DEBUG(this->get_logger(), "Time stamp : %d(s)---%d(ns)\n", msg->rigid_bodies[i].pose_stamped.header.stamp.sec, msg->rigid_bodies[i].pose_stamped.header.stamp.nanosec);
      //
      // Get the pose of each rigid body in the motive frame
      T_W_Btilde(0, 3) = msg->rigid_bodies[i].pose_stamped.pose.position.x;
      T_W_Btilde(1, 3) = msg->rigid_bodies[i].pose_stamped.pose.position.y;
      T_W_Btilde(2, 3) = msg->rigid_bodies[i].pose_stamped.pose.position.z;
      T_W_Btilde.block<3,3>(0,0) = this->quatToRotm(msg->rigid_bodies[i].pose_stamped.pose.orientation.x,
                                                    msg->rigid_bodies[i].pose_stamped.pose.orientation.y,
                                                    msg->rigid_bodies[i].pose_stamped.pose.orientation.z,
                                                    msg->rigid_bodies[i].pose_stamped.pose.orientation.w);
                                               
      // We need to invert the rotation as somehow the MoCap measures the rotation from the rotated frame of the rigid body back to its initial frame,
      // instead of (e.g. what we want) computing the rotation from the initial frame to the rotated frame of the rigid body.
      T_W_Btilde.block<3,3>(0,0).transposeInPlace();

      //
      // Compute the pose of the body in the robot base frame using T_0_W
      T_0_B = T_0_W * T_W_Btilde * T_Btilde_B;
      RCLCPP_DEBUG(this->get_logger(), "Transformation matrix from the robot base to the rigid body %ld.\n", msg->rigid_bodies[i].id);
      RCLCPP_DEBUG(this->get_logger(), (static_cast<std::ostringstream&&>(std::ostringstream() << T_0_B)).str().c_str());
      //
      /* Save the tranformed pose in the new message*/
      mocap_optitrack_interfaces::msg::RigidBody rb = msg->rigid_bodies[i];
      // Store the position
      rb.pose_stamped.pose.position.x = T_0_B(0, 3);
      rb.pose_stamped.pose.position.y = T_0_B(1, 3);
      rb.pose_stamped.pose.position.z = T_0_B(2, 3);
      // Store the orientation through the unit quaternion
      xi_0_B = this->rotmToQuat(T_0_B.block<3,3>(0,0));
      rb.pose_stamped.pose.orientation.x = xi_0_B(0);
      rb.pose_stamped.pose.orientation.y = xi_0_B(1);
      rb.pose_stamped.pose.orientation.z = xi_0_B(2);
      rb.pose_stamped.pose.orientation.w = xi_0_B(3);
      // Push the message to the array of messages to send
      msg_r.rigid_bodies.push_back(rb);
  }
  //
  // Publish the message
  this->publisher_->publish(msg_r);
}

// Save in R the rotation represented by the unit quaternion [qx,qy,qz,qw]
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
  //
  //Equivalent computation of the attitude
  //float xi0,xi1,xi2,xi3;
  //xi0 = qw;xi1 = qx;xi2 = qy; xi3 = qz;
  //R(0,0) = pow(xi0,2) + pow(xi1,2) - pow(xi2,2) - pow(xi3,2);
  //R(0,1) = 2*(xi1*xi2-xi0*xi3);
  //R(0,2) = 2*(xi0*xi2+xi1*xi3);
  //R(1,0) = 2*(xi0*xi3+xi1*xi2);
  //R(1,1) = pow(xi0,2) - pow(xi1,2) + pow(xi2,2) - pow(xi3,2);
  //R(1,2) = 2*(xi2*xi3-xi0*xi1);
  //R(2,0) = 2*(xi1*xi3-xi0*xi2);
  //R(2,1) = 2*(xi0*xi1+xi2*xi3);
  //R(2,2) = pow(xi0,2) - pow(xi1,2) - pow(xi2,2) + pow(xi3,2);
  return R;
}

// Conversion from rotation matrix to unit quaternion
Eigen::Vector4f WorldToBase::rotmToQuat(Eigen::Matrix3f R) const
{
  Vector4f q;
  float x;
  //
  x = R(2,1)- R(1,2);
  if(R(0,0)-R(1,1)-R(2,2)+1 > 0)
  {
    q(0) = ( (x >= 0) ? 1 : -1)*sqrt(R(0,0)-R(1,1)-R(2,2)+1);
  }else
  {
    q(0) = 0.0;
  }
  //
  x = R(0,2)- R(2,0);
  if(R(1,1)-R(2,2)-R(0,0)+1 > 0)
  {
    q(1) = ( (x >= 0) ? 1 : -1)*sqrt(R(1,1)-R(2,2)-R(0,0)+1);
  }else
  {
    q(1) = 0.0;
  }
  //
  x = R(1,0)- R(0,1);
  if(R(2,2)-R(1,1)-R(0,0)+1 > 0)
  {
    q(2) = ( (x >= 0) ? 1 : -1)*sqrt(R(2,2)-R(1,1)-R(0,0)+1);
  }else{
    q(2) = 0.0;
  }
  //
  if(R(0,0)+R(1,1)+R(2,2)+1 > 0)
  {
    q(3) = sqrt(R(0,0)+R(1,1)+R(2,2)+1);
  }else{
    q(3) = 0.0;
  }
  //
  q = 0.5*q;
  return q;
}

// Main method
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WorldToBase>());
    rclcpp::shutdown();
    return 0;
}