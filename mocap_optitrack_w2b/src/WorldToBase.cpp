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
  this->declare_parameter<std::string>("sub_topic", "rigid_body_topic");
  this->declare_parameter<std::string>("pub_topic", "rigid_body_baseframe_topic");

  //Subscribe to the topic for RigidBody messages
  std::string sub_topic_;
  this->get_parameter("sub_topic", sub_topic_);
  char* sub_topic = (char*) malloc(sub_topic_.length()*sizeof(char));
  strcpy(sub_topic, sub_topic_.c_str());
  this->subscription_ = this->create_subscription<mocap_optitrack_interfaces::msg::RigidBodyArray>(
    sub_topic, 10, std::bind(&WorldToBase::rigid_body_topic_callback, this, _1));
  
  //Publisher definition
  std::string pub_topic_;
  this->get_parameter("pub_topic", pub_topic_);
  char* pub_topic = (char*) malloc(pub_topic_.length()*sizeof(char));
  strcpy(pub_topic, pub_topic_.c_str());
  this->publisher_ = this->create_publisher<mocap_optitrack_interfaces::msg::RigidBodyArray>(pub_topic, 10);
}

//Callback to receive rigid body messages
void WorldToBase::rigid_body_topic_callback(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr msg) const
{
  //RCLCPP_INFO(this->get_logger(), "I heard: ");
  // Transform the pose of all the rigid bodies from the frame of the motion capture system to the base frame of the robot
  // Publish the message with the modified pose
  transformPoseAndSend(msg);
}

// Method that transforms the pose of the rigid bodies expressed in the motion capture system into the base frame of the robot
void WorldToBase::transformPoseAndSend(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr msg) const
{
  Vector3f P, P_base;//P_base is the position of the robot base recorded by Motive
  Vector4f P_1;
  Matrix3f R, R_base;//R_base is the orientation of the robot base recoded by Motive
  Matrix4f T_M_0, T_0_P;//transformation matrix from the motion capture system to the robot base frame

  P_base << 0,0,0;
  R_base << 0,0,0,0,0,0,0,0,0;
  T_M_0  << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1;
  T_0_P  << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1;
  P_1 << 0,0,0,1;

  int i,i_base = -1;
  int nRB = (int) msg->rigid_bodies.size();

  /*USE THE ROBOT BASE ONLINE*/
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
  //The rigid body associated to the base was not found
  if(i_base == -1)
  {
    printf("Rigid body of the base not found!\n\n");
    //TODO : decide what to do in this case, we just continue the process
  }

  // ? : To speed up we can avoid this query but if you change online not working anymore...
  /*RETREIVE THE POSE OF THE ROBOT BASE IN THE MOTIVE FRAME*/
  float base_qx, base_qy, base_qz, base_qw, initial_offset_x, initial_offset_y, initial_offset_z; 
  this->get_parameter("base_qx", base_qx);
  this->get_parameter("base_qy", base_qy);
  this->get_parameter("base_qz", base_qz);
  this->get_parameter("base_qw", base_qw);
  this->get_parameter("initial_offset_x", initial_offset_x);
  this->get_parameter("initial_offset_y", initial_offset_y);
  this->get_parameter("initial_offset_z", initial_offset_z);
  T_M_0.block<3,3>(0,0) = quatToRotm(base_qx, base_qy, base_qz, base_qw);
  T_M_0.block<3,1>(0,3) << initial_offset_x, initial_offset_y, initial_offset_z;
  T_M_0.block<3,1>(0,3) += P_base;

  std::cout << "Transformation matrix from robot base frame to motive : \n";
  std::cout << T_M_0 << std::endl;

  /*Get the transformation matrix from the body frame*/
  Matrix4f T_0_M; T_0_M << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1;
  T_0_M.block<3,3>(0,0) = T_M_0.block<3,3>(0,0).transpose();
  T_0_M.block<3,1>(0,3) = -T_0_M.block<3,3>(0,0)*T_M_0.block<3,1>(0,3);

  std::cout << T_0_M << std::endl;

  Eigen::Vector4f q;
  mocap_optitrack_interfaces::msg::RigidBodyArray msg_r;
  /*Compute the pose of each rigid body in the robot base frame*/
  for (i = 0; i < nRB; i++)
  {
      //Print some information
      printf("ID : %ld\n", msg->rigid_bodies[i].id);
      printf("Time stamp : %d(s)---%d(ns)\n", msg->rigid_bodies[i].pose_stamped.header.stamp.sec, msg->rigid_bodies[i].pose_stamped.header.stamp.nanosec);

      // Transform the pose
      T_0_P(0, 3) = msg->rigid_bodies[i].pose_stamped.pose.position.x;
      T_0_P(1, 3) = msg->rigid_bodies[i].pose_stamped.pose.position.y;
      T_0_P(2, 3) = msg->rigid_bodies[i].pose_stamped.pose.position.z;
      T_0_P.block<3,3>(0,0) = this->quatToRotm(msg->rigid_bodies[i].pose_stamped.pose.orientation.x,
                                               msg->rigid_bodies[i].pose_stamped.pose.orientation.y,
                                               msg->rigid_bodies[i].pose_stamped.pose.orientation.z,
                                               msg->rigid_bodies[i].pose_stamped.pose.orientation.w);
      //Compute the pose of the body in the robot base frame
      T_0_P = T_0_M*T_0_P;
      std::cout << T_0_P << std::endl;
      //Save the tranformed pose in the new message
      mocap_optitrack_interfaces::msg::RigidBody rb = msg->rigid_bodies[i];
      rb.pose_stamped.pose.position.x = T_0_P(0, 3);
      rb.pose_stamped.pose.position.y = T_0_P(1, 3);
      rb.pose_stamped.pose.position.z = T_0_P(2, 3);
      q = this->rotmToQuat(T_0_P.block<3,3>(0,0));
      rb.pose_stamped.pose.orientation.x = q(0);
      rb.pose_stamped.pose.orientation.y = q(1);
      rb.pose_stamped.pose.orientation.z = q(2);
      rb.pose_stamped.pose.orientation.w = q(3);

      // Add the current rigid body to the array of rigid bodies
      msg_r.rigid_bodies.push_back(rb);
  }

  /*Publish the message*/
  this->publisher_->publish(msg_r);
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

//Conversion from rotation matrix to unit quaternion
Eigen::Vector4f WorldToBase::rotmToQuat(Eigen::Matrix3f R) const
{
  Vector4f q;
  float x;
  //
  x = R(2,1)- R(1,2);
  q(0) = ( (x >= 0) ? 1 : -1)*sqrt(R(0,0)-R(1,1)-R(2,2)+1);
  //
  x = R(0,2)- R(2,0);
  q(1) = ( (x >= 0) ? 1 : -1)*sqrt(R(1,1)-R(2,2)-R(0,0)+1);
  //
  x = R(1,0)- R(0,1);
  q(2) = ( (x >= 0) ? 1 : -1)*sqrt(R(2,2)-R(1,1)-R(0,0)+1);
  //
  q(3) = sqrt(R(0,0)+R(1,1)+R(2,2)+1);
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