// RoS2 Node that handles the connection with the NatNet server (Motive)
#include <MoCapPublisher.h>

// Include standard libraries
#include <stdio.h>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
// Include the MoCap NatNet client
#include <MoCapNatNetClient.h>

using namespace std;
using namespace std::chrono_literals;
using namespace std::chrono;

bool cmpRigidBodyId(sRigidBodyData body_a, sRigidBodyData body_b)
{
  return body_a.ID < body_b.ID;
}

MoCapPublisher::MoCapPublisher(): Node("natnet_client")
{

  //Declare the ROS2 parameters used by the NatNet Client
  this->declare_parameter<std::string>("server_address", "10.125.37.2");
  this->declare_parameter<int>("connection_type", 0);
  this->declare_parameter<std::string>("multi_cast_address", "239.255.42.99");
  this->declare_parameter<uint16_t>("server_command_port", 1510);
  this->declare_parameter<uint16_t>("server_data_port", 1511);
  this->declare_parameter<std::string>("pub_topic", "rigid_body_topic");
  //
  //Create the publisher
  std::string topic_;
  this->get_parameter("pub_topic", topic_);
  this->publisher_ = this->create_publisher<mocap_optitrack_interfaces::msg::RigidBodyArray>(topic_.c_str(), 10);
  //
  //Get the current time for the timestamp of the messages
  this->t_start = high_resolution_clock::now();//get the current time
  //
  //Just for testing purposes send make messages every 500ms
  //this->timer_ = this->create_wall_timer(500ms, std::bind(&MoCapPublisher::sendFakeMessage, this));
  //
  //Log info about creation
  RCLCPP_INFO(this->get_logger(), "Created MoCap publisher node.\n");

  //TO REMOVE
  std::string address_;
  this->get_parameter("server_address", address_);
  RCLCPP_INFO(this->get_logger(),address_.c_str());
}

// Method that send over the ROS network the data of a rigid body
void MoCapPublisher::sendRigidBodyMessage(sRigidBodyData* bodies_ptr, int nRigidBodies)
{
  std::vector<sRigidBodyData> bodies;
  for(int i=0; i < nRigidBodies; i++) 
  {
    bodies.push_back(bodies_ptr[i]);
  }

  // sort rigid bodies by their id
  std::sort(bodies.begin(), bodies.end(), cmpRigidBodyId);

  //Instanciate variables
  mocap_optitrack_interfaces::msg::RigidBodyArray msg;
  high_resolution_clock::time_point t_current;
  // Log
  RCLCPP_INFO(get_logger(), "Sending message containing %d Rigid Bodies.\n\n", nRigidBodies);
  // Loop over all the rigid bodies
  for(int i=0; i < nRigidBodies; i++)
  {
    RCLCPP_INFO(this->get_logger(), "Rigid Body [ID=%d  Error=%3.2f  Valid=%d]\n", bodies[i].ID, bodies[i].MeanError, bodies[i].params & 0x01);
    RCLCPP_INFO(this->get_logger(), "\tx\ty\tz\tqx\tqy\tqz\tqw\n");
    RCLCPP_INFO(this->get_logger(), "\t%3.5f\t%3.5f\t%3.5f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
      bodies[i].x,
      bodies[i].y,
      bodies[i].z,
      bodies[i].qx,
      bodies[i].qy,
      bodies[i].qz,
      bodies[i].qw);
      //
      //Create the rigid body message
      mocap_optitrack_interfaces::msg::RigidBody rb;
      rb.id = bodies[i].ID;
      rb.valid =  bodies[i].params & 0x01;
      rb.mean_error = bodies[i].MeanError;
      rb.pose_stamped.pose.position.x = bodies[i].x;
      rb.pose_stamped.pose.position.y = bodies[i].y;
      rb.pose_stamped.pose.position.z = bodies[i].z;
      rb.pose_stamped.pose.orientation.x = bodies[i].qx;
      rb.pose_stamped.pose.orientation.y = bodies[i].qy;
      rb.pose_stamped.pose.orientation.z = bodies[i].qz;
      rb.pose_stamped.pose.orientation.w = bodies[i].qw;
      //
      // Add the time stamp information both in seconds and nanoseconds
      t_current = high_resolution_clock::now();
      auto time_span_s  = duration_cast<seconds>(t_current - this->t_start);
      auto time_span_ns = duration_cast<nanoseconds>(t_current - this->t_start);
      //
      rb.pose_stamped.header.stamp.sec = time_span_s.count();
      rb.pose_stamped.header.stamp.nanosec = time_span_ns.count();
      //
      // Add the current rigid body to the array of rigid bodies
      msg.rigid_bodies.push_back(rb);
  }
  // Publish the message.
  publisher_->publish(msg);
}

//Method used to send fake messages to the client
void MoCapPublisher::sendFakeMessage()
{
    int nRigidBodies = 2;
    sRigidBodyData* bodies = (sRigidBodyData*) malloc(nRigidBodies * sizeof(sRigidBodyData));

    for (int i = 0; i < nRigidBodies; i++)
    {
      bodies[i].x = 1*i;
      bodies[i].y = 2*i;
      bodies[i].z = 3*i;
      bodies[i].qx = 4*i;
      bodies[i].qy = 5*i;
      bodies[i].qz = 6*i;
      bodies[i].qw = 7*i;
      bodies[i].MeanError = 0.0;
      bodies[i].ID = i;
      bodies[i].params = 1;
    }
    //Send the message
    this->sendRigidBodyMessage(bodies, nRigidBodies);

    //Free the rigid bodies
    free(bodies);
}

std::string MoCapPublisher::getServerAddress()
{
  std::string addr_;
  this->get_parameter("server_address", addr_);
  return addr_;
}

int MoCapPublisher::getConnectionType()
{
  int type_;
  this->get_parameter("connection_type", type_);
  return type_;
}

std::string MoCapPublisher::getMulticastAddress()
{
  std::string addr_;
  this->get_parameter("multi_cast_address", addr_);
  return addr_;
}

uint16_t MoCapPublisher::getServerCommandPort()
{
  uint16_t port_;
  this->get_parameter("server_command_port", port_);
  return port_;
}

uint16_t MoCapPublisher::getServerDataPort()
{
  uint16_t port_;
  this->get_parameter("server_data_port", port_);
  return port_;
}


// Main
int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  
  // Initialize ROS2
  rclcpp::init(argc, argv);

  //Create the ROS2 publisher
  auto mocapPub = std::make_shared<MoCapPublisher>();
  //Create the MoCapNatNetClient
  MoCapNatNetClient* c = new MoCapNatNetClient(mocapPub.get());
  // Try to connect the client 
  int retCode = c->connect();
  if (retCode != 0)
  {
    return retCode;
  }
  // Ready to receive marker stream
  rclcpp::spin(mocapPub);
  // Delete all the objects created
  delete c;
  rclcpp::shutdown();//delete the ROS2 nodes
  return 0;
}
