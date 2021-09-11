// RoS2 Node that handles the connection with the NatNet server (Motive)
#include <MoCapPublisher.h>

// Include standard libraries
#include <stdio.h>
#include <unistd.h>
#include <memory>
#include <chrono>
#include <functional>
#include <string>
// Include the MoCap NatNet client
#include <MoCapNatNetClient.h>

using namespace std;
using namespace std::chrono_literals;

MoCapPublisher::MoCapPublisher(): Node("mocap_publisher")
{
  this->publisher_ = this->create_publisher<mocap_optitrack_interfaces::msg::RigidBodyArray>("rigid_body_topic", 10);
  this->timer_ = this->create_wall_timer(
      500ms, std::bind(&MoCapPublisher::sendFakeMessage, this));
}

// Method that send over the ROS network the data of a rigid body
void MoCapPublisher::sendRigidBodyMessage(sRigidBodyData* bodies, int nRigidBodies)
{
  printf("Sending message containing %d Rigid Bodies.\n\n", nRigidBodies);
  // Publish the message
  mocap_optitrack_interfaces::msg::RigidBodyArray msg;
  
  // Loop over all the rigid bodies
  for(int i=0; i < nRigidBodies; i++)
  {
    printf("Rigid Body [ID=%d  Error=%3.2f  Valid=%d]\n", bodies[i].ID, bodies[i].MeanError, bodies[i].params & 0x01);
    printf("\tx\ty\tz\tqx\tqy\tqz\tqw\n");
    printf("\t%3.5f\t%3.5f\t%3.5f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
      bodies[i].x,
      bodies[i].y,
      bodies[i].z,
      bodies[i].qx,
      bodies[i].qy,
      bodies[i].qz,
      bodies[i].qw);

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

      // Add the time stamp information
      rb.pose_stamped.header.stamp.sec = 1;
      rb.pose_stamped.header.stamp.nanosec = 1;

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
  //MoCapNatNetClient* c = new MoCapNatNetClient(mocapPub.get());
  // Try to connect the client 
  //int retCode = c->connect();
  //printf("Return code is : %d", retCode);
  //if (retCode != 0)
  //{
  //  printf("Error, exiting.");
  //  return retCode;
  //}
  // Ready to receive marker stream!
	//printf("\nClient is connected to server and listening for data...\n");
  printf("Starting the publisher...\n");
  rclcpp::spin(mocapPub);

	//bool bExit = false;
  //char choice;
	//while ( !bExit )
	//{
  //  scanf(" %c", &choice);
	//	if (choice == 'q') bExit = true;
  //}

  // Delete all the objects created
  //delete c;
  rclcpp::shutdown();//delete the ROS2 nodes

  return 0;
}
