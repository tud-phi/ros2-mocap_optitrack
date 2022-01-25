#include <InverseKinematicsNode.h>
#include <InverseKinematics.h>
#include <InverseKinematics3D.h>
#include <InverseKinematics2D.h>

// DA RIMUOVERE PER PROVA
#include <chrono>
#include <functional>
#include <memory>
#include <string>
using namespace std::chrono_literals;
//


//Class constructor
InverseKinematicsNode::InverseKinematicsNode(): Node("inverse_kinematics")
{
    //Declare the parameters of the node
    // this->declare_parameter<std::vector<long int>>("ring_ids");
    // this->declare_parameter<std::vector<double>>("ring_ls");
    // this->declare_parameter<std::vector<double>>("ring_ds");
    // this->declare_parameter<std::vector<double>>("segment_ls");
    this->declare_parameter("ring_ids");
    this->declare_parameter("ring_ls");
    this->declare_parameter("ring_ds");
    this->declare_parameter("segment_ls");

    this->declare_parameter<int>("base_id", 0);
    this->declare_parameter<bool>("2d_inverse_kinematics", false);
    this->declare_parameter<std::string>("sub_topic", "baseframe_rigid_bodies");
    this->declare_parameter<std::string>("pub_topic", "robot_configuration");
    //
    //Subscribe to the topic for RigidBody messages
    std::string sub_topic_;
    this->get_parameter("sub_topic", sub_topic_);
    char* sub_topic = (char*) malloc(sub_topic_.length()*sizeof(char));
    strcpy(sub_topic, sub_topic_.c_str());
    this->subscription_ = this->create_subscription<mocap_optitrack_interfaces::msg::RigidBodyArray>(
    sub_topic, 10, std::bind(&InverseKinematicsNode::rigid_body_topic_callback, this, _1));
    //
    //Publisher definition
    std::string pub_topic_;
    this->get_parameter("pub_topic", pub_topic_);
    char* pub_topic = (char*) malloc(pub_topic_.length()*sizeof(char));
    strcpy(pub_topic, pub_topic_.c_str());
    this->publisher_ = this->create_publisher<mocap_optitrack_interfaces::msg::ConfigurationArray>(pub_topic, 10);
    //
    //Create the node responsible of handling the inverse kinematics
    bool IK_type;
    this->get_parameter("2d_inverse_kinematics", IK_type);
    switch (IK_type){
        case true:
            RCLCPP_INFO(this->get_logger(), "Created 2D IK node. Listening for incoming data...\n");
            this->ik = std::unique_ptr<InverseKinematics>(new InverseKinematics2D(this));
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "Created 3D IK node. Listening for incoming data...\n");
            this->ik = std::unique_ptr<InverseKinematics>(new InverseKinematics3D(this));
    }
}

//Topic to receive the message of rigid bodies
void InverseKinematicsNode::rigid_body_topic_callback(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "Receceived message on rigid bodies.\n");
    //Retreive the parameters and perform the inverse kinematics
    std::vector<long int> IDs;
    this->get_parameter("ring_ids", IDs);
    std::vector<double> ls;
    this->get_parameter("ring_ls", ls);
    std::vector<double> ds;
    this->get_parameter("ring_ds", ds);
    std::vector<double> Ls;
    this->get_parameter("segment_ls", Ls);

    //
    //Call the inverse kinematics to get the configuration
    Eigen::VectorXf q = this->ik->getConfiguration(msg, IDs, ls, ds, Ls);
    std::cout << "Configuration:" << std::endl;
    std::cout << q << std::endl;
    //TODO: must generalize to handle also the 2D case. For the case of NoElongation delta_l is just 0.
    //Create the message of configurations and publish it
    mocap_optitrack_interfaces::msg::ConfigurationArray q_msg;
    int msg_size = (int) q.rows()/3;
    q_msg.configurations = std::vector<mocap_optitrack_interfaces::msg::Configuration>(msg_size);
    int i= 0, k = 0;
    for (; i < msg_size; i++)
    {
        q_msg.configurations[i].delta_x = q(k);
        q_msg.configurations[i].delta_y = q(k+1);
        q_msg.configurations[i].delta_l = q(k+2);
        k = k+3;
    }
    //Publish the message
    this->publisher_->publish(q_msg);
}

int main(int argc, char ** argv)
{
    //Create the node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InverseKinematicsNode>());
    rclcpp::shutdown();
    return 0;
}