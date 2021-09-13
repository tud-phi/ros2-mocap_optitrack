#ifndef WORLDTOBASE_H
#define WORLDTOBASE_H


#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "mocap_optitrack_interfaces/msg/rigid_body_array.hpp"
#include <eigen3/Eigen/Dense>

using std::placeholders::_1;
using namespace std;

class WorldToBase: public rclcpp::Node
{
private:
    // Methods
    void rigid_body_topic_callback(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr msg) const;
    void transformPoseAndSend(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr msg) const;
    Eigen::Matrix3f quatToRotm(float qx, float qy, float qz, float qw) const;//transform a unit quaternion representation into a rotation matrix and save it in R
    Eigen::Vector4f rotmToQuat(Eigen::Matrix3f R) const;
    // Attributes
    rclcpp::Subscription<mocap_optitrack_interfaces::msg::RigidBodyArray>::SharedPtr subscription_;
    rclcpp::Publisher<mocap_optitrack_interfaces::msg::RigidBodyArray>::SharedPtr publisher_;
    
    
// Public attributes and methods
public:
    // Definition of the construtors
    WorldToBase();

    // Getters
    
    // Setters
    
};
 
#endif
