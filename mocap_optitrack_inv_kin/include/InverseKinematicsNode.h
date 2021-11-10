#ifndef INVERSEKINEMATICSNODE_H
#define INVERSEKINEMATICSNODE_H

#include "rclcpp/rclcpp.hpp"
#include "mocap_optitrack_interfaces/msg/configuration_array.hpp"
#include "mocap_optitrack_interfaces/msg/rigid_body_array.hpp"
#include <stdio.h>


using std::placeholders::_1;

class InverseKinematics;

class InverseKinematicsNode: public rclcpp::Node
{
private:
    //Methods
    void rigid_body_topic_callback(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr msg) const;

    //Attributes
    rclcpp::Subscription<mocap_optitrack_interfaces::msg::RigidBodyArray>::SharedPtr subscription_;
    rclcpp::Publisher<mocap_optitrack_interfaces::msg::ConfigurationArray>::SharedPtr publisher_;
    std::unique_ptr<InverseKinematics> ik;

    //DA RIMUOVERE
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();
    //

public:
    //Methods
    InverseKinematicsNode();
};


#endif