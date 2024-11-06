// VirtualServoController.hpp
#ifndef SIMULATION_VIRTUALSERVOCONTROLLER_HPP
#define SIMULATION_VIRTUALSERVOCONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class VirtualServoController :  public rclcpp::Node
{
public:
    VirtualServoController();  
    ~VirtualServoController();
};

#endif // SIMULATION_VIRTUALSERVOCONTROLLER_HPP