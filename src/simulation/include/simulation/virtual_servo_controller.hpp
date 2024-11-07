/*
 * virtual_servo_controller.hpp
 *
 *      Author: agit
 */

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

private:
    void commandCallback(const std_msgs::msg::String msg);
    void initJointStates();
    void publishJointStates();

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ssc32u_messages_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr joint_state_pub_timer_;

    //name of joint and it's position in rad.
    std::map<std::string, double> joints;

    const std::map<uint8_t, std::string> servoNrToJoints =
    {
        {0, "base_link2turret"},
        {1, "turret2upperarm"},
        {2, "upperarm2forearm"},
        {3, "forearm2wrist"},
        {4, "wrist2hand"},
        {5, "gripper_left2hand"},
        {6, "gripper_right2hand"}
    };

};

#endif // SIMULATION_VIRTUALSERVOCONTROLLER_HPP