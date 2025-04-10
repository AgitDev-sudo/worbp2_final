#include "virtual_servo_controller.hpp"
#include "ssc32u_hardware_limitations.hpp"
#include "urdf/model.h"

//microseconds
constexpr double_t PULSE_WIDTH_RATE_US = SERVO_REFRESH_INTERVAL;
//milliseconds
constexpr double_t JOINT_STATE_PUBLISHER_RATE_MS = (PULSE_WIDTH_RATE_US/1000);

constexpr const char* DEFAULT_NODE_NAME = "virtual_servo_controller";
constexpr const char* DEFAULT_TOPIC_NAME_SUB = "ssc32u_command";
constexpr const char* DEFAULT_TOPIC_NAME_PUB = "joint_states";
constexpr const char* DEFAULT_ROBOT_URDF_FILE_PARAM_NAME = "robot_description_file";

VirtualServoController::VirtualServoController() : 
rclcpp::Node(DEFAULT_NODE_NAME),
  ssc32u_messages_(create_subscription<std_msgs::msg::String>(
            DEFAULT_TOPIC_NAME_SUB, 10, std::bind(&VirtualServoController::commandCallback, this, std::placeholders::_1))),
  joint_state_pub_ (create_publisher<sensor_msgs::msg::JointState>(DEFAULT_TOPIC_NAME_PUB,10)),
  joint_state_pub_timer_(create_wall_timer(std::chrono::milliseconds(static_cast<int64_t>(JOINT_STATE_PUBLISHER_RATE_MS)), std::bind(&VirtualServoController::publishJointStates, this)))

{
    this->declare_parameter<std::string>(DEFAULT_ROBOT_URDF_FILE_PARAM_NAME, "");

    if(!initJointStates(this->get_parameter(DEFAULT_ROBOT_URDF_FILE_PARAM_NAME).as_string()))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize joint states, the visualization will not be correct");
    }
}

VirtualServoController::~VirtualServoController()
{ 
}

void VirtualServoController::commandCallback(const std_msgs::msg::String msg)
{

}

bool VirtualServoController::initJointStates(const std::string& urdf_file)
{
    urdf::Model model;

    if (!model.initFile(urdf_file))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
        return false;
    }

    for (auto &joint : model.joints_)
    {
        RCLCPP_INFO(this->get_logger(), "Got joint: %s", joint.first.c_str());

        joints.insert(std::pair<std::string, double>(joint.first, 0));
    }

    if(servoNrToJoints.size() != joints.size())
    {
        RCLCPP_ERROR(this->get_logger(), "The number of joints in the URDF file does not match the number of servos");
        return false;
    }

    return true;
}

void VirtualServoController::publishJointStates()
{
    if (this->joints.empty())
    {
        RCLCPP_INFO(this->get_logger(), "We won't publish join states, joint states not properly initialzed");
        return;
    }

    auto msg = std::make_unique<sensor_msgs::msg::JointState>();
    msg->header.stamp = this->now();
    for (auto const& [jointName, jointPos] : this->joints)
    {
        msg->name.push_back(jointName);
        msg->position.push_back(jointPos);
    }
    joint_state_pub_->publish(std::move(msg));
}

