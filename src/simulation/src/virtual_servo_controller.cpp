#include "virtual_servo_controller.hpp"
#include "ssc32u_hardware_limitations.hpp"
#include "ssc32u_command_parser.hpp"
#include "urdf/model.h"

//microseconds
constexpr double_t PULSE_WIDTH_RATE_US = SERVO_REFRESH_INTERVAL;
//milliseconds
constexpr double_t JOINT_STATE_PUBLISHER_RATE_MS = (PULSE_WIDTH_RATE_US/1000);

constexpr const char* DEFAULT_NODE_NAME = "virtual_servo_controller";
constexpr const char* DEFAULT_TOPIC_NAME_COMMAND_SUB = "ssc32u_command";
constexpr const char* DEFAULT_TOPIC_NAME_RESPONSE_PUB = "ssc32u_response";
constexpr const char* DEFAULT_TOPIC_NAME_JOINT_STATE_PUB = "joint_states";
constexpr const char* DEFAULT_ROBOT_URDF_FILE_PARAM_NAME = "robot_description_file";

VirtualServoController::VirtualServoController() : 
rclcpp::Node(DEFAULT_NODE_NAME),
  ssc32u_request_messages_(create_subscription<std_msgs::msg::String>(
            DEFAULT_TOPIC_NAME_COMMAND_SUB, 10, std::bind(&VirtualServoController::commandCallback, this, std::placeholders::_1))),
  ssc32u_response_messages_(create_publisher<std_msgs::msg::String>(DEFAULT_TOPIC_NAME_RESPONSE_PUB,10)),
  joint_state_pub_ (create_publisher<sensor_msgs::msg::JointState>(DEFAULT_TOPIC_NAME_JOINT_STATE_PUB,10)),
  joint_state_pub_timer_(create_wall_timer(std::chrono::milliseconds(static_cast<int64_t>(JOINT_STATE_PUBLISHER_RATE_MS)), std::bind(&VirtualServoController::publishJointStates, this)))
{
    this->declare_parameter<std::string>(DEFAULT_ROBOT_URDF_FILE_PARAM_NAME, "");

    if(!initJointStates(this->get_parameter(DEFAULT_ROBOT_URDF_FILE_PARAM_NAME).as_string()))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize joint states, stopping the node");
        rclcpp::shutdown();
    }
}

VirtualServoController::~VirtualServoController()
{ 
}

void VirtualServoController::commandCallback(const std_msgs::msg::String msg)
{
    auto parsed_command = ssc32u::SSC32UCommandParser::parse(msg.data);
    switch (parsed_command.type_)
    {
        case ssc32u::SSC32UCommandType::SINGLE:
        {
            
            auto servo = parsed_command.servos_[0];
            auto it = servo_to_joints.find(servo.pin_);
            RCLCPP_INFO(this->get_logger(), "Got command for servo %d, pulse width %d", servo.pin_, servo.pulse_width_);
            if(it != servo_to_joints.end()) {

                double desired_rad = pwmToRad(servo.pin_, servo.pulse_width_);
                
                const std::string& joint_name = it->second;
                double current_rad = joints.find(joint_name)->second;
                double time = 0.0;

                if(servo.time_ms_.has_value()) {
                    time = *servo.time_ms_;
                }
                if(servo.speed_us_s_.has_value()) {
                    double speed_us_s = *servo.speed_us_s_;
                    if (speed_us_s > 0) {
                        time = std::max(time, calculateMoveDurationMsRadial(servo.pin_, current_rad, desired_rad, speed_us_s));
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Speed is zero, ignoring it");
                    }
                }

                if (time > 0) {
                    RCLCPP_INFO(this->get_logger(), "Moving servo %d to %f rad in %f ms", servo.pin_, desired_rad, time);
                    setDesiredJointState(servo.pin_, desired_rad, time);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Time is zero, ignoring it. Currently not supported");
                }
                
            } else {
                RCLCPP_ERROR(this->get_logger(), "Servo pin %d not found in the map", servo.pin_);
            }
            break;
        }
        case ssc32u::SSC32UCommandType::GROUP:
        {

            break;
        }
        case ssc32u::SSC32UCommandType::STATUS_QUERY:
        {
            auto msg = std::make_unique<std_msgs::msg::String>();
            msg->data = "+"; // move is still in progress

            if(!isArmMoving()) {
                msg->data = "."; // move is complete / not moving
            }

            ssc32u_response_messages_->publish(std::move(msg));

            break;
        }
        case ssc32u::SSC32UCommandType::EMERGENCY_STOP:
        {
            stopAllServos();
            break;
        }
        case ssc32u::SSC32UCommandType::STOP_SERVO:
        {
            if(parsed_command.stop_pin_.has_value())
            {
                auto it = moving_joint_threads_.find(*parsed_command.stop_pin_);
                if(it != moving_joint_threads_.end())
                {
                    stopServo(*parsed_command.stop_pin_);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Servo %d is already not moving, redundant call", *parsed_command.stop_pin_);
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Stop pin not found in the command");
            }
            break;
        }
        case ssc32u::SSC32UCommandType::UNKNOWN:
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown command");
            break;
        }
        default:
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown command type");
            break;
        }

    }
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

    if(servo_to_joints.size() != joints.size())
    {
        RCLCPP_ERROR(this->get_logger(), "The number of joints in the URDF file does not match the number of servos");
        return false;
    }

    //check if joint names match
    for (auto const& [servo_pin, joint_name] : servo_to_joints)
    {
        auto it = joints.find(joint_name);
        if(it == joints.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Joint %s not found in the URDF file", joint_name.c_str());
            return false;
        }
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

void VirtualServoController::setDesiredJointState(uint8_t servo_nr, double target_rad, double time_ms) {
    stopServo(servo_nr); // Stop any existing motion for this servo
    double difference = target_rad - this->joints.at(this->servo_to_joints.at(servo_nr));
    double step = 0;
    if (time_ms != 0 ) {
        step = difference / time_ms;
    } else {
        this->joints.at(this->servo_to_joints.at(servo_nr)) = target_rad;
        return;
    }
    
    ActiveMotion motion;
    motion.stop_requested = false;
    motion.is_moving = false;

    auto sendJointFunc = [this](double step, uint8_t servo_nr, double end_pos, uint16_t time_in_ms) {
        moving_joint_threads_[servo_nr].is_moving = true;
        for (int i = 0; i < time_in_ms; i++) {
            if (moving_joint_threads_[servo_nr].stop_requested) {
                end_pos = this->joints.at(this->servo_to_joints.at(servo_nr));
                break;
            }
            this->joints.at(this->servo_to_joints.at(servo_nr)) += step;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        
        this->joints.at(this->servo_to_joints.at(servo_nr)) = end_pos;
        moving_joint_threads_[servo_nr].is_moving = false;
    };

    std::thread t(sendJointFunc, step, servo_nr, target_rad, static_cast<uint16_t>(time_ms));

    motion.thread = std::move(t);

    moving_joint_threads_[servo_nr] = std::move(motion);

}


double VirtualServoController::pwmToRad(uint8_t servo_pin, uint16_t pwm) const
{
    auto it = servo_pwm_limits_.find(servo_pin);
    if (it == servo_pwm_limits_.end()) {
        RCLCPP_WARN(this->get_logger(), "No calibration found for pin %u, defaulting to 0", servo_pin);
        return 0.0;
    }

    const auto [pwm_min, pwm_max] = it->second;
    double clamped_pwm = pwm;
    if(pwm < pwm_min || pwm > pwm_max) {
        RCLCPP_WARN(this->get_logger(), "PWM %u is out of range for pin %u, clamping to [%u, %u]", pwm, servo_pin, pwm_min, pwm_max);
        clamped_pwm = std::clamp(static_cast<double>(pwm), static_cast<double>(pwm_min), static_cast<double>(pwm_max));
    }
    double normalized = (clamped_pwm - pwm_min) / (pwm_max - pwm_min);  // 0.0 tot 1.0
    return -M_PI_2 + normalized * M_PI;  // -π/2 tot π/2
}

double VirtualServoController::calculateMoveDurationMsRadial(uint8_t servo_pin, double current_rad, double target_rad, uint16_t speed_us_per_s) const
{
    auto it = servo_pwm_limits_.find(servo_pin);
    if (it == servo_pwm_limits_.end()) {
        RCLCPP_WARN(this->get_logger(), "No PWM limits found for pin %u", servo_pin);
        return 0.0;
    }

    const auto [pwm_min, pwm_max] = it->second;

    // radial range (-π/2 tot π/2)
    constexpr double rad_min = -M_PI_2; //-90 degrees
    constexpr double rad_max = M_PI_2;  // 90 degrees
    double rad_range = rad_max - rad_min;

    // PWM range
    double pwm_range = static_cast<double>(pwm_max - pwm_min);

    // Calculated max rad speed (rad/s)
    double rad_per_sec = (static_cast<double>(speed_us_per_s) / pwm_range) * rad_range;

    // Distance in radians
    double distance_rad = std::abs(target_rad - current_rad);

    // Duration in ms = (distance / speed) * 1000.0
    return (distance_rad / rad_per_sec) * 1000.0;
}

void VirtualServoController::stopServo(uint8_t servo_nr) {
    auto it = moving_joint_threads_.find(servo_nr);
    if (it != moving_joint_threads_.end()) {
        it->second.stop_requested = true;
        while(it->second.is_moving) {}
        if (it->second.thread.joinable()) {
            it->second.thread.join();
        }
        moving_joint_threads_.erase(it);
    }
}

void VirtualServoController::stopAllServos() {
    for (auto& [servo_nr, motion] : moving_joint_threads_) {
        motion.stop_requested = true;
    }

    for (auto& [servo_nr, motion] : moving_joint_threads_) {
        while(motion.is_moving) {}
        if (motion.thread.joinable()) {
            motion.thread.join();
        }
    }

    moving_joint_threads_.clear();
}

bool VirtualServoController::isArmMoving() {
    for (const auto& [servo_nr, motion] : moving_joint_threads_) {
        if (motion.is_moving) {
            return true;
        }
    }
    return false;
}





