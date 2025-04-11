/*
 * virtual_servo_controller.hpp
 *
 *      Author: agit
 */

#ifndef SIMULATION_VIRTUALSERVOCONTROLLER_HPP
#define SIMULATION_VIRTUALSERVOCONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <atomic>

struct ActiveMotion {
    std::thread thread;
    std::atomic_bool stop_requested{false};
    std::atomic_bool is_moving{false};


    ActiveMotion() = default;

    // Custom move constructor
    ActiveMotion(ActiveMotion&& other) noexcept
        : thread(std::move(other.thread)),
          stop_requested(other.stop_requested.load())
    {}

    // Custom move assignment
    ActiveMotion& operator=(ActiveMotion&& other) noexcept {
        if (this != &other) {
            if (thread.joinable()) thread.join();
            thread = std::move(other.thread);
            stop_requested.store(other.stop_requested.load());
        }
        return *this;
    }

    ActiveMotion(const ActiveMotion&) = delete;
    ActiveMotion& operator=(const ActiveMotion&) = delete;
};

class VirtualServoController :  public rclcpp::Node
{

public:
    VirtualServoController();  
    ~VirtualServoController();

private:

    void commandCallback(const std_msgs::msg::String msg);
    bool initJointStates(const std::string& urdf_file);
    void publishJointStates();
    void setDesiredJointState(uint8_t servo_nr, double target_rad, double time_ms);
    double pwmToRad(uint8_t servo_pin, uint16_t pwm) const;
    double calculateMoveDurationMsRadial(uint8_t servo_pin, double current_rad, double target_rad, uint16_t speed_us_per_s) const;
    void stopServo(uint8_t servo_pin);
    void stopAllServos();
    bool isArmMoving();

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ssc32u_request_messages_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ssc32u_response_messages_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr joint_state_pub_timer_;

    //name of joint and it's position in rad.
    std::map<std::string, double> joints;
    std::map<uint8_t, ActiveMotion> moving_joint_threads_;  // key = servo_nr

    const std::map<uint8_t, std::string> servo_to_joints =
    {
        {0, "base_link2turret"},
        {1, "turret2upperarm"},
        {2, "upperarm2forearm"},
        {3, "forearm2wrist"},
        {4, "wrist2hand"},
        {5, "gripper_left2hand"},
        {6, "gripper_right2hand"}
    };

    //<servo_pin, <min_pwm, max_pwm>>
    const std::map<uint8_t, std::pair<uint16_t, uint16_t>> servo_pwm_limits_ =
    {
        {0, {500,2500}},
        {1, {500,2500}},
        {2, {500,2500}},
        {3, {500,2500}},
        {4, {500,2500}},
        {5, {500,2500}},
        {6, {500,2500}}
        // {0, {600, 2400}},
        // {1, {700, 2300}},
        // {2, {600, 2400}},
        // {3, {700, 2300}},
        // {4, {700, 2300}},
        // {5, {700, 2300}},
        // {6, {700, 2300}}
    };

};

#endif // SIMULATION_VIRTUALSERVOCONTROLLER_HPP