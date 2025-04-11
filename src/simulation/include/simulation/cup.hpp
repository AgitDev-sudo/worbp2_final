/*
 * cup.hpp
 *
 *      Author: agit
 */

#ifndef INCLUDE_SIMULATION_CUP_HPP
#define INCLUDE_SIMULATION_CUP_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class Cup : public rclcpp::Node
{
public:
    Cup();
    ~Cup();
private:
    void initCup(const std::string& urdf_content);
    void updateColorInUrdf(const std::string& urdf_content, const std::string& color);
    void publishCup();
    void updatePos(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
    void cupIsHeldCb();

    bool cup_is_held_;
    rclcpp::TimerBase::SharedPtr cup_publish_timer_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description_pub_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr cup_pos_sub_;
    geometry_msgs::msg::TransformStamped current_transform_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    uint8_t speed_factor_;
    rclcpp::TimerBase::SharedPtr check_cup_held_timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr robot_arm_distance_pub_;
    

};

#endif // INCLUDE_SIMULATION_CUP_HPP
