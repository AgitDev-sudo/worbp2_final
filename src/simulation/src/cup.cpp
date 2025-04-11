#include "cup.hpp"

constexpr const char* DEFAULT_NODE_NAME = "cup";
constexpr const char* DEFAULT_CUP_DESC_PARAM_NAME = "cup_desc";

Cup::Cup(): Node(DEFAULT_NODE_NAME) ,
    cup_publish_timer_(this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Cup::publishCup, this))),
    cup_pos_sub_(this->create_subscription<geometry_msgs::msg::TransformStamped>("picked_up_cup", 10, std::bind(&Cup::updatePos, this, std::placeholders::_1))),
    speed_factor_(1),
    check_cup_held_timer_(this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Cup::cupIsHeldCb, this))),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_ (std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false)),
    robot_arm_distance_pub_(this->create_publisher<geometry_msgs::msg::TransformStamped>("picked_up_cup", 10))
{

    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.transient_local();  // zorgt dat late subscribers alsnog de laatste message krijgen
    description_pub_ = this->create_publisher<std_msgs::msg::String>("cup_description", qos_profile);

    current_transform_.header.frame_id = "base_link";
    current_transform_.child_frame_id = "cup";
    current_transform_.transform.translation.x = 0.45;
    current_transform_.transform.translation.y = 0.0;
    current_transform_.transform.translation.z = 0.05;
    current_transform_.transform.rotation.x = 0.0;
    current_transform_.transform.rotation.y = 0.0;
    current_transform_.transform.rotation.z = 0.0;

  
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    this->declare_parameter(DEFAULT_CUP_DESC_PARAM_NAME, std::string(""));
    initCup(this->get_parameter(DEFAULT_CUP_DESC_PARAM_NAME).as_string());
}
Cup::~Cup()
{
    RCLCPP_INFO(this->get_logger(), "Cup node destroyed");
}

bool Cup::initCup(const std::string& urdf_content)
{
    auto msg = std::make_unique<std_msgs::msg::String>();

    msg->data = urdf_content;

    description_pub_->publish(std::move(msg));
    return true; // Return true if initialization is successful
}
// void Cup::setCupPosition(double x, double y, double z)
// {
//     // Set the position of the cup in 3D space
//     RCLCPP_INFO(this->get_logger(), "Setting cup position to: x=%f, y=%f, z=%f", x, y, z);
// }

void Cup::publishCup() 
{
    geometry_msgs::msg::TransformStamped t;

    t = current_transform_;
    t.header.stamp = this->now();

    tf_broadcaster_->sendTransform(t);
}

void Cup::updatePos(const geometry_msgs::msg::TransformStamped::SharedPtr msg) {

    if (msg->header.frame_id == "hand") {
        current_transform_ = *msg;   
    } else if (msg->header.frame_id == "base_link" && msg->transform.translation.z > 0.05) {
        msg->transform.translation.z -= 0.005 * speed_factor_;
        msg->transform.rotation.y = 0.0;
        current_transform_ = *msg;
        ++speed_factor_;
    } else {
        speed_factor_ = 1;
    }
}

void Cup::cupIsHeldCb() {
    geometry_msgs::msg::TransformStamped msg;
    try
    {
        geometry_msgs::msg::TransformStamped transform_gripper_left = tf_buffer_->lookupTransform("gripper_left", "cup", tf2::TimePointZero);
        geometry_msgs::msg::TransformStamped transform_gripper_right = tf_buffer_->lookupTransform("gripper_right", "cup", tf2::TimePointZero);
        geometry_msgs::msg::TransformStamped transformHand = tf_buffer_->lookupTransform("hand", "cup", tf2::TimePointZero);

        RCLCPP_INFO(this->get_logger(), "Gripper left: x: %f, y: %f, z: %f", transform_gripper_left.transform.translation.x, transform_gripper_left.transform.translation.y, transform_gripper_left.transform.translation.z);
        if (
            transform_gripper_left.transform.translation.x < -0.006 &&
            transform_gripper_left.transform.translation.x > -0.007 &&
            transform_gripper_left.transform.translation.y < -0.01 &&
            transform_gripper_left.transform.translation.y > -0.015 &&
            transform_gripper_left.transform.translation.z < 0.022 &&
            transform_gripper_left.transform.translation.z > 0.02 &&
            transform_gripper_right.transform.translation.x < -0.006 &&
            transform_gripper_right.transform.translation.x > -0.007 &&
            transform_gripper_right.transform.translation.y < 0.015 &&
            transform_gripper_right.transform.translation.y > 0.01 &&
            transform_gripper_right.transform.translation.z < 0.022 &&
            transform_gripper_right.transform.translation.z > 0.02
        ) {
            msg = transformHand;
        } else {
            msg = tf_buffer_->lookupTransform("base_link", "cup", tf2::TimePointZero);
        }
        robot_arm_distance_pub_->publish(msg);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    }
}