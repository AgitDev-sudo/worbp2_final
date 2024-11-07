import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name_robot = 'urdf/lynxmotion_arm.urdf'

    robot_urdf = os.path.join(
        get_package_share_directory('simulation'),
        urdf_file_name_robot)
    with open(robot_urdf, 'r') as infp:
        robot_desc = infp.read()



    virtual_servo_controller_node = Node(
            package='simulation',
            executable='virtual_servo_controller_node',
            name='virtual_servo_controller_node',
            parameters=[{'robot_description': robot_desc}],
            # output='screen',
            # arguments=['--ros-args', '--log-level', 'ERROR']

        )
    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_publisher',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[robot_urdf],
        )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        robot_state_publisher_node,
        virtual_servo_controller_node,
    ])