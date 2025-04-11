import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name_robot = 'urdf/lynxmotion_arm.urdf'
    urdf_file_name_cup = 'urdf/cup.urdf'

    rviz_config_name = 'rviz/urdf.rviz'

    robot_urdf = os.path.join(
        get_package_share_directory('simulation'),
        urdf_file_name_robot)
    with open(robot_urdf, 'r') as infp:
        robot_desc = infp.read()


    cup_urdf = os.path.join(
        get_package_share_directory('simulation'),
        urdf_file_name_cup)
    with open(cup_urdf, 'r') as infp:
        cup_desc = infp.read()


    rviz_config = os.path.join(
        get_package_share_directory('simulation'),
        rviz_config_name)


    virtual_servo_controller_node = Node(
            package='simulation',
            executable='virtual_servo_controller_node',
            name='virtual_servo_controller_node',
            parameters=[{'robot_description_file': robot_urdf}]
            # output='screen',
            # arguments=['--ros-args', '--log-level', 'ERROR']

        )
    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_publisher',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}]
        )
    
    cup_node = Node (
            package='simulation',
            executable='cup_node',
            name='cup_node',
            parameters=[{'use_sim_time': use_sim_time, 'cup_desc': cup_desc}]

        )
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', str(rviz_config)]
        )
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        virtual_servo_controller_node,
        cup_node,
        robot_state_publisher_node,
        rviz_node,
    ])