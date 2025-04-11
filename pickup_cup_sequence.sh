#!/bin/bash

echo "[1/6] Arm naar bekertje-positie"
ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#1 P750 T2000\r\"}"   # schouder
ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#2 P1050 T2000\r\"}"  # elleboog
ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#3 P1800 T2000\r\"}"  # pols
sleep 2.1

echo "[2/6] Gripper sluiten (bekertje vastpakken)"
ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#5 P1507 T2000\r\"}"  # gripper
sleep 2.1

echo "[3/6] Arm terug naar neutrale stand (kopje blijft rechtop)"
ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#1 P1500 T2000\r\"}"  # schouder neutraal
ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#2 P1500 T2000\r\"}"  # elleboog neutraal
ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#3 P1500 T2000\r\"}"  # pols terug recht
sleep 2.1

echo "[4/6] Pols kantelen"
ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#4 P1200 T1000\r\"}"  # pols kantelt
sleep 1.1
ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#4 P1500 T1000\r\"}"  # pols kantelt terug
sleep 1.1


echo "[5/6] Gripper openen (kopje loslaten)"
ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#5 P1500 T1000\r\"}"   # gripper open
sleep 1.1

echo "[6/6] Sequence afgerond!"
