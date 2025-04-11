# worbp2_final

ros2 topic pub --once /ssc32u_command std_msgs/msg/String '{data: "#3 P1700 T800\r"}'

ros2 topic pub --once /ssc32u_command std_msgs/msg/String '{data: "STOP 0\r"}'

ros2 topic pub --once /ssc32u_command std_msgs/msg/String '{data: "STOP\r"}'

ros2 topic pub --once /ssc32u_command std_msgs/msg/String '{data: "Q\r"}'


ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#1 P750 T2000\r\"}"
ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#2 P1050 T2000\r\"}"
ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#3 P1800 T2000\r\"}"

//Gripper sluiten
ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#5 P1507 T2000\r\"}"
ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#6 P1507 T2000\r\"}"