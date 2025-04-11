#!/bin/bash

# Gripper openen
# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#5 P700 T500\r\"}"
# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#6 P2300 T500\r\"}"
# sleep 0.6

# Arm omlaag naar kopje (verder naar voren)
# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#1 P1250 T800\r\"}"  # schouder
# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#2 P1100 T800\r\"}"
# sleep 1

# # # Gripper sluiten
# # ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#5 P1600 T500\r\"}"
# # ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#6 P1400 T500\r\"}"
# # sleep 0.6

# # Arm omhoog met kopje
# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#1 P1700 T800\r\"}"  # schouder
# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#2 P1700 T800\r\"}"  # elleboog
# sleep 1

# Gripper openen (kopje loslaten)
# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#5 P700 T500\r\"}"
# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#6 P2300 T500\r\"}"
# sleep 0.6


# Sequentiële servo-aansturing
# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#1 P1320 T2000\r\"}"
# sleep 3.1

# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#2 P1390 T2000\r\"}"
# sleep 3.1

# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#3 P1550 T2000\r\"}"
# sleep 3.1

# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#5 P1502 T1000\r\"}"
# sleep 2.1

# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#6 P1502 T1000\r\"}"
# sleep 2.1

# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#1 P1500 T2000\r\"}"
# sleep 3.1

# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#2 P1500 T2000\r\"}"
# sleep 3.1

# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#3 P1500 T2000\r\"}"
# sleep 3.1

# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#4 P1400 T2000\r\"}"
# sleep 3.1

# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#4 P1500 T2000\r\"}"
# sleep 3.1

# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#5 P1500 T1000\r\"}"
# sleep 2.1

# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#6 P1500 T1000\r\"}"
# sleep 2.1

#!/bin/bash

# # Servo 1 naar ~-1.132 rad → PWM 932
# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#1 P932 T2000\r\"}"
# sleep 2.5
# # Servo 2 naar ~-0.691 rad → PWM 1110
# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#2 P1110 T2000\r\"}"
# sleep 2.5
# # Servo 3 naar ~+0.314 rad → PWM 1623
# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#3 P1623 T2000\r\"}"
# sleep 2.5
# # Gripper half sluiten → PWM 1502
# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#5 P1502 T1000\r\"}"
# sleep 2.5
# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#6 P1502 T1000\r\"}"
# sleep 2.5

# # Arm terug naar neutraal (0 rad → PWM 1500)
# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#1 P1500 T2000\r\"}"
# sleep 2.5
# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#2 P1500 T2000\r\"}"
# sleep 2.5
# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#3 P1500 T2000\r\"}"
# sleep 2.5
# # Pols naar -0.628 rad → PWM 1030
# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#4 P1030 T2000\r\"}"
# sleep 2.5

# # Pols terug naar neutraal
# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#4 P1500 T2000\r\"}"
# sleep 2.5
# # Gripper openen
# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#5 P1500 T1000\r\"}"
# sleep 2.5
# ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#6 P1500 T1000\r\"}"
# sleep 2.5


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
ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#4 P1200 T1000\r\"}"  # pols kantelt naar voren
sleep 1.1
ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#4 P1500 T1000\r\"}"  # pols kantelt naar voren
sleep 1.1


echo "[5/6] Gripper openen (kopje loslaten)"
ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#5 P1500 T1000\r\"}"   # gripper open
sleep 1.1

echo "[6/6] Sequence afgerond!"
