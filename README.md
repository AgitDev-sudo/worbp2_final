# worbp2_final

## Benodigdheden
 - ROS 2 Jazzy Jalisco
 - RViz2
 - Tf2

## Installeren
1. Clone de repo
   
   `git clone git@github.com:AgitDev-sudo/worbp2_final.git`

2. Ga naar de root van de workspace / repro
  
   `cd worbp2_final`

3. Build de packages
   
   `colcon build`

## Draai kopje demo
Note: Vergeet niet in elke nieuwe terminal te sourcen!!!:
```bash
source instal/setup.bash
```

1. Draai in een terminal:
```bash
ros2 launch simulation simulation_launch.py
```

2. Open een nieuwe terminal en draai de demo script:
```bash
./pickup_up_sequence.sh
```
Indien je geen rechten heb tot te script verleen rechten met:
```bash
chmod +x pickup_cup_sequence.sh
```

Met deze demo zien we hoe de robot arm door middel van `single servo commands` een kopje oppakt, verplaats en loslaat.  

## API

In dit onderdeel worden de API beschreven van de ros nodes die gebruikt worden voor dit beroepsproduct.
De voorbeelden hoe de nodes standalone gedraait worden puur weergeven om inzicht te geven welke parameters er opgegeven kan worden aan een node.
Het is aangeraden om de launch file van de simulation package te gebruiken hierin zijn alle default argumenten verwerkt.

```bash
ros2 launch simulation simulation_launch.py
```

### virtual_servo_controller_node


De node standalone draaien:
```bash
ros2 run simulation virtual_servo_controller_node --ros-args -p robot_description_file:=<pad/naar/urdf/lynxmotion_arm.urdf> #voorbeeld
ros2 run simulation virtual_servo_controller_node --ros-args -p robot_description_file:=../worbp2_final/src/simulation/urdf/lynxmotion_arm.urdf #In ons geval
ros2 run simulation virtual_servo_controller_node --ros-args -p robot_description_file:=$(ros2 pkg prefix simulation)/share/simulation/urdf/lynxmotion_arm.urdf #Makkelijkste manier! 
```

De node draaien met Rviz, cup_node en robot_state_publisher. Aangeraden want alle arugmenten zijn al ingevuld in de launch file.
```bash
ros2 launch simulation simulation_launch.py
```

#### Commando's zonder response
Commando's sturen naar de virtuele servo controller:
Volgens de specificatie van de SSC32U protocol zijn er commando's waar je wel of geen antwoord op krijgt. Hieronder volgen de commando's die de ros package ondersteunen waarvan we geen antwoord verwachten.

##### Single servo commands:
```bash 
ros2 topic pub --once /ssc32u_command std_msgs/msg/String '{data: "#3 P1700 T800\r"}' #Beweeg servo 3 met een pwm waarde van 1700 binnen 800 ms.
ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#0 P2500 T10000\r\"}" #Beweeg servo 0, de base, helemaal naar in 10 seconde. 
ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#2 P1800 S800\r\"}" #Beweeg servo 2 met een snelheid van 800us/S naar positie 1800.
ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#3 P1800 S800 T1000\r\"}" #Servo berekent met behulp van zijn huidigie positie met welke tijd die het langst over mag doen en die neemt die!

ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#5 P1507 T1000\r\"}" #Sluit gripper in 1 seconde. (1 servo voor de gesimuleerde linker en rechter gripper)

ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#5 P1500 T1000\r\"}" #Open gripper

ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#4 P1800\r\"}" #Valide commando, wordt geparsed, maar wordt niet uitgevoerd, omdat we de maximale snelheid van de echte servo niet weten heb ik dit niet mee genomen. 
```
![api_single_servo](./api_single_servo.png)

##### Stoppen van servo:
```bash
ros2 topic pub --once /ssc32u_command std_msgs/msg/String '{data: "STOP 0\r"}' ## Stop servo 0 (dus de base)
ros2 topic pub --once /ssc32u_command std_msgs/msg/String '{data: "STOP 1\r"}' ## Stop servo 1
ros2 topic pub --once /ssc32u_command std_msgs/msg/String '{data: "STOP\r"}' ##Stop alle servos
```
#### Group command
De node accepteerd group command zoals
```bash
ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#0 P2500 S1000 #1 P800 T2000\r\"}
```
maar doet er niks mee, omdat hier geen tijd voor was om het uit te werken en goed te testen. Het parsen van de berichten is grondig getest en werkt wel. (In andere worden alle data word in de SSC32Command data type geplaats maar en wordt niet mee gerekent en wat uitgevoerd)

![api_group_servo](./api_multi_servo.png)

#### Samenvatting onderdeel m.b.t. servo commando
- Valide pin nummer [0-5]
- T commando wordt ondersteunt
- S commando wordt ondersteunt
- Commbinatie ook.
- Geeft een foutmelding als commando niet valide is.

##### Improvment voor volgend student:
1. Ondersteuning voor een commando met alleen P waarde: 
```bash
ros2 topic pub --once /ssc32u_command std_msgs/msg/String "{data: \"#0 P1800\r\"}"
```
Het parsen zit er in echter de uitvoering niet, omdat ik de hardware limit / max speed van de betreffende servo niet weet. Wanneer geen P of T is opgegeven dan hoort een servo zo snel mogelijk naar desbetreffende positie te gaan. Dit is natuurlijk afhankelijk van de hardware.

2. Groupcommand gebruiken.

#### Commando's met response
De volgende commando's retouneren een response bericht. Deze responses worden op de ``ssc32u_response`` topic gepubliseert.

```bash
ros2 topic echo /ssc32u_response
```

##### Opvragen van arm status
```bash
ros2 topic pub --once /ssc32u_command std_msgs/msg/String '{data: "Q\r"}' #Response: "+" arm beweegt of "." arm is stil
```


## robot_state_publiser
De node standalone draaien:
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat $(ros2 pkg prefix simulation)/share/simulation/urdf/lynxmotion_arm.urdf)" \
  -p use_sim_time:=false
```

## cup_node
De node standalone draaien:
```bash
ros2 run simulation cup_node --ros-args \
  -p use_sim_time:=false \
  -p cup_pose.x:=0.45 -p cup_pose.y:=0.0 -p cup_pose.z:=0.05 \
  -p cup_pose.roll:=0.0 -p cup_pose.pitch:=0.0 -p cup_pose.yaw:=0.0 \
  -p cup_desc:="$(cat $(ros2 pkg prefix simulation)/share/simulation/urdf/cup.urdf)"

```
Een kopje kan op een willekeurige plek op de wereld geplaats worden.

## rviz2
De node standalone draaien:
```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix simulation)/share/simulation/rviz/urdf.rviz

```


