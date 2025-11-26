# Inleiding

In deze documentatie wordt uitgelegd hoe je verschillende microros controllers kunt gebruiken met een micro-ROS agent. De joystick controller kan worden aangesloten via USB of WiFi, afhankelijk van de beschikbare hardware en voorkeuren.

De volgende controllers worden ondersteund:
- Joystick controller via USB
- Joystick controller via WiFi
- Ultarasonic sensor via USB

Voor dat je begint dien je Visual Studio Code met PlatformIO geïnstalleerd te hebben. Daarnaast is het aan te raden om een Linux omgeving te gebruiken, bijvoorbeeld:
- Native Ubuntu installatie
- Dual boot met Ubuntu
- Virtual Machine met Ubuntu
- WSL2 op Windows

Zie voor nadere instructies de [IDE installatie handleiding](ide_installation.md).

Tevens dien je een micro-ROS agent geïnstalleerd te hebben. Zie voor nadere instructies de [micro-ROS agent installatie handleiding](micro_ros_agent_installation.md).

# Microros Joystick Controller





## Under construction

Clone repository
```
cd ~
git clone https://github.com/AvansMechatronica/microros_joystick.git
```



Hardware
USB:

WiFi:
Setup EPS32 lite filesystem see:
[setup filesystem](https://randomnerdtutorials.com/esp32-vs-code-platformio-littlefs/)

Bringup microros:
Wifi
```
ros2 launch p3dx_bringup wifi.launch.py
```
of
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

USB
```
ros2 launch p3dx_bringup usb.launch.py
```
of
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6
```


Install microros Agent
```
source /opt/ros/$ROS_DISTRO/setup.bash

mkdir uros_ws && cd uros_ws

git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

rosdep update && rosdep install --from-paths src --ignore-src -y

colcon build

source install/local_setup.bash

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh

echo "source ~/uros_ws/install/local_setup.bash" >> $HOME/.bashrc

source install/local_setup.bash
```