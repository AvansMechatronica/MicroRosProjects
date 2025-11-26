# MicroRos Agent

## Wifi
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

## USB
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6
```


Hardware
USB:

WiFi:
Setup EPS32 lite filesystem see:
[setup filesystem](https://randomnerdtutorials.com/esp32-vs-code-platformio-littlefs/)