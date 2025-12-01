# microROS Agent starten

Je kunt de de microROS agent starten met de volgende commando's, afhankelijk van de verbinding die je gebruikt:


:::::{card} 

::::{tab-set}

:::{tab-item} USB

```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6
```

:::

:::{tab-item} Wifi
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

:::

::::

:::::