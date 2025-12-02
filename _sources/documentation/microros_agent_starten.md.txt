# microROS Agent starten

Je kunt de de microROS agent starten met de volgende commando's, afhankelijk van de verbinding die je gebruikt:

De microROS-agent kan worden gestart met het volgende commando:

:::::{card} 

::::{tab-set}

:::{tab-item} USB

```
ros2 run micro_ros_agent micro_ros_agent serial --dev <usb-device>
```

Afhankelijk op welke USB poort je het embedded systeem hebt aangesloten dien je usb-device in te vullen.
 Bijvoorbeeld:
 * /dev/ttyUSB0
 * /dev/ttyACM0

{octicon}`alert;2em;sd-text-info` Bij gebruik van WSL moet je het embedded systeem wel eerst met WSL verbinden, zie [Koppelen USB-devices aan WSL-Distributie](https://avansmechatronica.github.io/WindowsSubsystemForLinuxHandleiding/documentation/WSL_Handleiding.html#koppelen-usb-devices-aan-wsl-distributie)

{octicon}`bell;2em;sd-text-info` Het device wordt getoond bij het uploaden/programmeren van het embedded systeem in Visual Code met de Platform IO plugin

Je kunt eventueel extra opties meegeven, zoals het instellen van de log-level:

```bash 
ros2 run micro_ros_agent micro_ros_agent serial --dev <usb-device> --log-level debug
```
Het --log-level debug- zorgt ervoor dat er meer informatie in de terminal wordt getoond. Devolgende log-levels zijn beschikbaar: trace, debug, info, warn, error, fatal.

:::

:::{tab-item} Wifi

{octicon}`alert;2em;sd-text-info` Dit werkt alleen als in *platformio.ini* bestand de parameter **board_microROS_transport = wifi** is gezet.

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```
Je kunt eventueel extra opties meegeven, zoals het instellen van de log-level:

```bash 
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 --log-level debug
```
Het --log-level debug- zorgt ervoor dat er meer informatie in de terminal wordt getoond. Devolgende log-levels zijn beschikbaar: trace, debug, info, warn, error, fatal.


:::

::::

:::::


