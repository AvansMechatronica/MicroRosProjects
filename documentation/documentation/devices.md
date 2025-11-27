# microROS
# ESP32 devices
In deze workshop wordt gewerkt met de volgende ESP32 devices:

:::::{card} 
::::{tab-set}

:::{tab-item} ESP32-WROOM

![image](../../images/ESP32/esp32.jpg)
Environmet: eps32-upesy_wroom
Environment: **eps32-upesy_wroom** 
```text
platform = espressif32
board = esp32-s3-devkitc-1
```
[Info](https://www.espboards.dev/esp32/upesy-wrover/)


:::

:::{tab-item} ESP32-WROOM-Mini
![image](../../images/ESP32/esp32-mini.jpg)
Environment: **eps32-upesy_wroom** 
```text
platform = espressif32
board = esp32-s3-devkitc-1
```
[Info](https://www.espboards.dev/esp32/upesy-wrover/)

{octicon}`alert;2em;sd-text-info`Deze ESP32-WROOM-Mini heeft dezelfde specificatie als de ESP32-WROOM, echter de pinout is afwijkend:

![Pinots](../../images/ESP32/esp32-mini-pinout.png)
:::

:::{tab-item} ESP32C3-Mini

![image](../../images/ESP32/esp32-c3-mini.jpg)
Environment: **esp32-C3** 
```text
platform = espressif32
board = lolin_c3_mini
```
[Info](https://www.espboards.dev/esp32/esp32-c3-super-mini/)

:::

:::{tab-item} ESP32S3-Mini
![image](../../images/ESP32/esp32-S3-mini.jpg)
Environment: **esp32-S3** 
```text
platform = espressif32
board = esp32-s3-devkitc-1
```
[Info](https://www.espboards.dev/esp32/esp32-s3-zero/)

:::

::::

:::::

*In het platformio.ini bestand van het project kun je details m.b.t. compileropties voor ieder environment inzien.*

## Uploaden programma naar ESP32 devices
### Environment selecteren
Alvorens je een microROS programma kan compileren dien je de juiste environment te selecteren die bij het het gekozen ESP32 board past.
Activeer de environment slectector:

![image](../../images/PlatformIO/SelectEnvironmentPre.JPG)

Kies daarna de juiste environmet:

![image](../../images/PlatformIO/SelectEnvironment.JPG)

### Software Compileren
Je kunt het programma voor het device compileren met onderstaande functie:

![image](../../images/PlatformIO/Compile.JPG)

### Uploaden naar ESP32 device
Nadat je het device hebt aangesloten via USB kun je de software naar het device uploaden:

![image](../../images/PlatformIO/Upload.JPG)

Nadat de software succesvol is geüpload zal het programma automatisch in het device starten.

Tip: *Tijdens het uploaden zal getoond worden welke USB-device hiervoor gebruikt wordt. Maak hiervan een aantekening deze heb je nodig bij het starten van de microROS-agent.*

{octicon}`alert;2em;sd-text-info` Bij gebruik van WSL moet je het embedded systeem wel eerst met WSL verbinden, zie [Koppelen USB-devices aan WSL-Distributie](https://avansmechatronica.github.io/WindowsSubsystemForLinuxHandleiding/documentation/WSL_Handleiding.html#koppelen-usb-devices-aan-wsl-distributie)

{octicon}`alert;2em;sd-text-info`Zorg er voor dat bij het uploaden de bijbehorende microROS-agent is afgesloten.

{octicon}`alert;2em;sd-text-info`Start in geen geval de *Serial Monitor*. Deze zal de communicatie over USB naar de microROS-agent blokkeren.

![image](../../images/PlatformIO/NoSerialMonitor.JPG)


## Voorbeelden
Er zijn in het kader van deze modules al twee microROS implementaties gerealiseerd voor de volgende workshops:

* ROS2 Basics: [range-sensor](../../1_basics/ESP32/ultrasonic_sensor.md)

* Manipulation: [joystick](../../3_navigation/ESP32/joystick.md)

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

:::

:::{tab-item} Wifi

{octicon}`alert;2em;sd-text-info` Dit werkt alleen als in *platformio.ini* bestand de parameter **board_microros_transport = wifi** is gezet.

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```
:::

::::

:::::




