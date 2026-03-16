# microROS Ultrasoon sensor

In dit voorbeeld wordt een SFR-04 ultrasoon sensor gekoppeld aan een ESP32 device. Nadat deze is geprogrammeerd zal dit device een topic **/sensor_info** publiceren. Met de ultrasoon sensor kun je vervolgens een afstand meten.


## Openen van een microROS project
Open met Visual Code project van de range-sensor in de volgende map (alleen map selecteren):
```text
~/microROSProjects/ultrasonic_sensor
```

## SRF-04 aansluiten
![image](../images/ESP32/srf-04.jpg)

|    ESP32 Pin     | SRF-04 Pin |
|:----------------:|:----------:|
|        5V        |    VCC     |
|       GND        |    GND     |
| SR04_TRIG_PIN(*) | Trig       |
| SR04_ECHO_PIN(*) | Echo       |
 
 (*) Deze pin-aansluitingen (van je gekozen ESP32 device) kun je vinden in het *platformio.ini* bestand:
```bash
~/microROSProjects/ultrasonic_sensor/platformio.ini
``` 
## Testen
Nadat het device is geprogrammeerd kun je de werking controleren met;
```bash
ros2 topic echo /sensor_info
```
{octicon}`alert;2em;sd-text-info` Je dient wel eerst de microROS-agent te starten. 

[Brief instructions programming ESP32 devices with VisualCode/Platform IO](instructions_programming_esp32.md)