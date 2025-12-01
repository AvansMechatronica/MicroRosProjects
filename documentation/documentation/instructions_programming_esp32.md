# Programmeren van een ESP32 device met Visual Code en PlatformIO

In dit hoofdstuk worden de belangrijkste functies beschreven voor het programmeren van een ESP32 device met Visual Code en de PlatformIO plugin.

1. Start Visual Code
2. Schakel PlatformIO omgeving in (zie: Visual Code/PlatformIO documentatie)
    * [Visual Code docs](https://code.visualstudio.com/docs)
    * [PlatformIO docs](https://docs.platformio.org/en/latest/core/index.html)
3. Selecteer ESP32 workspace:
    * __Bestand-->Open Workspace uit bestand...-->esp32.code-workspace__ (in de ESP32 map van deze repository)

(_Waarschuwing: Soms toont Visual Code zijn dialoogvensters achter het hoofdvenster, schakel over naar het dialoogvenster_)

![Image](../images/PlatformIO//ScreenshotVisualCodeLayout.png)
Commando's kunnen worden gegeven via de statusbalk aan de onderkant van het Visual Code venster, zie hieronder.
    
![Image](../images/PlatformIO/ScreenshotStatusBar.png)

4. Selecteer project om de ESP32 te programmeren, door over de statusbalk te bewegen en klik op __Switch PlatormIO project Environment__.

![Image](../images/PlatformIO/ScreenshotSelectProjectStatusBar.png)
    Selecteer vervolgens project uit de Projectenlijst:

![Image](../images/PlatformIO/ScreenshotSelectProject.png)

5. Bouw project, door over de statusbalk te bewegen en klik op de __Build__ knop, er mogen geen fouten verschijnen in de Terminal

![Image](../images/PlatformIO/ScreenshotCompile.png)
6. Verbind ESP32 device met de computer via een USB-kabel.

7. Upload het programma naar de ESP32, door over de statusbalk te bewegen en klik op de __Upload__ knop, er mogen geen fouten verschijnen in de Terminal

![Image](../images/PlatformIO/ScreenshotUpload.png)

7. Als alternatief kun je de output van het programma volgen in het serial monitor venster, door over de statusbalk te bewegen en klik op de __Serial Monitor__ knop.

![Image](../images/PlatformIO/ScreenshotTerminal.png)  
[terug](ultrasonic_sensor.md)

