# Wireless Sensor Receiver Hub

[![Arduino Compile Sketches](https://github.com/Andy4495/Wireless-Sensor-Receiver-Hub/actions/workflows/arduino-compile-sketches.yml/badge.svg)](https://github.com/Andy4495/Wireless-Sensor-Receiver-Hub/actions/workflows/arduino-compile-sketches.yml)
[![Check Markdown Links](https://github.com/Andy4495/Wireless-Sensor-Receiver-Hub/actions/workflows/check-links.yml/badge.svg)](https://github.com/Andy4495/Wireless-Sensor-Receiver-Hub/actions/workflows/check-links.yml)

![Fully assembled Hub: Ethernet Shield (bottom), Shield-LaunchPad Interface (middle-bottom), MSP-EXP430FR6989 LaunchPad (middle-top), and CC110L BoosterPack (top).](jpg/hub.jpg)

This sketch is an example of a wireless receiver hub which receives sensor readings from various low-power, battery-operated remote sensors. These sensors use a short-range, low-power communication protocol implemented with CC110L [BoosterPack][2] from Texas Instruments. The hub receives periodic readings from the remote sensors and forwards the data using MQTT to the [ThingSpeak][5] IoT platform over ethernet.

The sketch is specifically designed to run on an MSP-EXPFR6989 [LaunchPad][1], CC110L [BoosterPack][2], and W5200-based [ethernet shield][3]. It will probably run on other MSP430-based lauchpads with sufficient program memory and minor code changes.

The code is not very generic and is currently written to support two types of sensors with specific data structures:

- An outdoor weather sensing station based on the SENSORS BoosterPack
- Simple, low-power MSP430 module using its internal temperature sensor

The current design sends the received sensor readings to the [ThingSpeak][5] IOT platform. Previous iterations used [Adafruit's][4] and [Cayenne's][9] platforms.

The built-in LCD display on the LaunchPad is used to display current temperature and battery level from the weather station sensor, plus connection status with the MQTT server (the antenna symbol).

An external OLED display is also supported, and can be used to print miscellaneous status information. This OLED display is optional and not necessary for basic receiver hub operation.

Additional sensors and changes to the data structures of the existing sensors would require changes to this sketch. Feel free to use this as a starting point, but do not expect it to work as-is for your application.

See the [Hardware](./Hardware) folder for specific hardware details.

Please read the sketch comments for details on the software operation.

## Header Files for User-Specific Info

In order to compile the code, an additional header file is needed: `MQTT_private_config.h`.

A template of the file named `MQTT_private_config-template` is included in the repo. This file should be updated with data specific to your configuration and then copied to `MQTT_private_config.h`.

In addition, `MQTT_publishing_feeds.h` should be updated with channel information specific to your application.

## Thingspeak API updates

ThingSpeak updated their MQTT interface in July 2021 with release [R2021a][10], which changed how MQTT API authentication works.

Instead of authenticating using your ThingSpeak username and MQTT API key, the updated method uses a separate MQTT device client ID and password. In addition, the ThingSpeak MQTT server is now named `mqtt3.thingspeak.com`.

The code and header files have been updated with these changes.

## External Libraries

- [Modified version][15] of Adafruit MQTT Library version 1.3.0
- [Modified version][17] of the WIZnet Ethernet Library
- [NewhavenOLED](https://github.com/Andy4495/NewhavenOLED)  
  *The NewhavenOLED library is only needed if you plan to use an external OLED display as mentioned above.*

## Supported Remote Sensors

- [MSP430 Low Power Temperature Sensor][14]
- [MSP430 Temperature Sensor with Display][11]
- [MSP430 Temperature Sensor with Built-in LCD][19]
- [Outdoor Weather Sensor][12]
- [Sensor Repeater][13]

## References

- [MSP430FR6989 LaunchPad][1]
- CC110L BoosterPack [Quick Start Guide][2]
- [Seeed Studio W5200 Ethernet Shield][3]
- [ThingSpeak][5] IoT Platform
- ThingSpeak [release notes][10]
- Original [WIZnet Ethernet Library][18]
- Original [Adafruit MQTT Library][16]

## License

The software and other files in this repository are released under what is commonly called the [MIT License][100]. See the file [`LICENSE.txt`][101] in this repository.

[1]: http://www.ti.com/tool/MSP-EXP430FR6989
[2]: https://www.ti.com/lit/ml/swru312b/swru312b.pdf
[3]: http://wiki.seeedstudio.com/Ethernet_Shield_V2.0/
[4]: https://io.adafruit.com/
[5]: https://thingspeak.com/
[9]: https://cayenne.mydevices.com
[10]: https://www.mathworks.com/help/thingspeak/release-notes.html
[11]: https://github.com/Andy4495/MSP430TempSensorWithDisplay
[12]: https://github.com/Andy4495/Outdoor-Weather-Sensor
[13]: https://github.com/Andy4495/Sensor-Repeater
[14]: https://github.com/Andy4495/MSP430LowPowerTempSensor
[15]: https://github.com/Andy4495/Adafruit_MQTT_Library-1.3.0
[16]: https://github.com/adafruit/Adafruit_MQTT_Library
[17]: https://github.com/Andy4495/WIZ_Ethernet_Library
[18]: https://github.com/Wiznet/WIZ_Ethernet_Library
[19]: https://github.com/Andy4495/MSP430TempSensorLCD
[100]: https://choosealicense.com/licenses/mit/
[101]: ./LICENSE.txt
[//]: # ([200]: https://github.com/Andy4495/Wireless-Sensor-Receiver-Hub)
