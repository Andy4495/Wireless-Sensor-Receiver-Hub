# Wireless Sensor Receiver Hub

![Fully assembled Hub: Ethernet Shield (bottom), Shield-LaunchPad Interface (middle-bottom), MSP-EXP430FR6989 LaunchPad (middle-top), and CC110L BoosterPack (top).](jpg/hub.jpg)

This sketch is an example of a wireless receiver hub which receives sensor readings from various low-power, battery-operated remote sensors. These sensors use a short-range, low-power communication protocol implemented with CC110L [BoosterPack][2] from Texas Instruments. The hub receives periodic readings from the remote sensors and forwards the data using MQTT to the [ThingSpeak][5] IoT platform over ethernet.

The sketch is specifically designed to run on an MSP-EXPFR6989 [LaunchPad][1], CC110L [BoosterPack][2], and W5200-based [ethernet shield][3]. It will probably run on other MSP430-based lauchpads with sufficient program memory and minor code changes.

The code is not very generic and is currently written to support two types of sensors with specific data structures:

- An outdoor weather sensing station based on the SENSORS BoosterPack
- Simple, low-power MSP430 module using its internal temperature sensor

The current design sends the received sensor readings to the [ThingSpeak][5] IOT platform. Previous iterations used [Adafruit's][4] and [Cayenne's][9] platforms.

The built-in LCD display on the LaunchPad is used to display current temperature and battery level from the weather station sensor, plus connection status with the MQTT server.

An additional external OLED display is also supported, and can be used to print miscellaneous status information. This OLED display is optional and not necessary for basic receiver hub operation.

Additional sensors and changes to the data structures of the existing sensors would require changes to this sketch. Feel free to use this as a starting point, but do not expect it to work as-is for your application.

See the [Hardware](./Hardware) folder for specific hardware details.

Please read the sketch comments for details on the software operation.

## External Header Files

In order to compile the code, two header files are needed:

- MQTT_private_config.h
- MQTT_private_feeds.h

The details on the contents of these files are explained in the comments.

These files are not included in this repository because they contain secrets specific to your installation.

## Thingspeak API updates

ThingSpeak released an update in July 2021 ([R2021a][10]) which changed how MQTT API authentication works. Previously, the MQTT connection was authenticated using your ThingSpeak username and MQTT API key. This method has now been deprecated, and the new method uses a separate MQTT device client ID and password to authenticate. This also eliminates the need for API Keys in the MQTT topic.

In addition to the code changes included in this repository, some minor changes are also required in the "secrets" files that are not included with this repo:

- In `MQTT_private_config.h`:
  - Update `TS_USERNAME` and `TS_KEY` definitions with the new MQTT device Username and Password credentials
  - Change `TS_SERVER` definition to `mqtt3.thingspeak.com`
  - Add `#define TS_CLIENTID` with your MQTT device Client ID (which is typically the same as the Username)
- In `MQTT_private_feeds.h`
  - Remove the API keys from the `Adafruit_MQTT_Publish` constructors.
  - Also make sure there is no slash (`/`) after `publish` in each of the constructors (that is, the MQTT "topic" should be of the form `channels/<channel_ID>/publsh` and NOT end in a slash)

## External Libraries

- [Adafruit_MQTT](https://github.com/adafruit/Adafruit_MQTT_Library)  
  *See the Adafruit_MQTT [readme][8] for changes necessary to support long `connect` messages, which are generated when using the [Cayenne][9] platform.*
- [WIZnet Ethernet](https://github.com/Wiznet/WIZ_Ethernet_Library)  
  *[Modifications](./Ethernet.md) are required to work with Energia and fix a memory leak.*
- [NewhavenOLED](https://github.com/Andy4495/NewhavenOLED)  
  *The NewhavenOLED library is only needed if you plan to use an external OLED display as mentioned above.*

## Example Remote Sensors

- [MSP430 Low Power Temperature Sensor][14]
- [MSP430 Temperature Sensor with Display][11]
- [Outdoor Weather Sensor][12]
- [Sensor Repeater][13]

## References

- [MSP430FR6989 LaunchPad][1]
- [CC110L BoosterPack][2]
- [Seeed Studio W5200 Ethernet Shield][3]
- [ThingSpeak][5] IoT Platform
- ThingSpeak [release notes][10].

## License

The software and other files in this repository are released under what is commonly called the [MIT License][100]. See the file [`LICENSE.txt`][101] in this repository.

[1]: http://www.ti.com/tool/MSP-EXP430FR6989
[2]: http://www.ti.com/tool/430BOOST-CC110L
[3]: http://wiki.seeedstudio.com/Ethernet_Shield_V2.0/
[4]: https://io.adafruit.com/
[5]: https://thingspeak.com/
[8]: ./Adafruit_MQTT.md
[9]: https://cayenne.mydevices.com
[10]: https://www.mathworks.com/help/thingspeak/release-notes.html
[11]: https://github.com/Andy4495/MSP430TempSensorWithDisplay
[12]: https://github.com/Andy4495/Outdoor-Weather-Sensor
[13]: https://github.com/Andy4495/Sensor-Repeater
[14]: https://github.com/Andy4495/MSP430LowPowerTempSensor
[100]: https://choosealicense.com/licenses/mit/
[101]: ./LICENSE.txt
[200]: https://github.com/Andy4495/Wireless-Sensor-Receiver-Hub
