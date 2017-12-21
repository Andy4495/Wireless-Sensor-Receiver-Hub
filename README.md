Wireless Sensor Receiver Hub
============================

- ![Fully assembled Hub: Ethernet Shield (bottom), Shield-LaunchPad Interface (middle-bottom), MSP-EXP430FR6989 LaunchPad (middle-top), and CC110L BoosterPack (top).] (jpg/hub.jpg)

This sketch is designed to run on an MSP-EXPFR6989 LaunchPad, CC110L BoosterPack, and W5200-based ethernet shield. It should run on other MSP430-based lauchpads with sufficient program memory.
The code is not very generic and is currently written to support two sensors with specific data structures:
- An outdoor weather sensing station based on the SENSORHUB BoosterPack
- A temperature sensor based on an MSP430G2553

The built-in LCD display on the LaunchPad is used to display current temperature and battery levels from the sensors, plus connection status with the MQTT server. The full sensor data is sent to the MQTT server and is also output through the Serial USB backchannel at 9600 baud.

Additional sensors and changes to the data structures of the existing sensors would require changes to this sketch. Feel free to use this as a starting point, but do not expect it to work as-is for your application.

See the Hardware folder for specific hardware details.

Please read the .ino file comments for details on the software operation.

## External Header Files ##

In order to compile the code, two header files are needed:
* MQTT_private_config.h
* MQTT_private_feeds.h

The details on the contents of these files are in the .ino file comments.

## External Libraries ##
* Adafruit_MQTT
* Ethernet (from WizNet)
