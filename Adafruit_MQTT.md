# Adafruit MQTT Library Notes

The Wireless Sensor Receiver Hub makes use of the Adafruit [MQTT library][2]. This choice was mainly based on the fact that the project was initially designed to make use of the [AdafruitIO][3] platform for IoT data storage and visualization.

While follow-on iterations of the project moved to [ThingSpeak][4], it was convenient to continue using the Adafruit library and therefore limit the code changes necessary to connect to the ThnkSpeak servers.

Adafruit has since made some API changes which are incompatible with this sketch. In addition, a minor change is needed because the MSP430 compiler does not support the `atof()` function.

For this reason, I have cloned version 1.3.0 of the Adafruit MQTT library and updated it so it works with the MSP430 compiler. This [specific cloned version][7] is required to compile the Wireless-Sensor-Receiver-Hub.ino sketch.

## Long Messages

While trying to use the [Cayenne][5] IoT platform, a shortcoming in the Adafruit MQTT library became apparent: the library does not support MQTT messages longer than 127 bytes. In particular, it only supports a single-byte Remaining Length field instead of the [protocol-defined][1] variable-length sized field. The Cayenne API makes use of particularly long username, password, and clientID strings, all of which are required as part of the `connect` message. This results in the message being 128 bytes long, which is then encoded incorrectly by the Adafruit library.

This is [documented][6] on the Adafruit GitHub site. That particular issue does not have an ETA for a fix.

Since I am now using the ThingSpeak IoT platform instead of Cayenne, I am not running into this issue with my current setup. However, the changes required are outlined below if needed.

## Modifications to Adafruit MQTT Library to Support Long Messages

These changes are specific to version 1.3.0 of the Adafruit MQTT Library. Other versions of the library may be at different line numbers or have the problem fixed.

1. Edit the file `Adafruit_MQTT.cpp` and replace lines 660 - 662:  

   ```cpp
   len = p - packet;  

   packet[1] = len-2;  // don't include the 2 bytes of fixed header data
   ```

   With the following lines:

   ```cpp
   len = p - packet - 2; // don't include the 2 bytes of fixed header data
   if (len < 128) {
     packet[1] = len;
     len = len + 2;
   }
   else {  // This will handle up to 2-byte packet length
     memmove(packet+3, packet+2, len);
     packet[1] = 0x80 | len % 128;
     packet[2] = len / 128;
     len = len + 3;
   }
   ```

2. Certain IoT APIs create longer messages (e.g.[Cayenne][5], so you may also want to increase the packet buffer size. Note that this causes the library to use more RAM. The buffer size is defined in `Adafruit_MQTT.h` line 110:

   ```cpp
   #define MAXBUFFERSIZE (150) 
   ```

   In my implementation, I increased the buffer size from 150 to 200.

Similar updates are probably also necessary for the `subscribe` and messages. The `publish` message handler includes code that appears to properly handle the Remaining Length field.

## References

- MQTT v3.1.1 Specification, Section 2.2.3 [Remaining Length][1] field definition
- Adafruit [MQTT library][2]  
- Adafruit MQTT library [packet length issue][6]
- [AdafruitIO][3]
- [ThingSpeak][4]
- [Cayenne][5]

[1]: http://docs.oasis-open.org/mqtt/mqtt/v3.1.1/os/mqtt-v3.1.1-os.html#_Toc398718023
[2]: https://github.com/adafruit/Adafruit_MQTT_Library
[3]: https://io.adafruit.com
[4]: https://thingspeak.com/
[5]: https://cayenne.mydevices.com
[6]: https://github.com/adafruit/Adafruit_MQTT_Library/issues/79
[7]: https://github.com/Andy4495/Adafruit_MQTT_Library-1.3.0
[200]: https://github.com/Andy4495/Wireless-Sensor-Receiver-Hub

[//]: # ( )
