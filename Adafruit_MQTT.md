Adafruit MQTT Library Notes
===========================

The Wireless Sensor Receiver Hub makes use of the Adafruit [MQTT library][2]. This choice was mainly based on the fact that the project was initially designed to make use of the [AdafruitIO][3] platform for IoT data storage and visualization.

While follow-on iterations of the project moved to [ThingSpeak][4], it was convenient to continue using the Adafruit library and therefore limit the code changes necessary to connect to the ThnkSpeak servers.

However, while trying to use the [Cayenne][5] IoT platform, a shortcoming in the Adafruit MQTT libraries became apparent: the library does not support MQTT messages longer than 127 bytes. In particular, it only supports a single-byte Remaining Length field instead of the [protocol-defined][1] variable-length sized field. The Cayenne API makes use of particularly long username, password, and clientID strings, all of which are required as part of the `connect` message. This results in the message being 128 bytes long, which is then encoded incorrectly by the Adafruit library.

This is [documented][6] on the Adafruit GitHub site, and as of this writing, there is no ETA for a fix from Adafruit.

Below are instructions for fixing the problem for the connect message. Similar updates are probably also necessary for the `subscribe` and messages. The `publish` message handler includes code that appears to properly handle the Remaining Length field.

### Modifications to Adafruit MQTT library  ###

1. Install the library as you normally would. This can be done by using the library manager in the Arduino IDE, or manually installing the library by downloading the ZIP from GitHub.
2. Edit the file `Adafruit_MQTT.cpp` and replace lines 626 - 628:  

        len = p - packet;  
        packet[1] = len-2;  // don't include the 2 bytes of fixed header data

  With the following lines:

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
3. Because the Cayenne API creates longer messages, you may also want to increase the packet buffer size. Note that this causes the library to use more RAM. The buffer size is defined in `Adafruit_MQTT.h`:   

    #define MAXBUFFERSIZE 150  

In my implementation, I changed the default value of 150 to 200.

### References ###
- MQTT v3.1.1 Specification, Section 2.2.3 [Remaining Length][1] field definition
- Adafruit [MQTT library][2]  
- [AdafruitIO][3]
- [ThingSpeak][4]
- [Cayenne][5]

[1]: http://docs.oasis-open.org/mqtt/mqtt/v3.1.1/os/mqtt-v3.1.1-os.html#_Toc398718023
[2]: https://github.com/adafruit/Adafruit_MQTT_Library
[3]: https//io.adafruit.com
[4]: https://thingspeak.com/
[5]: https://cayenne.mydevices.com
[6]: https://github.com/adafruit/Adafruit_MQTT_Library/issues/79
