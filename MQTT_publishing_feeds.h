/* MQTT publishing feeds
     Each feed that you wish to publish needs to be defined below.
*/

// New Thingspeak format
Adafruit_MQTT_Publish Weather_Channel = Adafruit_MQTT_Publish(&thingspeak,
                                        "channels/" TS_WEATHER_CHANNEL_ID "/publish");
Adafruit_MQTT_Publish Temp_Sensor4 = Adafruit_MQTT_Publish(&thingspeak,
                                     "channels/" TS_TEMP4_CHANNEL_ID "/publish");
Adafruit_MQTT_Publish Temp_Sensor5 = Adafruit_MQTT_Publish(&thingspeak,
                                     "channels/" TS_TEMP5_CHANNEL_ID "/publish");
Adafruit_MQTT_Publish Temp_Slim = Adafruit_MQTT_Publish(&thingspeak,
                                  "channels/" TS_SLIM_CHANNEL_ID "/publish");
Adafruit_MQTT_Publish Weather_Meta = Adafruit_MQTT_Publish(&thingspeak,
                                     "channels/" TS_WEATHER_META_CHANNEL_ID "/publish");
Adafruit_MQTT_Publish Sensor_Errors = Adafruit_MQTT_Publish(&thingspeak,
                                      "channels/" TS_SENSOR_ERRORS_CHANNEL_ID "/publish");
Adafruit_MQTT_Publish Temp_Sensor6 = Adafruit_MQTT_Publish(&thingspeak,
                                     "channels/" TS_TEMP6_CHANNEL_ID "/publish");
Adafruit_MQTT_Publish Pond_Sensor = Adafruit_MQTT_Publish(&thingspeak,
                                     "channels/" TS_POND_CHANNEL_ID "/publish");
