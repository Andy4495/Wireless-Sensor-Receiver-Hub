/**
   Sensor Receiver Station
   https://github.com/Andy4495/Sensor-Receiver

   1.0 - 11/19/17 - A.T. - Original
   1.1 - 11/19/17 - A.T. - Display info on built-in LCD
   1.2 - 11/20/17 - A.T. - Press PUSH1 to cycle LCD display
   1.3 - 11/24/17 - A.T. - Make LED selection with DEFINE
                         - LCD display controlled with DEFINE
                         - Moved message processing into switch
                           statement and function calls.
   2.0 - 11/26/17 - A.T. - Added ethernet and MQTT support.
   2.1 - 12/07/17 - A.T. - Moved MQTT config to private header.
                         - Added separate control of W5200 RESET
                           - Previously tied to MSP430 RST
                           - This resolves a power-up timing issue
                             and also allows the MSP to reset the
                             ethernet chip if necessary.
   2.2 - 12/20/17 - A.T. - Reset ethernet board if connection lost
                           Stop sending BMP180_T to MQTT
                           Display SHT21_T instead of BMP180_T
   2.3 - 12/21/17 - A.T. - Add support for Newhaven OLED display:
                           - Press PUSH2 to temporarily turn on OLED
                           - OLED displays RSSI, LQI, and seconds since
                             last message for each sensor.
   2.4 - 01/15/18 - A.T. - Add CRC check on received packet.
                           Write "0" battery level to MQTT if CRC failed.
   2.5 - 01/16/18 - A.T. - Change lux unit to long int.
   2.6 - 01/21/18 - A.T. - Additional temp sensor modules.
   2.7 - 01/25/18 - A.T. - Add support for ThingSpeak IoT
                         - Put constant strings in F() macro for better
                           compatiblity for running on Arduino
   2.8 - 01/30/18 - A.T. - Tickle a pin for external watchdog module.
                           Add program FRAM write protection to FR6989
                           Optimize buffer sizes to reduce RAM usage.
   3.0 - 02/01/18 - A.T. - Update message structure to align on word boundary.
   3.1 - 02/02/18 - A.T. - Combine Rx buffer, weather struct, temp struct into
                           union to save RAM and decrease copying of data.
                           Disable OLED by default.
   3.2 - 03/18/18 - A.T. - Add another temp sensor (ID#6), includes additional
                           sensor data. Updated temsensor data structure.
                           New elements added to end of existing structure, so
                           it is backwards-compatible with existing remote
                           temp sensors.
   4.0 - 04/07/18 - A.T. - Change from AIO to Cayenne IOT MQTT server.
   4.1 - 04/08/18 - A.T. - Cayenne fixes: LUX data type, door sensor changed to digital
                           Send RX HUB uptime on Garage channel 7
   4.2 - 04/15/18 - A.T. - Make Serial writes optional at compile time
                           - Note that debug/error modes also need to be disabled in Adafruit_MQTT.h
                             to fully disable serial
                           Replace sprintf with snprintf when using fieldBuffer
   4.3 - 04/24/18 - A.T. - Send Rx Hub Ethernet uptime on Garage channel 3 to ThingSpeak (replacing loops).
   4.4 - 08/21/18 - A.T. - Stop sending status field without any other fields, since that causes nulls to be sent when queried.
                           Now keep track of cumulative CRC errors and send as part of status field on every message.
                           Continue sending CRC error messages to Sensor_Errors feed, just not the data feed.
   4.5 - 09/09/18 - A.T. - Add support from pond sensor (received from the repeater).
                           Initially uses same RX ID as weather sensor; will be changed to its own ID in future.
   4.6 - 10/07/18 - A.T. - Add support for battery temperature on pond sensor.
   4.7 - 10/29/18 - A.T. - Remove built-in LCD status, use optional external OLED instead
                           No longer send updates to Cayenne
                           General code cleanup
   4.8 - 02/02/19 - A.T. - Display outdoor temp on built-in LCD.
   4.9 - 02/02/19 - A.T. - Display outdoor weather station battery level on built-in LCD.
*/

/**
   Receiver hub for various sensor transmitting modules.
   Uses Anaren CC110L BoosterPack as receiver module.
   ** Note that the program will freeze in setup() if
   ** the CC110L BoosterPack is not installed.

   The current supported ethernet board is the Seeed Studio
   W5200 Ethernet Shield designed for Arduino. This code
   may work with other W5200-based ethernet cards. The
   ethernet library needs to be modified to work with chips
   other than the WIZnet W5200.

   Currently defined transmitters
   - Outdoor weather station using SENSORHUB BoosterPack
   - Simple temp monitor using MSP430 internal sensor
   - Additional MSP430 temp sensors using same data structure
   - Other transmitters may be defined in the future

   The receiving station code needs to be modified whenever
   a new sensor is added:
   - Sensor CC110L device address
   - Data payload structure
   - Output stream (Serial, LCD, and/or IP)

   The sketch toggles Pin 12 every time through loop() so
   long as the there was a valid ThingSpeak connection. An external
   watchdog module can monitor the signal and reset the
   receiver hub if the receiver hub unable to reconnect.

**/
/**
   EXTERNAL LIBRARIES:
   - Ethernet: https://github.com/Wiznet/WIZ_Ethernet_Library
       - Modified to work with Energia
       - #define updates to support Seeed W5200 Ethernet Shield V2.2
   - MQTT: https://github.com/adafruit/Adafruit_MQTT_Library
       - Adafruit_MQTT.cpp modified to comment out lines 425-431
         to remove support for floating point. Specifically,
         commented out the block starting with:
            "else if (sub->callback_double != NULL)"
   - NewhavenOLED: https://github.com/Andy4495/NewhavenOLED
       - Only used if OLED_ENABLED is #defined
*/

#include <SPI.h>
#include <AIR430BoostFCC.h>

/* BOARD CONFIGURATION AND OTHER DEFINES
   -------------------------------------

   To use an external Newhaven OLED display:
      #define OLED_ENABLED
   Comment out the line if the OLED is not used:
      //#define OLED_ENABLED
   The use of an external OLED is entirely optional and just
   allows the display of some information that is already
   sent out the serial port and/or MQTT.
   Using the OLED takes up about 4K additional program space.

   To enable ethernet and uploading data to MQTT server:
      #define ETHERNET_ENABLED
      You will also need to set up your various MQTT feeds below.
   Otherwise, comment out the line:
      //#define ETHERNET_ENABLED
   ** Note that the program will freeze in setup() if the proper
   ** ethernet card is not configured.

   To reduce output sent to Serial, keep the following line commented:
     //#define PRINT_ALL_CLIENT_STATUS
   It can be uncommented to help debug connection issues

   To reduce RAM (and program space) usage, disable Serial prints by
   commenting the line:
     //#define SKETCH_DEBUG
*/
#define OLED_ENABLED
#define ETHERNET_ENABLED
//#define PRINT_ALL_CLIENT_STATUS
//#define SKETCH_DEBUG
// -----------------------------------------------------------------------------

#ifdef SKETCH_DEBUG
#define SKETCH_PRINT(...) { Serial.print(__VA_ARGS__); }
#define SKETCH_PRINTLN(...) { Serial.println(__VA_ARGS__); }
#else
#define SKETCH_PRINT(...) {}
#define SKETCH_PRINTLN(...) {}
#endif

#if defined(__MSP430FR4133__)
#define LCD_ENABLED
#define BOARD_LED LED2
#endif

#if defined(__MSP430FR6989__)
#define LCD_ENABLED
#define BOARD_LED GREEN_LED
#endif

#if defined(__MSP430G2553__)
#define BOARD_LED GREEN_LED
#endif

#if defined(__MSP430F5529__)
#define BOARD_LED GREEN_LED
#endif

#if defined(__MSP430FR2433__)
#define BOARD_LED LED2
#endif

#if defined(__MSP430FR5969__)
#define BOARD_LED LED2
#endif

// -----------------------------------------------------------------------------
#ifdef LCD_ENABLED
#include "LCD_Launchpad.h"
LCD_LAUNCHPAD myLCD;
#endif
// -----------------------------------------------------------------------------
#ifdef OLED_ENABLED
/* OLED PIN AND HARDWARE DEFINITIONS
   ---------------------------------
   OLED_CS   - Active LOW.
   OLED_SI   - MOSI data line from MSP to OLED
   OLED_SCK  - Serial clock line from MSP to OLED
   OLED_ROWS - # of rows of characters on the OLED
   OLED_COLS - # of character columns on the OLED

   Note that the OLED library used is a bit-bang SPI implementation,
   and therefore the OLED is not connected to the SPI bus used
   by the CC110L and Ethernet shield.
*/

#define OLED_CS   38
#define OLED_SI   37
#define OLED_SCK  36
#define OLED_ROWS  2
#define OLED_COLS 16
#include <NewhavenOLED.h>
const byte row_address[2] = {0x80, 0xC0};   // DDRAM addresses for rows (2-row models)
NewhavenOLED oled(OLED_ROWS, OLED_COLS, OLED_SI, OLED_SCK, OLED_CS, NO_PIN);
byte oled_text[OLED_ROWS][OLED_COLS + 1] =
{ "Sensor Rx Hub   ",
  "    Initializing"
};
#define DISPLAY_TIMEOUT 8
int displayTimeoutCount = DISPLAY_TIMEOUT;
#endif
// -----------------------------------------------------------------------------
#ifdef ETHERNET_ENABLED
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Ethernet.h>
#include <EthernetClient.h>
#include <Dns.h>
#include <Dhcp.h>
#include "MQTT_private_config.h"
/* The MQTT_private_config.h file needs to include the following definitions
   specific to your configuration:
     byte mac[] = {6 byte MAC address for ethernet card};
     #define AIO_SERVER      "address of your MQTT server (e.g. io.adafruit.com)"
     #define AIO_SERVERPORT  Port number of your MQTT server, e.g. 1883
     #define AIO_USERNAME    "Username for MQTT server account"
     #define AIO_KEY         "MQTT key required for your MQTT server account"
   If using ThingSpeak, then WRITE keys for each channel may also be #defined here.
*/
EthernetClient client_ts;
Adafruit_MQTT_Client thingspeak(&client_ts, TS_SERVER, TS_SERVERPORT, TS_USERNAME, TS_KEY);
#define PAYLOADSIZE 132
char payload[PAYLOADSIZE];    // MQTT payload string
#define FIELDBUFFERSIZE 20
char fieldBuffer[FIELDBUFFERSIZE];  // Temporary buffer to construct a single field of payload string
/***** MQTT publishing feeds *****
   Each feed/channel that you wish to publish needs to be defined.
     - ThingSpeak Channels follow the form: channels/<CHANNEL_ID>/publish/<WRITE_API_KEY>, for example:
         Adafruit_MQTT_Publish myChannel = Adafruit_MQTT_Publish(&mqtt,
                                        "channels/" CHANNEL_ID "/publish/" CHANNEL_WRITE_API_KEY);
         See https://www.mathworks.com/help/thingspeak/publishtoachannelfeed.html
   The file "MQTT_private_feeds.h" needs to include the feed/channel definitions
   specific to your configuration.
*/
#include "MQTT_private_feeds.h"
#endif
// -----------------------------------------------------------------------------

/* -----------------------
   CONTROL PIN DEFINITIONS
   -----------------------

    W5200_RESET - Connected to RESET on W5200 chip. Active LOW.
    W5200_CS    - Connected to SCS pin on W5200 chip. Active LOW.
    CC110L_CS   - Connected to CS pin on CC110L chip. Active LOW.

    NOTE: The CS pins are also defined separately in the associated driver
         libraries. Both the driver libraries and the following
         definitions need to be updated in order to change the pin
         assignments
   -----------------------
*/

#define W5200_RESET 5
#define W5200_CS    8
#define CC110L_CS  18
#define RF_GDO0    19
#define WD_PIN     12             // Toggle for external watchdog

// Sensor device addresses (Radio IDs)
#define ADDRESS_LOCAL    0x01
#define ADDRESS_WEATHER  0x02
#define ADDRESS_G2       0x03
#define ADDRESS_SENSOR4  0x04
#define ADDRESS_SENSOR5  0x05
#define ADDRESS_SENSOR6  0x06
#define ADDRESS_SENSOR7  0x07
#define LAST_ADDRESS     0x07

// Declaring size of "+1" wastes a little memory, but makes the code a
// little clearer (if only the device addresses started at zero...)
unsigned int CRC_count[LAST_ADDRESS + 1];

struct WeatherData {
  int             BME280_T;  // Tenth degrees F
  unsigned int    BME280_P;  // Pressure in inches of Hg * 100
  int             BME280_H;  // % Relative Humidity
  int             TMP007_Ti; // Tenth degrees F
  int             TMP007_Te; // Tenth degrees F
  unsigned long   LUX;       // Lux units
  int             MSP_T;     // Tenth degrees F
  unsigned int    Batt_mV;   // milliVolts
  unsigned int    Loops;
  unsigned long   Millis;
};

struct TempSensor {
  int             MSP_T;     // Tenth degrees F
  unsigned int    Batt_mV;   // milliVolts
  unsigned int    Loops;
  unsigned long   Millis;
  unsigned int    Light_Sensor;
  unsigned int    Door_Sensor;
};

struct PondData {
  int             MSP_T;          // Tenth degrees F
  int             SUBMERGED_T;    // Tenth degrees F
  unsigned int    Batt_mV;        // milliVolts
  int             PUMP_STATUS;    // Unimplemented
  int             AERATOR_STATUS; // Unimplemented
  unsigned long   Millis;
  int             Battery_T;      // Tenth degrees F; only meaningful when using a Fuel Tank BoosterPack
};

// Check "struct_type" member for the type of structure to
// decode in the sPacket union.
enum {WEATHER_STRUCT, TEMP_STRUCT, POND_STRUCT};

struct sPacket  // CC110L packet structure
{
  uint8_t from;              // Local node address that message originated from
  uint8_t struct_type;       // Filler byte to keep rest of struct on word boundary
  // Also used to indicate type of struct in union
  union {
    uint8_t message[58];     // Local node message keep even word boundary
    WeatherData weatherdata;
    TempSensor  sensordata;
    PondData    ponddata;
  };
};

// -----------------------------------------------------------------------------
/**
    Global data
*/
struct sPacket rxPacket;

int lostConnectionCount = 0;
int lastRssi, lastLqi;
int crcFailed = 0; // 1 is bad CRC, 0 is good CRC
int WD_state = 0;
unsigned long lastEthernetReset = 0;
int  EthDisconnected = 0; // Ethernet status

int currentDisplay; //Remember which temp value is on LCD
// Declaring size of "+1" wastes a little memory, but makes the code a
// little clearer (if only the device addresses started at zero...):
int temperatures[LAST_ADDRESS + 1];        // Store last temp value received
int batteries[LAST_ADDRESS + 1];           // Store last battery reading
unsigned long lastTime[LAST_ADDRESS + 1];  // Store time (in millis) of last message received

void setup()
{
  // Set the CC110L Chip Select High to make sure it doesn't interfere with Ethernet
  digitalWrite(CC110L_CS, HIGH);
  pinMode(CC110L_CS, OUTPUT);

  // Set Ethernet Chip Select High to disable before initializing
  digitalWrite(W5200_CS, HIGH);
  pinMode(W5200_CS, OUTPUT);

  // Reset the Ethernet Chip
  // Pin 5 is connected to RESET on W5200 chip
  digitalWrite(W5200_RESET, HIGH);
  pinMode(W5200_RESET, OUTPUT);
  digitalWrite(W5200_RESET, LOW); // Reset W5200
  delay(5);           // Needs to be low at least 2 us; we'll go a little longer
  digitalWrite(W5200_RESET, HIGH);
  lastEthernetReset = millis();
  delay(200);         // RESET needs to be cleared for at least 150 ms before operating

  // Set up the watchdog pin
  pinMode(WD_PIN, OUTPUT);
  digitalWrite(WD_PIN, WD_state);

  // Setup serial for status printing.
#ifdef SKETCH_DEBUG
  Serial.begin(9600);
#endif
  SKETCH_PRINTLN(F(" "));
  SKETCH_PRINTLN(F("Sensor receiver hub with CC110L."));
  delay(500);

#if defined(__MSP430FR6989__)
  // Lock down FRAM - disable writes to program memory
  SKETCH_PRINTLN("Writing FRAM control registers.");
  unsigned int* MPUCTL0_reg = (unsigned int*) 0x05a0;
  unsigned int* MPUSAM_reg  = (unsigned int*)0x05a8;
  *MPUCTL0_reg = (unsigned int) 0xa501;
  *MPUSAM_reg  = (unsigned int) 0x7555;
  SKETCH_PRINT("MPUCTL0: ");
  SKETCH_PRINTLN((unsigned int) * (int*)MPUCTL0_reg, HEX);
  SKETCH_PRINT("MPUSAM: ");
  SKETCH_PRINTLN((unsigned int) * (int*)MPUSAM_reg, HEX);
#endif

#ifdef LCD_ENABLED
  myLCD.init();
#ifdef ETHERNET_ENABLED
  myLCD.displayText(F("ETHER "));
#endif
  SKETCH_PRINTLN(F("Started LCD display"));
#endif

#ifdef OLED_ENABLED
  oled.begin();
  oledDisplay();
  // Clear out the buffer
  memset(oled_text, ' ', sizeof(oled_text));
#endif

#ifdef ETHERNET_ENABLED
  SKETCH_PRINTLN(F("Starting Ethernet..."));
  Ethernet.begin(mac);
  SKETCH_PRINTLN(F("Ethernet enabled."));
  SKETCH_PRINTLN("Attempting to connect to Thingspeak...");
  MQTT_connect(&thingspeak, &client_ts);
#endif

  // Setup CC110L data structure
  rxPacket.from = 0;         // "from" and "struct_type" filled in by received message
  rxPacket.struct_type = 0;  // Zero them out here just for completeness
  memset(rxPacket.message, 0, sizeof(rxPacket.message));

  pinMode(BOARD_LED, OUTPUT);       // Flash LED to indicate ready to receive
  digitalWrite(BOARD_LED, HIGH);
  delay(500);
  digitalWrite(BOARD_LED, LOW);

  pinMode(PUSH1, INPUT_PULLUP);     // PUSH1 Cycles OLED display
  pinMode(PUSH2, INPUT_PULLUP);     // PUSH2 temporarily turns on OLED

#ifdef LCD_ENABLED
  for (int i = 0; i < LAST_ADDRESS + 1; i++) {
    temperatures[i] = 0;
    batteries[i] = 0;
    lastTime[i] = 0;
  }
  myLCD.displayText(F("RX ON "));
  SKETCH_PRINTLN(F("Waiting for first Rx message. "));
#endif

  for (int i = 0; i < LAST_ADDRESS + 1; i++) CRC_count[i] = 0; // Clear the counter
}

void loop()
{
#ifdef ETHERNET_ENABLED
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).
  MQTT_connect(&thingspeak, &client_ts);
#ifdef PRINT_ALL_CLIENT_STATUS
  SKETCH_PRINT(F("Ethernet Maintain status: "));
  SKETCH_PRINTLN(Ethernet.maintain());
#endif
  printClientStatus(&client_ts);
#endif

  // Flip the watchdog pin once per loop if we are connected
  // If there is no ethernet, then flip the pin every time through loop
#ifdef ETHERNET_ENABLED
  if (EthDisconnected == 0) {
#endif
    WD_state = !WD_state;
    digitalWrite(WD_PIN, WD_state);
#ifdef ETHERNET_ENABLED
  }
#endif

#ifdef OLED_ENABLED
  if ((displayTimeoutCount == 0) && (digitalRead(PUSH2) == LOW)) {
    displayTimeoutCount++;
  }
  switch (displayTimeoutCount) {
    case 0:     // OLED not enabled, so do nothing
      break;
    case 1:     // First time, so activate display
      oled.command(0x0C);         // Turn on display
      buildStatusString();        // Put the data into display buffer
      oledDisplay();              // Write to the display
      displayTimeoutCount++;
      break;
    case DISPLAY_TIMEOUT:  // If we reach the max, then turn off display
      oled.command(0x08);
      displayTimeoutCount = 0;
      break;
    default:
      buildStatusString();        // Put the data into display buffer
      oledDisplay();              // Write to the display
      displayTimeoutCount++;
      break;
  }
#endif

  int packetSize;

  // Turn on the receiver and listen for incoming data. Timeout after 1 seconds.
  // The receiverOn() method returns the number of bytes copied to rxData.
  // The radio library uses the SPI library internally, this call initializes
  // SPI/CSn and GDO0 lines. Also setup initial address, channel, and TX power.
  SPI.begin();
  Radio.begin(ADDRESS_LOCAL, CHANNEL_1, POWER_MAX);
  packetSize = Radio.receiverOn((unsigned char*)&rxPacket, sizeof(rxPacket), 1000);
  // Simulate a Radio.end() call, but don't want to disable Chip Select pin
  // Radio.end();
  while (Radio.busy()) ; // Empty loop statement
  detachInterrupt(RF_GDO0);
  digitalWrite(CC110L_CS, HIGH);
  pinMode(CC110L_CS, OUTPUT);    // Need to pull radio CS high to keep it off the SPI bus

  if (packetSize > 0) {
    digitalWrite(BOARD_LED, HIGH);
    SKETCH_PRINTLN(F("--"));
    SKETCH_PRINT(F("Received packet from device: "));
    SKETCH_PRINT(rxPacket.from);
    SKETCH_PRINT(F(", bytes: "));
    SKETCH_PRINTLN(packetSize);
    if (Radio.getCrcBit() == 0) {
      crcFailed = 1;
      SKETCH_PRINTLN(F("*** CRC check failed! ***"));
    } else crcFailed = 0;

    lastRssi = Radio.getRssi();
    lastLqi = Radio.getLqi();

    switch (rxPacket.from) {
      case (ADDRESS_WEATHER):
        process_weatherdata();
        break;
      case (ADDRESS_G2):
      case (ADDRESS_SENSOR4):
      case (ADDRESS_SENSOR5):
      case (ADDRESS_SENSOR6):
        process_sensordata();
        break;
      case (ADDRESS_SENSOR7):     // Pond Sensor
        process_ponddata();
        break;
      default:
        SKETCH_PRINTLN(F("Message received from unknown sensor."));
        break;
    }

    SKETCH_PRINT(F("RSSI: "));
    SKETCH_PRINTLN(lastRssi);
    SKETCH_PRINT(F("LQI: "));
    SKETCH_PRINTLN(lastLqi);
    SKETCH_PRINTLN(F("--"));
    digitalWrite(BOARD_LED, LOW);
  }
  else {
    SKETCH_PRINT(F("Nothing received: "));
    SKETCH_PRINTLN(millis());
  }

#ifdef ETHERNET_ENABLED
  // ping the server to keep the mqtt connection alive
  if (! thingspeak.ping()) {
    thingspeak.disconnect();
    SKETCH_PRINTLN(F("ThingSpeak MQTT ping failed, disconnecting."));
    EthDisconnected = 1;
#ifdef LCD_ENABLED
    myLCD.showSymbol(LCD_SEG_RADIO, false);
#endif
  }
  else
    myLCD.showSymbol(LCD_SEG_RADIO, true);
#endif
} // loop()

void process_weatherdata() {
  if (crcFailed) {
    // If CRC failed, increase the counter, but don't send a message.
    CRC_count[rxPacket.from]++;
#ifdef ETHERNET_ENABLED
    process_failedCRC();
#endif
  }
  else {
    temperatures[rxPacket.from] = rxPacket.weatherdata.TMP007_Ti;
    batteries[rxPacket.from] = rxPacket.weatherdata.Batt_mV;
    lastTime[rxPacket.from] = millis();
    SKETCH_PRINTLN(F("Temperature (F): "));
    SKETCH_PRINT(F("    BME280:  "));
    SKETCH_PRINT(rxPacket.weatherdata.BME280_T / 10);
    SKETCH_PRINT(F("."));
    SKETCH_PRINTLN(rxPacket.weatherdata.BME280_T % 10);
    SKETCH_PRINT(F("    TMP106 (Die):  "));
    SKETCH_PRINT(rxPacket.weatherdata.TMP007_Ti / 10);
    SKETCH_PRINT(F("."));
    SKETCH_PRINTLN(rxPacket.weatherdata.TMP007_Ti % 10);
    SKETCH_PRINT(F("    TMP106 (Ext):  "));
    SKETCH_PRINT(rxPacket.weatherdata.TMP007_Te / 10);
    SKETCH_PRINT(F("."));
    SKETCH_PRINTLN(rxPacket.weatherdata.TMP007_Te % 10);
    SKETCH_PRINT(F("    MSP Die: "));
    SKETCH_PRINT(rxPacket.weatherdata.MSP_T / 10);
    SKETCH_PRINT(F("."));
    SKETCH_PRINTLN(rxPacket.weatherdata.MSP_T % 10);
    SKETCH_PRINT(F("Pressure (inHg): "));
    SKETCH_PRINT(rxPacket.weatherdata.BME280_P / 100);
    SKETCH_PRINT(F("."));
    SKETCH_PRINT((rxPacket.weatherdata.BME280_P / 10) % 10);
    SKETCH_PRINTLN(rxPacket.weatherdata.BME280_P % 10);
    SKETCH_PRINT(F("%RH: "));
    SKETCH_PRINT(rxPacket.weatherdata.BME280_H / 10);
    SKETCH_PRINT(F("."));
    SKETCH_PRINTLN(rxPacket.weatherdata.BME280_H % 10);
    SKETCH_PRINT(F("Lux: "));
    SKETCH_PRINTLN(rxPacket.weatherdata.LUX);
    SKETCH_PRINT(F("Battery V: "));
    SKETCH_PRINT(rxPacket.weatherdata.Batt_mV / 1000);
    SKETCH_PRINT(F("."));
    SKETCH_PRINT((rxPacket.weatherdata.Batt_mV / 100) % 10);
    SKETCH_PRINT((rxPacket.weatherdata.Batt_mV / 10) % 10);
    SKETCH_PRINT(rxPacket.weatherdata.Batt_mV % 10);
    SKETCH_PRINTLN(F(" "));
    SKETCH_PRINT(("Loops: "));
    SKETCH_PRINTLN(rxPacket.weatherdata.Loops);
    SKETCH_PRINT(("Millis: "));
    SKETCH_PRINTLN(rxPacket.weatherdata.Millis);

#ifdef LCD_ENABLED
    displayTempOnLCD(rxPacket.weatherdata.TMP007_Ti);
    displayBattOnLCD(rxPacket.weatherdata.Batt_mV);
#endif

#ifdef ETHERNET_ENABLED
    SKETCH_PRINTLN(F("Sending data to ThingSpeak Weather_Channel..."));
    payload[0] = '\0';
    BuildPayload(payload, fieldBuffer, 1, rxPacket.weatherdata.BME280_T);
    BuildPayload(payload, fieldBuffer, 2, rxPacket.weatherdata.TMP007_Te);
    BuildPayload(payload, fieldBuffer, 3, rxPacket.weatherdata.TMP007_Ti);
    BuildPayload(payload, fieldBuffer, 4, rxPacket.weatherdata.MSP_T);
    BuildPayload(payload, fieldBuffer, 5, rxPacket.weatherdata.BME280_H);
    BuildPayload(payload, fieldBuffer, 6, rxPacket.weatherdata.BME280_P);
    BuildPayload(payload, fieldBuffer, 7, rxPacket.weatherdata.LUX);
    BuildPayload(payload, fieldBuffer, 8, rxPacket.weatherdata.Batt_mV);
    BuildPayload(payload, fieldBuffer, 12, "CRC Errors: ");
    snprintf(fieldBuffer, FIELDBUFFERSIZE, "%u", CRC_count[rxPacket.from]);
    strcat(payload, fieldBuffer);
    SKETCH_PRINT(F("Payload: "));
    SKETCH_PRINTLN(payload);
    if (! Weather_Channel.publish(payload)) {
      SKETCH_PRINTLN(F("Weather_Channel Feed Failed to ThingSpeak."));
    }
    SKETCH_PRINTLN(F("Checking RSSI and LQI..."));
    if ((lastRssi < -95) | (lastLqi > 5)) {
      payload[0] = '\0';
      BuildPayload(payload, fieldBuffer, 1, lastRssi);
      BuildPayload(payload, fieldBuffer, 2, lastLqi);
      BuildPayload(payload, fieldBuffer, 3, rxPacket.from);
      BuildPayload(payload, fieldBuffer, 12, "Weak signal from: ");
      snprintf(fieldBuffer, FIELDBUFFERSIZE, "%d", rxPacket.from);
      strcat(payload, fieldBuffer);
      strcat(payload, ", RSSI: ");
      snprintf(fieldBuffer, FIELDBUFFERSIZE, "%d", lastRssi);
      strcat(payload, fieldBuffer);
      strcat(payload, ", LQI: ");
      snprintf(fieldBuffer, FIELDBUFFERSIZE, "%d", lastLqi);
      strcat(payload, fieldBuffer);
      if (! Sensor_Errors.publish(payload)) {
        SKETCH_PRINTLN(F("Failed to send weak signal to ThingSpeak"));
      }
    }
#endif  // #ifdef ETHERNET_ENABLED
  }
} // process_weatherdata()

void process_sensordata() {
  if (crcFailed) {
    // If CRC failed, increase the counter, but don't send a message.
    CRC_count[rxPacket.from]++;
#ifdef ETHERNET_ENABLED
    process_failedCRC();
#endif
  }
  else {
    temperatures[rxPacket.from] = rxPacket.sensordata.MSP_T;
    batteries[rxPacket.from] = rxPacket.sensordata.Batt_mV;
    lastTime[rxPacket.from] = millis();
    SKETCH_PRINT(F("Received packet from temperature sensor: "));
    SKETCH_PRINTLN(rxPacket.from);
    SKETCH_PRINT(F("Temperature (F): "));
    SKETCH_PRINT(rxPacket.sensordata.MSP_T / 10);
    SKETCH_PRINT(F("."));
    SKETCH_PRINTLN(rxPacket.sensordata.MSP_T % 10);
    SKETCH_PRINT(F("Battery V: "));
    SKETCH_PRINT(rxPacket.sensordata.Batt_mV / 1000);
    SKETCH_PRINT(F("."));
    SKETCH_PRINT((rxPacket.sensordata.Batt_mV / 100) % 10);
    SKETCH_PRINT((rxPacket.sensordata.Batt_mV / 10) % 10);
    SKETCH_PRINT(rxPacket.sensordata.Batt_mV % 10);
    SKETCH_PRINTLN(F(" "));
    SKETCH_PRINT(F("Loops: "));
    SKETCH_PRINTLN(rxPacket.sensordata.Loops);
    SKETCH_PRINT(F("Millis: "));
    SKETCH_PRINTLN(rxPacket.sensordata.Millis);
    if (rxPacket.from == ADDRESS_SENSOR6) {
      SKETCH_PRINT(F("Light: "));
      SKETCH_PRINTLN(rxPacket.sensordata.Light_Sensor);
      SKETCH_PRINT(F("Door: "));
      SKETCH_PRINTLN(rxPacket.sensordata.Door_Sensor);
    }

#ifdef ETHERNET_ENABLED
    payload[0] = '\0';
    BuildPayload(payload, fieldBuffer, 1, rxPacket.sensordata.MSP_T);
    BuildPayload(payload, fieldBuffer, 2, rxPacket.sensordata.Batt_mV);
    if (rxPacket.from == ADDRESS_SENSOR6) // Send ethernet card uptime instead of loops for sensor6
      BuildPayload(payload, fieldBuffer, 3, (millis() - lastEthernetReset) / 1000 / 60);
    else
      BuildPayload(payload, fieldBuffer, 3, rxPacket.sensordata.Loops);
    BuildPayload(payload, fieldBuffer, 4, rxPacket.sensordata.Millis);
    BuildPayload(payload, fieldBuffer, 5, lastRssi);
    BuildPayload(payload, fieldBuffer, 6, lastLqi);
    BuildPayload(payload, fieldBuffer, 12, "CRC Errors: ");
    snprintf(fieldBuffer, FIELDBUFFERSIZE, "%u", CRC_count[rxPacket.from]);
    strcat(payload, fieldBuffer);
    switch (rxPacket.from) {
      case ADDRESS_G2:
        SKETCH_PRINTLN(F("Sending data to ThingSpeak..."));
        SKETCH_PRINT(F("Payload: "));
        SKETCH_PRINTLN(payload);
        if (! Temp_Slim.publish(payload)) {
          SKETCH_PRINTLN(F("Temp_Slim Channel Failed to ThingSpeak."));
        }
        break;
      case ADDRESS_SENSOR4:
        SKETCH_PRINTLN(F("Sending data to ThingSpeak..."));
        SKETCH_PRINT(F("Payload: "));
        SKETCH_PRINTLN(payload);
        if (! Temp_Sensor4.publish(payload)) {
          SKETCH_PRINTLN(F("Temp_Sensor4 Channel Failed to ThingSpeak."));
        }
        break;
      case ADDRESS_SENSOR5:
        SKETCH_PRINTLN(F("Sending data to ThingSpeak..."));
        SKETCH_PRINT(F("Payload: "));
        SKETCH_PRINTLN(payload);
        if (! Temp_Sensor5.publish(payload)) {
          SKETCH_PRINTLN(F("Temp_Sensor5 Channel Failed to ThingSpeak."));
        }
        break;
      case ADDRESS_SENSOR6:
        BuildPayload(payload, fieldBuffer, 7, millis() / 1000 / 60);             // Send RX hub uptime in minutes
        BuildPayload(payload, fieldBuffer, 8, rxPacket.sensordata.Door_Sensor);
        SKETCH_PRINTLN(F("Sending data to ThingSpeak..."));
        SKETCH_PRINT(F("Payload: "));
        SKETCH_PRINTLN(payload);
        if (! Temp_Sensor6.publish(payload)) {
          SKETCH_PRINTLN(F("Temp_Sensor6 Channel Failed to ThingSpeak."));
        }
        break;
      default:
        break;
    }
#endif // #ifdef ETHERNET_ENABLED
  }
} // process_sensordata()

void process_ponddata() {
  if (crcFailed) {
    // If CRC failed, increase the counter, but don't send a message.
    CRC_count[rxPacket.from]++;
#ifdef ETHERNET_ENABLED
    process_failedCRC();
#endif
  }
  else {
    temperatures[rxPacket.from] = rxPacket.ponddata.SUBMERGED_T;
    batteries[rxPacket.from] = rxPacket.weatherdata.Batt_mV;
    lastTime[rxPacket.from] = millis();
    SKETCH_PRINTLN(F("Received packet from pond sensor. "));
    SKETCH_PRINT(F("Temperature (F): "));
    SKETCH_PRINT(rxPacket.ponddata.SUBMERGED_T / 10);
    SKETCH_PRINT(F("."));
    SKETCH_PRINTLN(rxPacket.ponddata.SUBMERGED_T % 10);
    SKETCH_PRINT(F("Battery V: "));
    SKETCH_PRINT(rxPacket.ponddata.Batt_mV / 1000);
    SKETCH_PRINT(F("."));
    SKETCH_PRINT((rxPacket.ponddata.Batt_mV / 100) % 10);
    SKETCH_PRINT((rxPacket.ponddata.Batt_mV / 10) % 10);
    SKETCH_PRINT(rxPacket.ponddata.Batt_mV % 10);
    SKETCH_PRINTLN(F(" "));
    SKETCH_PRINT(F("Millis: "));
    SKETCH_PRINTLN(rxPacket.ponddata.Millis);
#ifdef ETHERNET_ENABLED
    payload[0] = '\0';
    BuildPayload(payload, fieldBuffer, 1, rxPacket.ponddata.MSP_T);
    BuildPayload(payload, fieldBuffer, 2, rxPacket.ponddata.SUBMERGED_T);
    BuildPayload(payload, fieldBuffer, 3, rxPacket.ponddata.Batt_mV);
    BuildPayload(payload, fieldBuffer, 4, rxPacket.ponddata.Millis);
    BuildPayload(payload, fieldBuffer, 5, rxPacket.ponddata.PUMP_STATUS);
    BuildPayload(payload, fieldBuffer, 6, rxPacket.ponddata.AERATOR_STATUS);
    BuildPayload(payload, fieldBuffer, 7, rxPacket.ponddata.Battery_T);
    BuildPayload(payload, fieldBuffer, 12, "CRC Errors: ");
    snprintf(fieldBuffer, FIELDBUFFERSIZE, "%u", CRC_count[rxPacket.from]);
    strcat(payload, fieldBuffer);
    SKETCH_PRINTLN(F("Sending data to ThingSpeak..."));
    SKETCH_PRINT(F("Payload: "));
    SKETCH_PRINTLN(payload);
    if (! Pond_Sensor.publish(payload)) {
      SKETCH_PRINTLN(F("Pond_Sensor Channel Failed to ThingSpeak."));
    }
#endif // #ifdef ETHERNET_ENABLED
  }
} // process_ponddata()


// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care of connecting.
#ifdef ETHERNET_ENABLED
void MQTT_connect(Adafruit_MQTT_Client * mqtt_server, EthernetClient * client ) {
  int8_t ret;

  // Return if already connected.
  if (mqtt_server->connected()) {
    EthDisconnected = 0;
#ifdef LCD_ENABLED
    myLCD.showSymbol(LCD_SEG_RADIO, true);
#endif
    return;
  }

  SKETCH_PRINTLN(F("MQTT Disconnected."));
  EthDisconnected = 1;
#ifdef LCD_ENABLED
  myLCD.showSymbol(LCD_SEG_RADIO, false);
#endif
  printClientStatus(client);
  SKETCH_PRINT(F("Attempting reconnect to MQTT: "));
  SKETCH_PRINTLN(millis());
  ret = mqtt_server->connect();
  printClientStatus(client);
  // W5200 chip seems to have a problem of getting stuck in CLOSE_WAIT state
  // There may be other issues, so just force a hard reset on ethernet
  // shield if we lose connection
  if (ret != 0) {
    lostConnectionCount++;
    SKETCH_PRINT(F("Ethernet connection lost #: "));
    SKETCH_PRINTLN(lostConnectionCount);
    if (lostConnectionCount > 5) {

      SKETCH_PRINT(F("Ethernet connection loss over max: "));
      SKETCH_PRINTLN(lostConnectionCount);
      lostConnectionCount = 0;
#ifdef OLED_ENABLED
      // Turn off the OLED while waiting to reconnect
      oled.command(0x08);
      displayTimeoutCount = 0;
#endif
      client_ts.stop();
      digitalWrite(W5200_RESET, LOW);
      delay(5);
      digitalWrite(W5200_RESET, HIGH);
      lastEthernetReset = millis();
      delay(200); // Need to delay at least 150 ms after reset
      SKETCH_PRINTLN(F("Starting Ethernet..."));
      Ethernet.begin(mac);
      SKETCH_PRINTLN(F("Ethernet enabled."));
    }
  }
  SKETCH_PRINTLN(mqtt_server->connectErrorString(ret));
  if (ret == 0) {
    EthDisconnected = 0;
#ifdef LCD_ENABLED
    myLCD.showSymbol(LCD_SEG_RADIO, true);
#endif
  }
}
#endif

#ifdef ETHERNET_ENABLED
int printClientStatus(EthernetClient * client) {
  int ClientStatus;
  ClientStatus = client->status();
  if (ClientStatus != 0x17) {
    SKETCH_PRINT(F("Ethernet Client Status: "));
    SKETCH_PRINT(F(" - "));
  }
  switch (ClientStatus) {
    case 0:
      SKETCH_PRINTLN(F("CLOSED"));
      break;
    case 0x13:
      SKETCH_PRINTLN(F("INIT"));
      break;
    case 0x14:
      SKETCH_PRINTLN(F("LISTEN"));
      break;
    case 0x15:
      SKETCH_PRINTLN(F("SYNSENT"));
      break;
    case 0x16:
      SKETCH_PRINTLN(F("SYNRECV"));
      break;
    case 0x17:
#ifdef PRINT_ALL_CLIENT_STATUS
      SKETCH_PRINT(F("Ethernet Client Status: "));
      SKETCH_PRINT(F(" - "));
      SKETCH_PRINTLN(F("ESTABLISHED"));
#endif
      break;
    case 0x18:
      SKETCH_PRINTLN(F("FIN_WAIT"));
      break;
    case 0x1a:
      SKETCH_PRINTLN(F("CLOSING"));
      break;
    case 0x1b:
      SKETCH_PRINTLN(F("TIME_WAIT"));
      break;
    case 0x1c:
      SKETCH_PRINTLN(F("CLOSE_WAIT"));
      break;
    case 0x1d:
      SKETCH_PRINTLN(F("LAST_ACK"));
      break;
    case 0x22:
      SKETCH_PRINTLN(F("UTP"));
      break;
    case 0x32:
      SKETCH_PRINTLN(F("IPRAW"));
      break;
    case 0x42:
      SKETCH_PRINTLN(F("MACRAW"));
      break;
    case 0x5f:
      SKETCH_PRINTLN(F("PPPOE"));
      break;
    default:
      SKETCH_PRINT(F("Unknown State: "));
      SKETCH_PRINTLN(ClientStatus, 16);
      break;
  }
  return ClientStatus;
}
#endif

#ifdef OLED_ENABLED
void oledDisplay() {
  byte r = 0;
  byte c = 0;

  oled.command(0x01); // Clear display and cursor home
  delay(2);           // Need a pause after clearing display
  for (r = 0; r < OLED_ROWS; r++)        // One row at a time
  {
    oled.command(row_address[r]);        //  moves the cursor to the first column of that line
    for (c = 0; c < OLED_COLS; c++)      //  One character at a time
    {
      oled.data(oled_text[r][c]);         //  displays the correspondig string
    }
  }
}

void buildStatusString() {
  int splen;
  unsigned long timeSince;

  splen = snprintf((char*)oled_text[0], OLED_COLS + 1, "%d: T=%3d, V=%d.%02d",
                   displayTimeoutCount, temperatures[displayTimeoutCount],
                   batteries[displayTimeoutCount] / 1000, ((batteries[displayTimeoutCount] % 1000) + 5) / 10 );
  // Pad the rest of the string with spaces.
  for (int i = splen; i < OLED_COLS; i++) {
    oled_text[0][i] = ' ';
  }
  oled_text[0][OLED_COLS] = '\0';

  timeSince = (millis() - lastTime[displayTimeoutCount]) / 1000 / 60;
  if (timeSince > 9999) timeSince = 9999;
  // Print # minutes since last message received
  splen = snprintf((char*)oled_text[1], OLED_COLS + 1, "Last: %4d mins", timeSince);
  // Pad the rest of the string with spaces.
  for (int i = splen; i < OLED_COLS; i++) {
    oled_text[1][i] = ' ';
  }
  oled_text[1][OLED_COLS] = '\0';
}
#endif

#ifdef ETHERNET_ENABLED
void process_failedCRC() {
  payload[0] = '\0';
  BuildPayload(payload, fieldBuffer, 1, lastRssi);
  BuildPayload(payload, fieldBuffer, 2, lastLqi);
  BuildPayload(payload, fieldBuffer, 3, rxPacket.from);
  BuildPayload(payload, fieldBuffer, 12, "CRC failed from: ");
  snprintf(fieldBuffer, FIELDBUFFERSIZE, "%d", rxPacket.from);
  strcat(payload, fieldBuffer);
  strcat(payload, ", RSSI: ");
  snprintf(fieldBuffer, FIELDBUFFERSIZE, "%d", lastRssi);
  strcat(payload, fieldBuffer);
  strcat(payload, ", LQI: ");
  snprintf(fieldBuffer, FIELDBUFFERSIZE, "%d", lastLqi);
  strcat(payload, fieldBuffer);
  if (! Sensor_Errors.publish(payload)) {
    SKETCH_PRINTLN(F("Failed to send bad CRC to ThingSpeak"));
  }
}
#endif

#ifdef ETHERNET_ENABLED
/*******************************************************************
  // BuildPayload() functions for ThingSpeak MQTT
  // See https://www.mathworks.com/help/thingspeak/publishtoachannelfeed.html
  // Overloaded function formats data field based on parameter type.
  // Be sure to set msgBuffer[0] = '\0' to start a new payload string
  // Use fieldNum==12 to format the Status field
*******************************************************************/

// This is the "worker" version that is called by all other versions of the function
// It is also used if a string is already available and does not need to be converted
void BuildPayload(char* msgBuffer, int fieldNum, char* dataField) {
  char  numBuffer[4];
  numBuffer[0] = '\0';

  if (fieldNum < 9) {
    if (msgBuffer[0] == '\0')
      strcat(msgBuffer, "field");
    else
      strcat(msgBuffer, "&field");
    snprintf(numBuffer, 4, "%d", fieldNum);
    strcat(msgBuffer, numBuffer);
    strcat(msgBuffer, "=");
    strcat(msgBuffer, dataField);
  }
  else { // fieldNum >= 9
    if (msgBuffer[0] == '\0')
      strcat(msgBuffer, "status=");
    else
      strcat(msgBuffer, "&status=");
    strcat(msgBuffer, dataField);
  }
  // Note that ThingSpeak defines several other Payload Parameters beyond
  // field1-8. I have only implemented the "status" field, which is the 12th
  // paramter type in the API docs, hence, use "12" for status, even though
  // anything > 8 would work as is currently coded.
}

void BuildPayload(char* msgBuffer, char* dataFieldBuffer, int fieldNum, int data) {
  snprintf(dataFieldBuffer, FIELDBUFFERSIZE, "%d", data);
  BuildPayload(msgBuffer, fieldNum, dataFieldBuffer);
}

void BuildPayload(char* msgBuffer, char* dataFieldBuffer, int fieldNum, unsigned int data) {
  snprintf(dataFieldBuffer, FIELDBUFFERSIZE, "%u", data);
  BuildPayload(msgBuffer, fieldNum, dataFieldBuffer);
}

void BuildPayload(char* msgBuffer, char* dataFieldBuffer, int fieldNum, unsigned long data) {
  snprintf(dataFieldBuffer, FIELDBUFFERSIZE, "%lu", data);
  BuildPayload(msgBuffer, fieldNum, dataFieldBuffer);
}

void BuildPayload(char* msgBuffer, char* dataFieldBuffer, int fieldNum, const char* data) {
  snprintf(dataFieldBuffer, FIELDBUFFERSIZE, data);
  BuildPayload(msgBuffer, fieldNum, dataFieldBuffer);
}
#endif

#ifdef LCD_ENABLED
void displayTempOnLCD(int T) {
#define DISPLAYSIZE 6
  char tempChar[DISPLAYSIZE + 1];
  int tempLen;
  int tempSign;
  int Tprime;

  if (T < 0) {
    tempSign = 1;
    T = -T;
  } else tempSign = 0;

  Tprime = (T + 5) / 10;  // Add 5 to round up, then truncate the 10ths
  if (Tprime == 0) tempSign = 0; // Correct for negative tenth degrees after rounding

  if (T > 999) T = 999; // Keep it to 3 digits in case we get an erroneous reading

  tempChar[6] = '\0';
  tempChar[5] = ' ';
  if (Tprime > 0) tempChar[4] = Tprime % 10 + '0';     // Ones degrees
  else tempChar[4] = '0';
  Tprime = Tprime / 10;
  if (Tprime > 0) tempChar[3] = Tprime % 10 + '0';     // Tens degrees
  else tempChar[3] = ' ';
  Tprime = Tprime / 10;                                // Hundreds degrees
  if (Tprime > 0) tempChar[2] = Tprime % 10 + '0';
  else tempChar[2] = ' ';
  tempChar[1] = 'T';
  tempChar[0] = ' ';

  myLCD.clear();
  myLCD.displayText(tempChar);
  myLCD.showSymbol(LCD_SEG_COLON2, 1);
  myLCD.showSymbol(LCD_SEG_DEG5, 1);
  myLCD.showSymbol(LCD_SEG_MINUS1, tempSign);
  myLCD.showSymbol(LCD_SEG_RADIO, !EthDisconnected);
}

// MSP430G2 requires minimum 2.2V
void displayBattOnLCD(int mV) {
  if (mV > 2999) myLCD.showSymbol(LCD_SEG_BAT5, 1);
  if (mV > 2800) myLCD.showSymbol(LCD_SEG_BAT4, 1);
  if (mV > 2600) myLCD.showSymbol(LCD_SEG_BAT3, 1);
  if (mV > 2500) myLCD.showSymbol(LCD_SEG_BAT2, 1);
  if (mV > 2400) myLCD.showSymbol(LCD_SEG_BAT1, 1);
  if (mV > 2300) myLCD.showSymbol(LCD_SEG_BAT0, 1);
  myLCD.showSymbol(LCD_SEG_BAT_ENDS, 1);
  myLCD.showSymbol(LCD_SEG_BAT_POL, 1);
}

#endif
