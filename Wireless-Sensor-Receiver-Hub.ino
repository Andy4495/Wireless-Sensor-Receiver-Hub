/**
   Sensor Receiver Station
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
   - Simple temp monitor using MSP430G2553 internal sensor
   - Additional MSP430 temp sensors using same data structure
   - Other transmitters may be defined in the future

   The receiving station code needs to be modified whenever
   a new sensor is added:
   - Sensor CC110L device address
   - Data payload structure
   - Output stream (Serial, LCD, and/or IP)

   The sketch toggles Pin 12 every time through loop() so
   long as the mqtt connection is valid. An external
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
   - NewhavenOLED: https://gitlab.com/Andy4495/NewhavenOLED
       - Only used if OLED_ENABLED is #defined
*/

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
*/
//#define OLED_ENABLED
#define ETHERNET_ENABLED
//#define PRINT_ALL_CLIENT_STATUS

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

/* CONTROL PIN DEFINITIONS
   -----------------------

    W5200_RESET - Connected to RESET on W5200 chip. Active LOW.
    W5200_CS    - Connected to SCS pin on W5200 chip. Active LOW.
    CC110L_CS   - Connected to CS pin on CC110L chip. Active LOW.

    NOTE: The CS pins are also defined separately in the associated driver
         libraries. Both the driver libraries and the following
         definitions need to be updated in order to change the pin
         assignments
*/

#define W5200_RESET 5
#define W5200_CS    8
#define CC110L_CS  18
#define WD_PIN     12             // Toggle for external watchdog

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

#include <SPI.h>
#include <AIR430BoostFCC.h>

#ifdef LCD_ENABLED
#include "LCD_Launchpad.h"
LCD_LAUNCHPAD myLCD;
#endif

#ifdef OLED_ENABLED
#include <NewhavenOLED.h>
const byte row_address[2] = {0x80, 0xC0};   // DDRAM addresses for rows (2-row models)
NewhavenOLED oled(OLED_ROWS, OLED_COLS, OLED_SI, OLED_SCK, OLED_CS, NO_PIN);
byte oled_text[OLED_ROWS][OLED_COLS + 1] =
{ "Sensor Rx Hub   ",
  "    Initializing"
};
#define WEATHER_ROW 0
#define G2_ROW 1
#define DISPLAY_TIMEOUT 7
int displayTimeoutCount = DISPLAY_TIMEOUT;
#endif

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
EthernetClient client_cayenne;
EthernetClient client_ts;
Adafruit_MQTT_Client cayenne(&client_cayenne, CAY_SERVER, CAY_SERVERPORT, CAY_CLIENTID, CAY_USERNAME, CAY_PASSWORD);
Adafruit_MQTT_Client thingspeak(&client_ts, TS_SERVER, TS_SERVERPORT, TS_USERNAME, TS_KEY);
char payload[110];    // MQTT payload string
char fieldBuffer[20];  // Temporary buffer to construct a single field of payload string
#endif

// -----------------------------------------------------------------------------


#define ADDRESS_LOCAL    0x01
#define ADDRESS_WEATHER  0x02
#define ADDRESS_G2       0x03
#define ADDRESS_SENSOR4  0x04
#define ADDRESS_SENSOR5  0x05
#define ADDRESS_SENSOR6  0x06
#define LAST_ADDRESS     0x06

#define RF_GDO0       19

struct WeatherData {
  int             BME280_T;  // Tenth degrees F
  unsigned int    BME280_P;  // Pressure in inches of Hg * 100
  int             BME280_H;  // % Relative Humidity
  int             TMP107_Ti; // Tenth degrees F
  int             TMP107_Te; // Tenth degrees F
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

// Check "struct_type" member for the type of structure to
// decode in the sPacket union.
enum {WEATHER_STRUCT, TEMP_STRUCT};

struct sPacket  // CC110L packet structure
{
  uint8_t from;              // Local node address that message originated from
  uint8_t struct_type;       // Filler byte to keep rest of struct on word boundary
  // Also used to indicate type of struct in union
  union {
    uint8_t message[58];     // Local node message keep even word boundary
    WeatherData weatherdata;
    TempSensor  sensordata;
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
#ifdef OLED_ENABLED
unsigned long lastG2Millis, lastWeatherMillis;
#endif

#ifdef LCD_ENABLED
int currentDisplay; //Remember which temp value is on LCD
int temperatures[LAST_ADDRESS - 1]; // Store last temp value received
int batteries[LAST_ADDRESS - 1];  // Store last battery reading
// The following values store the state of the status symbols so they
// can be restored after an LCD.clear() operation:
int  RadioStatus = 0;
int  TxStatus = 0;
#endif
int  MarkStatus = 0; // Need this state regardless of LCD, so put outside ifdef

#ifdef ETHERNET_ENABLED
/***** MQTT publishing feeds *****
   Each feed/channel that you wish to publish needs to be defined.
     - Adafruit IO feeds follow the form: <username>/feeds/<feedname>, for example:
         Adafruit_MQTT_Publish pressure = Adafruit_MQTT_Publish(&mqtt,  AIO_USERNAME "/feeds/pressure");
     - ThingSpeak Channels follow the form: channels/<CHANNEL_ID>/publish/<WRITE_API_KEY>, for example:
         Adafruit_MQTT_Publish myChannel = Adafruit_MQTT_Publish(&mqtt,
                                        "channels/" CHANNEL_ID "/publish/" CHANNEL_WRITE_API_KEY);
         See https://www.mathworks.com/help/thingspeak/publishtoachannelfeed.html
     - Cayenne Channel format:
     Adafruit_MQTT_Publish Topic = Adafruit_MQTT_Publish(&cayenne,
                                "v1/" CAY_USERNAME "/things/" CAY_CLIENT_ID "/data/" CAY_CHANNEL_ID);
       See https://mydevices.com/cayenne/docs/cayenne-mqtt-api/#cayenne-mqtt-api-mqtt-messaging-topics
   The file "MQTT_private_feeds.h" needs to include the feed/channel definitions
   specific to your configuration.
*/
#include "MQTT_private_feeds.h"
#endif

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
  delay(200);         // RESET needs to be cleared for at least 150 ms before operating

  // Set up the watchdog pin
  pinMode(WD_PIN, OUTPUT);
  digitalWrite(WD_PIN, WD_state);

  // Setup serial for status printing.
  Serial.begin(9600);
  Serial.println(F(" "));
  Serial.println(F("Sensor receiver hub with CC110L."));
  delay(500);

#if defined(__MSP430FR6989__)
  // Lock down FRAM - disable writes to program memory
  Serial.println("Writing FRAM control registers.");
  unsigned int* MPUCTL0_reg = (unsigned int*) 0x05a0;
  unsigned int* MPUSAM_reg  = (unsigned int*)0x05a8;
  *MPUCTL0_reg = (unsigned int) 0xa501;
  *MPUSAM_reg  = (unsigned int) 0x7555;
  Serial.print("MPUCTL0: ");
  Serial.println((unsigned int) * (int*)MPUCTL0_reg, HEX);
  Serial.print("MPUSAM: ");
  Serial.println((unsigned int) * (int*)MPUSAM_reg, HEX);
#endif

  MarkStatus = 1;
#ifdef LCD_ENABLED
  myLCD.init();
#ifdef ETHERNET_ENABLED
  myLCD.displayText(F("ETHER "));
  // Display the "!" LCD symbol to show we are initializing
  myLCD.showSymbol(LCD_SEG_MARK, MarkStatus);
#endif
  Serial.println(F("Started LCD display"));
#endif

#ifdef OLED_ENABLED
  oled.begin();
  oledDisplay();
  // Clear out the buffer
  memset(oled_text, ' ', sizeof(oled_text));
#endif

#ifdef ETHERNET_ENABLED
  Serial.println(F("Starting Ethernet..."));
  Ethernet.begin(mac);
  Serial.println(F("Ethernet enabled."));
  Serial.println("Attempting to connect to Cayenne...");
  MQTT_connect(&cayenne, &client_cayenne);
  Serial.println("Attempting to connect to Thingspeak...");
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

  pinMode(PUSH1, INPUT_PULLUP);     // PUSH1 cycles LCD display
  pinMode(PUSH2, INPUT_PULLUP);     // PUSH2 temporarily turns on OLED

#ifdef OLED_ENABLED
  lastG2Millis = millis();
  lastWeatherMillis = lastG2Millis;
#endif

#ifdef LCD_ENABLED
  for (int i = 0; i < (LAST_ADDRESS - 2); i++) {
    temperatures[i] = 0;
    batteries[i] = 0;
  }
  myLCD.displayText(F("RX ON "));
  RadioStatus = 1;
  myLCD.showSymbol(LCD_SEG_RADIO, RadioStatus);
  Serial.println(F("Waiting for first Rx message. "));
#endif
}

void loop()
{
#ifdef ETHERNET_ENABLED
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).
  MQTT_connect(&cayenne, &client_cayenne);
  MQTT_connect(&thingspeak, &client_ts);
#ifdef PRINT_ALL_CLIENT_STATUS
  Serial.print(F("Ethernet Maintain status: "));
  Serial.println(Ethernet.maintain());
#endif
  printClientStatus(&client_cayenne);
  printClientStatus(&client_ts);
#endif

  // Flip the watchdog pin once per loop if we are connected
  // If there is no ethernet, then flip the pin every time through loop
#ifdef ETHERNET_ENABLED
  if (MarkStatus == 0) {
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
      displayTimeoutCount++;
  }

#endif

  int packetSize;

  // Turn on the receiver and listen for incoming data. Timeout after 1 seconds.
  // The receiverOn() method returns the number of bytes copied to rxData.
  // The radio library uses the SPI library internally, this call initializes
  // SPI/CSn and GDO0 lines. Also setup initial address, channel, and TX power.
  SPI.begin(CC110L_CS);
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
#ifdef LCD_ENABLED
    myLCD.showSymbol(LCD_SEG_RX, 1);
#endif
    Serial.println(F("--"));
    Serial.print(F("Received packet from device: "));
    Serial.print(rxPacket.from);
    Serial.print(F(", bytes: "));
    Serial.println(packetSize);
    if (Radio.getCrcBit() == 0) {
      crcFailed = 1;
      Serial.println(F("*** CRC check failed! ***"));
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
      default:
        Serial.println(F("Message received from unknown sensor."));
        break;
    }

    Serial.print(F("RSSI: "));
    Serial.println(lastRssi);
    Serial.print(F("LQI: "));
    Serial.println(lastLqi);
    Serial.println(F("--"));
    digitalWrite(BOARD_LED, LOW);
#ifdef LCD_ENABLED
    myLCD.showSymbol(LCD_SEG_RX, 0);
#endif
  }
  else {
    Serial.print(F("Nothing received: "));
    Serial.println(millis());
  }

#ifdef LCD_ENABLED
  if (digitalRead(PUSH1) == 0) {
    myLCD.clear();
    currentDisplay++;
    if (currentDisplay > LAST_ADDRESS) currentDisplay = 2;
    displayTempOnLCD(temperatures[currentDisplay - 2]);
    displayBattOnLCD(batteries[currentDisplay - 2]);
    switch (currentDisplay) {
      case ADDRESS_WEATHER:       // 0x02
        myLCD.showSymbol(LCD_SEG_CLOCK, 1);
        break;
      case ADDRESS_G2:            // 0x03
        myLCD.showSymbol(LCD_SEG_CLOCK, 1);
        myLCD.showSymbol(LCD_SEG_R, 1);
        break;
      case ADDRESS_SENSOR4:       // 0x04
        myLCD.showSymbol(LCD_SEG_HEART, 1);
        break;
      case ADDRESS_SENSOR5:       // 0x05
        myLCD.showSymbol(LCD_SEG_HEART, 1);
        myLCD.showSymbol(LCD_SEG_R, 1);
        break;
      case ADDRESS_SENSOR6:       // 0x06
        myLCD.showSymbol(LCD_SEG_CLOCK, 1);
        myLCD.showSymbol(LCD_SEG_HEART, 1);
        break;
      default:
        break;
    }
#ifdef ETHERNET_ENABLED
    payload[0] = '\0';
    BuildPayload(payload, fieldBuffer, 12, "PUSH1 Pressed.");
    if (! Sensor_Errors.publish(payload)) {
      Serial.println(F("Failed to send PUSH1 to ThingSpeak"));
    }
#endif
  }
#endif

#ifdef ETHERNET_ENABLED
  // ping the server to keep the mqtt connection alive
  if (! cayenne.ping()) {
    cayenne.disconnect();
    Serial.println(F("Cayenne MQTT ping failed, disconnecting."));
    MarkStatus = 1;
#ifdef LCD_ENABLED
    myLCD.showSymbol(LCD_SEG_MARK, MarkStatus);
#endif
  }
  if (! thingspeak.ping()) {
    thingspeak.disconnect();
    Serial.println(F("ThingSpeak MQTT ping failed, disconnecting."));
    MarkStatus = 1;
#ifdef LCD_ENABLED
    myLCD.showSymbol(LCD_SEG_MARK, MarkStatus);
#endif
  }
#endif
}

void process_weatherdata() {
  if (crcFailed) {
#ifdef ETHERNET_ENABLED
    // If CRC was bad, then send status message to ThingSpeak, along with RSSI and LQI
    process_failedCRC();
    payload[0] = '\0';
    BuildPayload(payload, fieldBuffer, 12, "CRC Failed!");
    if (! Weather_Channel.publish(payload)) {
      Serial.println(F("Failed to send bad CRC to Weather_Channel ThingSpeak"));
    }
#endif
  }
  else {
    Serial.println(F("Temperature (F): "));
    Serial.print(F("    BME280:  "));
    Serial.print(rxPacket.weatherdata.BME280_T / 10);
    Serial.print(F("."));
    Serial.println(rxPacket.weatherdata.BME280_T % 10);
    Serial.print(F("    TMP106 (Die):  "));
    Serial.print(rxPacket.weatherdata.TMP107_Ti / 10);
    Serial.print(F("."));
    Serial.println(rxPacket.weatherdata.TMP107_Ti % 10);
    Serial.print(F("    TMP106 (Ext):  "));
    Serial.print(rxPacket.weatherdata.TMP107_Te / 10);
    Serial.print(F("."));
    Serial.println(rxPacket.weatherdata.TMP107_Te % 10);
    Serial.print(F("    MSP Die: "));
    Serial.print(rxPacket.weatherdata.MSP_T / 10);
    Serial.print(F("."));
    Serial.println(rxPacket.weatherdata.MSP_T % 10);
    Serial.print(F("Pressure (inHg): "));
    Serial.print(rxPacket.weatherdata.BME280_P / 100);
    Serial.print(F("."));
    Serial.print((rxPacket.weatherdata.BME280_P / 10) % 10);
    Serial.println(rxPacket.weatherdata.BME280_P % 10);
    Serial.print(F("%RH: "));
    Serial.print(rxPacket.weatherdata.BME280_H / 10);
    Serial.print(F("."));
    Serial.println(rxPacket.weatherdata.BME280_H % 10);
    Serial.print(F("Lux: "));
    Serial.println(rxPacket.weatherdata.LUX);
    Serial.print(F("Battery V: "));
    Serial.print(rxPacket.weatherdata.Batt_mV / 1000);
    Serial.print(F("."));
    Serial.print((rxPacket.weatherdata.Batt_mV / 100) % 10);
    Serial.print((rxPacket.weatherdata.Batt_mV / 10) % 10);
    Serial.print(rxPacket.weatherdata.Batt_mV % 10);
    if (rxPacket.weatherdata.Batt_mV < 2400) {
      Serial.print(F("   *** Out of Spec ***"));
    }
    Serial.println(F(" "));
    Serial.print(("Loops: "));
    Serial.println(rxPacket.weatherdata.Loops);
    Serial.print(("Millis: "));
    Serial.println(rxPacket.weatherdata.Millis);

#ifdef LCD_ENABLED
    displayTempOnLCD(rxPacket.weatherdata.BME280_T);
    myLCD.showSymbol(LCD_SEG_CLOCK, 1);
    displayBattOnLCD(rxPacket.weatherdata.Batt_mV);
    currentDisplay = ADDRESS_WEATHER;
    temperatures[ADDRESS_WEATHER - 2] = rxPacket.weatherdata.BME280_T;
    batteries[ADDRESS_WEATHER - 2]    = rxPacket.weatherdata.Batt_mV;
#endif

#ifdef ETHERNET_ENABLED
#ifdef LCD_ENABLED
    myLCD.showSymbol(LCD_SEG_TX, 1);
#endif
    Serial.println(F("Sending data to Cayenne..."));
    payload[0] = '\0';
    sprintf(payload, "temp,f=%d.%d", rxPacket.weatherdata.TMP107_Ti/10, rxPacket.weatherdata.TMP107_Ti%10);
    if (! Weather_TMP007I.publish(payload)) {
      Serial.println(F("TMP107_Ti Failed"));
    }
    payload[0] = '\0';
    sprintf(payload, "bp,hpa=%d", rxPacket.weatherdata.BME280_P);
    if (! Weather_BME280P.publish(payload)) {
      Serial.println(F("BME280_P Failed"));
    }
    payload[0] = '\0';
    sprintf(payload, "rel_hum,p=%d.%d", rxPacket.weatherdata.BME280_H/10, rxPacket.weatherdata.BME280_H%10);
    if (! Weather_BME280H.publish(payload)) {
      Serial.println(F("RH Failed"));
    }
    payload[0] = '\0';
    sprintf(payload, "lum,lux=%d", rxPacket.weatherdata.LUX);
    if (! Weather_LUX.publish(payload)) {
      Serial.println(F("LUX Failed"));
    }
    payload[0] = '\0';
    sprintf(payload, "voltage,mv=%d", rxPacket.weatherdata.Batt_mV);
    if (! Weather_BATT.publish(payload)) {
      Serial.println(F("Batt Failed"));
    }

    Serial.println(F("Sending data to ThingSpeak Weather_Channel..."));
    payload[0] = '\0';
    BuildPayload(payload, fieldBuffer, 1, rxPacket.weatherdata.BME280_T);
    BuildPayload(payload, fieldBuffer, 2, rxPacket.weatherdata.TMP107_Te);
    BuildPayload(payload, fieldBuffer, 3, rxPacket.weatherdata.TMP107_Ti);
    BuildPayload(payload, fieldBuffer, 4, rxPacket.weatherdata.MSP_T);
    BuildPayload(payload, fieldBuffer, 5, rxPacket.weatherdata.BME280_H);
    BuildPayload(payload, fieldBuffer, 6, rxPacket.weatherdata.BME280_P);
    BuildPayload(payload, fieldBuffer, 7, rxPacket.weatherdata.LUX);
    BuildPayload(payload, fieldBuffer, 8, rxPacket.weatherdata.Batt_mV);
    Serial.print(F("Payload: "));
    Serial.println(payload);
    if (! Weather_Channel.publish(payload)) {
      Serial.println(F("Weather_Channel Feed Failed to ThingSpeak."));
    }
    Serial.println(F("Checking RSSI and LQI..."));
    if ((lastRssi < -95) | (lastLqi > 5)) {
      payload[0] = '\0';
      BuildPayload(payload, fieldBuffer, 12, "Weak signal from: ");
      sprintf(fieldBuffer, "%d", rxPacket.from);
      strcat(payload, fieldBuffer);
      strcat(payload, ", RSSI: ");
      sprintf(fieldBuffer, "%d", lastRssi);
      strcat(payload, fieldBuffer);
      strcat(payload, ", LQI: ");
      sprintf(fieldBuffer, "%d", lastLqi);
      strcat(payload, fieldBuffer);
      if (! Weather_Channel.publish(payload)) {
        Serial.println(F("Failed to send weak signal message to ThingSpeak"));
      }
    }

#ifdef LCD_ENABLED
    myLCD.showSymbol(LCD_SEG_TX, 0);
#endif
#endif
#ifdef OLED_ENABLED
    lastWeatherMillis = millis();
#endif
  }
}

void process_sensordata() {
  if (crcFailed) {
#ifdef ETHERNET_ENABLED
    // If CRC was bad, then send status message to ThingSpeak, along with RSSI and LQI
    process_failedCRC();
    payload[0] = '\0';
    BuildPayload(payload, fieldBuffer, 12, "CRC Failed!");
    switch (rxPacket.from) {
      case ADDRESS_G2:
        if (! Temp_Slim.publish(payload)) {
          Serial.println(F("Failed to send bad CRC to G2 ThingSpeak"));
        }
        break;
      case ADDRESS_SENSOR4:
        if (! Temp_Sensor4.publish(payload)) {
          Serial.println(F("Failed to send bad CRC to  ThingSpeak"));
        }
        break;
      case ADDRESS_SENSOR5:
        if (! Temp_Sensor5.publish(payload)) {
          Serial.println(F("Failed to send bad CRC to ThingSpeak"));
        }
        break;
      case ADDRESS_SENSOR6:
        if (! Temp_Sensor6.publish(payload)) {
          Serial.println(F("Failed to send bad CRC to ThingSpeak"));
        }
        break;
      default:
        break;
    }
#endif
  }
  else {
    Serial.print(F("Received packet from temperature sensor: "));
    Serial.println(rxPacket.from);
    Serial.print(F("Temperature (F): "));
    Serial.print(rxPacket.sensordata.MSP_T / 10);
    Serial.print(F("."));
    Serial.println(rxPacket.sensordata.MSP_T % 10);
    Serial.print(F("Battery V: "));
    Serial.print(rxPacket.sensordata.Batt_mV / 1000);
    Serial.print(F("."));
    Serial.print((rxPacket.sensordata.Batt_mV / 100) % 10);
    Serial.print((rxPacket.sensordata.Batt_mV / 10) % 10);
    Serial.print(rxPacket.sensordata.Batt_mV % 10);
    if (rxPacket.sensordata.Batt_mV < 2200) {
      Serial.print(F("   *** Out of Spec ***"));
    }
    Serial.println(F(" "));
    Serial.print(F("Loops: "));
    Serial.println(rxPacket.sensordata.Loops);
    Serial.print(F("Millis: "));
    Serial.println(rxPacket.sensordata.Millis);
    if (rxPacket.from == ADDRESS_SENSOR6) {
      Serial.print(F("Light: "));
      Serial.println(rxPacket.sensordata.Light_Sensor);
      Serial.print(F("Door: "));
      Serial.println(rxPacket.sensordata.Door_Sensor);
    }
#ifdef LCD_ENABLED
    displayTempOnLCD(rxPacket.sensordata.MSP_T);
    switch (rxPacket.from) {
      case ADDRESS_G2:
        myLCD.showSymbol(LCD_SEG_CLOCK, 1);
        myLCD.showSymbol(LCD_SEG_R, 1);
        currentDisplay = ADDRESS_G2;
        temperatures[ADDRESS_G2 - 2] = rxPacket.sensordata.MSP_T;
        batteries[ADDRESS_G2 - 2]    = rxPacket.sensordata.Batt_mV;
        break;
      case ADDRESS_SENSOR4:
        myLCD.showSymbol(LCD_SEG_HEART, 1);
        currentDisplay = ADDRESS_SENSOR4;
        temperatures[ADDRESS_SENSOR4 - 2] = rxPacket.sensordata.MSP_T;
        batteries[ADDRESS_SENSOR4 - 2]    = rxPacket.sensordata.Batt_mV;
        break;
      case ADDRESS_SENSOR5:
        myLCD.showSymbol(LCD_SEG_HEART, 1);
        myLCD.showSymbol(LCD_SEG_R, 1);
        currentDisplay = ADDRESS_SENSOR5;
        temperatures[ADDRESS_SENSOR5 - 2] = rxPacket.sensordata.MSP_T;
        batteries[ADDRESS_SENSOR5 - 2]    = rxPacket.sensordata.Batt_mV;
        break;
      case ADDRESS_SENSOR6:
        myLCD.showSymbol(LCD_SEG_HEART, 1);
        myLCD.showSymbol(LCD_SEG_CLOCK, 1);
        currentDisplay = ADDRESS_SENSOR6;
        temperatures[ADDRESS_SENSOR6 - 2] = rxPacket.sensordata.MSP_T;
        batteries[ADDRESS_SENSOR6 - 2]    = rxPacket.sensordata.Batt_mV;
        break;
      default:
        break;
    }
    displayBattOnLCD(rxPacket.sensordata.Batt_mV);
#endif
#ifdef ETHERNET_ENABLED
#ifdef LCD_ENABLED
    myLCD.showSymbol(LCD_SEG_TX, 1);
#endif
    payload[0] = '\0';
    BuildPayload(payload, fieldBuffer, 1, rxPacket.sensordata.MSP_T);
    BuildPayload(payload, fieldBuffer, 2, rxPacket.sensordata.Batt_mV);
    BuildPayload(payload, fieldBuffer, 3, rxPacket.sensordata.Loops);
    BuildPayload(payload, fieldBuffer, 4, rxPacket.sensordata.Millis);
    BuildPayload(payload, fieldBuffer, 5, lastRssi);
    BuildPayload(payload, fieldBuffer, 6, lastLqi);
    switch (rxPacket.from) {
      case ADDRESS_G2:
        Serial.println(F("Sending data to ThingSpeak..."));
        Serial.print(F("Payload: "));
        Serial.println(payload);
        if (! Temp_Slim.publish(payload)) {
          Serial.println(F("Temp_Slim Channel Failed to ThingSpeak."));
        }
        Serial.println(F("Sending data to Cayenne..."));
        payload[0] = '\0';
        sprintf(payload, "temp,f=%d.%d", rxPacket.sensordata.MSP_T/10, rxPacket.sensordata.MSP_T%10);
        if (! Slim_T.publish(payload)) {
          Serial.println(F("MSP_T Failed"));
        }
        payload[0] = '\0';
        sprintf(payload, "voltage,mv=%d", rxPacket.sensordata.Batt_mV);
        if (! Slim_BATT.publish(payload)) {
          Serial.println(F("Batt Failed"));
        }
        break;
      case ADDRESS_SENSOR4:
        Serial.println(F("Sending data to ThingSpeak..."));
        Serial.print(F("Payload: "));
        Serial.println(payload);
        if (! Temp_Sensor4.publish(payload)) {
          Serial.println(F("Temp_Sensor4 Channel Failed to ThingSpeak."));
        }
        break;
      case ADDRESS_SENSOR5:
        Serial.println(F("Sending data to ThingSpeak..."));
        Serial.print(F("Payload: "));
        Serial.println(payload);
        if (! Temp_Sensor5.publish(payload)) {
          Serial.println(F("Temp_Sensor5 Channel Failed to ThingSpeak."));
        }
        Serial.println(F("Sending data to Cayenne..."));
        payload[0] = '\0';
        sprintf(payload, "temp,f=%d.%d", rxPacket.sensordata.MSP_T/10, rxPacket.sensordata.MSP_T%10);
        if (! Indoor_T.publish(payload)) {
          Serial.println(F("MSP_T Failed"));
        }
        payload[0] = '\0';
        sprintf(payload, "voltage,mv=%d", rxPacket.sensordata.Batt_mV);
        if (! Indoor_BATT.publish(payload)) {
          Serial.println(F("Batt Failed"));
        }
        break;
      case ADDRESS_SENSOR6:
        BuildPayload(payload, fieldBuffer, 7, rxPacket.sensordata.Light_Sensor);
        BuildPayload(payload, fieldBuffer, 8, rxPacket.sensordata.Door_Sensor);
        Serial.println(F("Sending data to ThingSpeak..."));
        Serial.print(F("Payload: "));
        Serial.println(payload);
        if (! Temp_Sensor6.publish(payload)) {
          Serial.println(F("Temp_Sensor6 Channel Failed to ThingSpeak."));
        }
        Serial.println(F("Sending data to Cayenne..."));
        payload[0] = '\0';
        sprintf(payload, "temp,f=%d.%d", rxPacket.sensordata.MSP_T/10, rxPacket.sensordata.MSP_T%10);
        if (! Garage_T.publish(payload)) {
          Serial.println(F("MSP_T Failed"));
        }
        payload[0] = '\0';
        sprintf(payload, "voltage,mv=%d", rxPacket.sensordata.Batt_mV);
        if (! Garage_BATT.publish(payload)) {
          Serial.println(F("Batt Failed"));
        }
        payload[0] = '\0';
        sprintf(payload, "analog_sensor=%d", rxPacket.sensordata.Door_Sensor);
        if (! Garage_DOOR.publish(payload)) {
          Serial.println(F("Door Failed"));
        }
        break;
      default:
        break;
    }
#ifdef LCD_ENABLED
    myLCD.showSymbol(LCD_SEG_TX, 0);
#endif
#endif
  }
}

#ifdef LCD_ENABLED
void displayTempOnLCD(int temp) {
  char tempChar[32];
  int tempLen;
  int tempSign;

  if (temp < 0) {
    tempSign = 1;
    temp = -temp;
  } else tempSign = 0;
  itoa(temp, tempChar, 10);
  tempLen = strlen(tempChar);
  myLCD.clear();
  // Need to add a leading zero if len == 1
  if (tempLen == 1) {
    char x = tempChar[0];
    tempChar[0] = 0;
    tempChar[1] = x;
    tempChar[2] = '\0';
    tempLen = 2;
  }
  for (int i = 0; i < tempLen; i++) {
    myLCD.showChar(tempChar[i], 5 - tempLen + i);
  }
  myLCD.showSymbol(LCD_SEG_DOT4, 1);
  myLCD.showSymbol(LCD_SEG_DEG5, 1);
  myLCD.showSymbol(LCD_SEG_MINUS1, tempSign);
  myLCD.showSymbol(LCD_SEG_RADIO, RadioStatus);
  myLCD.showSymbol(LCD_SEG_MARK, MarkStatus);
  myLCD.showSymbol(LCD_SEG_TX, TxStatus);
}

void displayBattOnLCD(int mV) {
  if (mV > 3200) myLCD.showSymbol(LCD_SEG_BAT5, 1);
  if (mV > 3000) myLCD.showSymbol(LCD_SEG_BAT4, 1);
  if (mV > 2800) myLCD.showSymbol(LCD_SEG_BAT3, 1);
  if (mV > 2600) myLCD.showSymbol(LCD_SEG_BAT2, 1);
  if (mV > 2400) myLCD.showSymbol(LCD_SEG_BAT1, 1);
  if (mV > 2200) myLCD.showSymbol(LCD_SEG_BAT0, 1);
  myLCD.showSymbol(LCD_SEG_BAT_ENDS, 1);
  myLCD.showSymbol(LCD_SEG_BAT_POL, 1);
}
#endif

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care of connecting.
#ifdef ETHERNET_ENABLED
void MQTT_connect(Adafruit_MQTT_Client* mqtt_server, EthernetClient* client ) {
  int8_t ret;

  // Return if already connected.
  if (mqtt_server->connected()) {
    MarkStatus = 0;
#ifdef LCD_ENABLED
    myLCD.showSymbol(LCD_SEG_MARK, MarkStatus);
#endif
    return;
  }

  Serial.println(F("MQTT Disconnected."));
  MarkStatus = 1;
#ifdef LCD_ENABLED
  myLCD.showSymbol(LCD_SEG_MARK, MarkStatus);
#endif
  printClientStatus(client);
  Serial.print(F("Attempting reconnect to MQTT: "));
  Serial.println(millis());
  ret = mqtt_server->connect();
  printClientStatus(client);
  // W5200 chip seems to have a problem of getting stuck in CLOSE_WAIT state
  // There may be other issues, so just force a hard reset on ethernet
  // shield if we lose connection
  if (ret != 0) {
    lostConnectionCount++;
    Serial.print(F("Ethernet connection lost #: "));
    Serial.println(lostConnectionCount);
    if (lostConnectionCount > 5) {
      
      Serial.print(F("Ethernet connection loss over max: "));
      Serial.println(lostConnectionCount);
      lostConnectionCount = 0;
#ifdef OLED_ENABLED
      // Turn off the OLED while waiting to reconnect
      oled.command(0x08);
      displayTimeoutCount = 0;
#endif
      client_cayenne.stop();
      client_ts.stop();
      digitalWrite(W5200_RESET, LOW);
      delay(5);
      digitalWrite(W5200_RESET, HIGH);
      delay(200); // Need to delay at least 150 ms after reset
      Serial.println(F("Starting Ethernet..."));
      Ethernet.begin(mac);
      Serial.println(F("Ethernet enabled."));
    }
  }
  Serial.println(mqtt_server->connectErrorString(ret));
  if (ret == 0) {
    MarkStatus = 0;
#ifdef LCD_ENABLED
    myLCD.showSymbol(LCD_SEG_MARK, MarkStatus);
#endif
  }
  /* Comment out the loop; just try once and let loop() take care of reconnecting
    while ((ret = mqtt_server->connect()) != 0) { // connect will return 0 for connected
      Serial.println(mqtt_server->connectErrorString(ret));
      Serial.println("Retrying MQTT connection in 5 seconds...");
      mqtt_server->disconnect();
      delay(5000);  // wait 5 seconds
    }
    Serial.println("MQTT Connected!");
  */
}
#endif

#ifdef ETHERNET_ENABLED
int printClientStatus(EthernetClient* client) {
  int ClientStatus;
  ClientStatus = client->status();
  if (ClientStatus != 0x17) {
    Serial.print(F("Ethernet Client Status: "));
    Serial.print(F(" - "));
  }
  switch (ClientStatus) {
    case 0:
      Serial.println(F("CLOSED"));
      break;
    case 0x13:
      Serial.println(F("INIT"));
      break;
    case 0x14:
      Serial.println(F("LISTEN"));
      break;
    case 0x15:
      Serial.println(F("SYNSENT"));
      break;
    case 0x16:
      Serial.println(F("SYNRECV"));
      break;
    case 0x17:
#ifdef PRINT_ALL_CLIENT_STATUS
      Serial.print(F("Ethernet Client Status: "));
      Serial.print(F(" - "));
      Serial.println(F("ESTABLISHED"));
#endif
      break;
    case 0x18:
      Serial.println(F("FIN_WAIT"));
      break;
    case 0x1a:
      Serial.println(F("CLOSING"));
      break;
    case 0x1b:
      Serial.println(F("TIME_WAIT"));
      break;
    case 0x1c:
      Serial.println(F("CLOSE_WAIT"));
      break;
    case 0x1d:
      Serial.println(F("LAST_ACK"));
      break;
    case 0x22:
      Serial.println(F("UTP"));
      break;
    case 0x32:
      Serial.println(F("IPRAW"));
      break;
    case 0x42:
      Serial.println(F("MACRAW"));
      break;
    case 0x5f:
      Serial.println(F("PPPOE"));
      break;
    default:
      Serial.print(F("Unknown State: "));
      Serial.println(ClientStatus, 16);
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

  // Make sure the printed values are bounded to fit in the display width
  timeSince = (millis() - lastWeatherMillis) / 1000;
  if (timeSince > 99999) timeSince = 99999;
  // Print # seconds since last message received
  splen = snprintf((char*)oled_text[WEATHER_ROW], OLED_COLS + 1, "Weather: %d", timeSince);
  // Pad the rest of the string with spaces.
  for (int i = splen; i < OLED_COLS; i++) {
    oled_text[WEATHER_ROW][i] = ' ';
  }
  oled_text[WEATHER_ROW][OLED_COLS] = '\0';

  // Make sure the printed values are bounded to fit in the display width
  timeSince = (millis() - lastG2Millis) / 1000;
  if (timeSince > 99999) timeSince = 99999;
  // Print # seconds since last message received
  splen = snprintf((char*)oled_text[G2_ROW], OLED_COLS + 1, "Slim: %d", timeSince);
  // Pad the rest of the string with spaces.
  for (int i = splen; i < OLED_COLS; i++) {
    oled_text[G2_ROW][i] = ' ';
  }
  oled_text[G2_ROW][OLED_COLS] = '\0';
}
#endif

#ifdef ETHERNET_ENABLED
void process_failedCRC() {
  payload[0] = '\0';
  BuildPayload(payload, fieldBuffer, 1, lastRssi);
  BuildPayload(payload, fieldBuffer, 2, lastLqi);
  BuildPayload(payload, fieldBuffer, 3, rxPacket.from);
  BuildPayload(payload, fieldBuffer, 12, "CRC failed from: ");
  sprintf(fieldBuffer, "%d", rxPacket.from);
  strcat(payload, fieldBuffer);
  strcat(payload, ", RSSI: ");
  sprintf(fieldBuffer, "%d", lastRssi);
  strcat(payload, fieldBuffer);
  strcat(payload, ", LQI: ");
  sprintf(fieldBuffer, "%d", lastLqi);
  strcat(payload, fieldBuffer);
  if (! Sensor_Errors.publish(payload)) {
    Serial.println(F("Failed to send bad CRC to ThingSpeak"));
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
    sprintf(numBuffer, "%d", fieldNum);
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
  sprintf(dataFieldBuffer, "%d", data);
  BuildPayload(msgBuffer, fieldNum, dataFieldBuffer);
}

void BuildPayload(char* msgBuffer, char* dataFieldBuffer, int fieldNum, unsigned int data) {
  sprintf(dataFieldBuffer, "%u", data);
  BuildPayload(msgBuffer, fieldNum, dataFieldBuffer);
}

void BuildPayload(char* msgBuffer, char* dataFieldBuffer, int fieldNum, unsigned long data) {
  sprintf(dataFieldBuffer, "%lu", data);
  BuildPayload(msgBuffer, fieldNum, dataFieldBuffer);
}

void BuildPayload(char* msgBuffer, char* dataFieldBuffer, int fieldNum, const char* data) {
  sprintf(dataFieldBuffer, data);
  BuildPayload(msgBuffer, fieldNum, dataFieldBuffer);
}
#endif
