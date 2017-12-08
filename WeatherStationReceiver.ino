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
   - Other transmitters may be defined in the future

   The receiving station code needs to be modified whenever
   a new sensor is added:
   - Sensor CC110L device address
   - Data payload structure
   - Output stream (Serial, LCD, and/or IP)
**/
/**
   EXTERNAL LIBRARIES:
   - Ethernet: https://github.com/Wiznet/WIZ_Ethernet_Library
       - Modified to work with Energia
       - #define updates to support Seeed W5200 Ethernet Shield V2.2
   - MQTT:
       - Adafruit_MQTT.cpp modified to comment out lines 425-431
         to remove support for floating point. Specifically, 
         commented out the block starting with:
            "else if (sub->callback_double != NULL)"
*/

/* BOARD CONFIGURATION AND OTHER DEFINES
   -------------------------------------

   LED to signal received message:
   For FR4133:
      #define BOARD_LED LED2
      (Do not use LED1, as this conflicts with the UART
   For FR6989:
      #define BOARD_LED RED_LED
      or
      #define BOARD_LED GREEN_LED
   For other LaunchPads, check the user guides

   When using a LaunchPad with a built-in LCD (FR4133, FR6989):
      #define LCD_ENABLED
   On other LaunchPads, comment out the line:
      //#define LCD_ENABLED

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
#define BOARD_LED GREEN_LED
#define LCD_ENABLED
#define ETHERNET_ENABLED
//#define PRINT_ALL_CLIENT_STATUS

/* CONTROL PIN DEFINITIONS 
 * -----------------------
 * 
 *  W5200_RESET - Connected to RESET on W5200 chip. Active LOW. 
 *  W5200_CS    - Connected to SCS pin on W5200 chip. Active LOW. 
 *  CC110L_CS   - Connected to CS pin on CC110L chip. Active LOW.
 *  
 *  NOTE: The CS pins are also defined separately in the associated driver
 *       libraries. Both the driver libraries and the following 
 *       definitions need to be updated in order to change the pin 
 *       assignments
 */

#define W5200_RESET 5
#define W5200_CS    8
#define CC110L_CS  18 

#include <SPI.h>
#include <AIR430BoostFCC.h>

#ifdef LCD_ENABLED
#include "LCD_Launchpad.h"
LCD_LAUNCHPAD myLCD;
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
*/
EthernetClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
#endif

// -----------------------------------------------------------------------------


#define ADDRESS_LOCAL    0x01
#define ADDRESS_WEATHER  0x02
#define ADDRESS_G2       0x03
#define LAST_ADDRESS     0x03

#define RF_GDO0       19

struct sPacket  // CC110L packet structure
{
  uint8_t from;           // Local node address that message originated from
  uint8_t message[59];    // Local node message [MAX. 59 bytes]
};

// -----------------------------------------------------------------------------
/**
    Global data
*/

struct sPacket rxPacket;

struct WeatherData {
  int             BMP180_T;  // Tenth degrees F
  unsigned int    BMP180_P;  // Pressure in inches of Hg * 100
  int             TMP106_Ti; // Tenth degrees F
  unsigned int    LUX;       // Limited in code to 65535
  int             SHT_T;     // Tenth degrees F
  int             SHT_H;     // Tenth % Relative Humidity * 10
  int             MSP_T;     // Tenth degrees F
  unsigned int    Batt_mV;   // milliVolts
  unsigned int    Loops;
  unsigned long   Millis;
  unsigned int    Resets;
};

struct G2Sensor {
  int             MSP_T;     // Tenth degrees F
  unsigned int    Batt_mV;   // milliVolts
  unsigned int    Loops;
  unsigned long   Millis;
};

WeatherData weatherdata;
G2Sensor sensordata;

#ifdef LCD_ENABLED
int currentDisplay; //Remember which temp value is on LCD
int temperatures[LAST_ADDRESS - 1]; // Store last temp value received
int batteries[LAST_ADDRESS - 1];  // Store last battery reading
// The following values store the state of the status symbols so they
// can be restored after an LCD.clear() operation:
int  MarkStatus = 0;
int  RadioStatus = 0;
int  TxStatus = 0;
#endif

/* MQTT publishing feeds
     Each feed that you wish to publish needs to be defined.
     Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>, for example: 
        Adafruit_MQTT_Publish pressure = Adafruit_MQTT_Publish(&mqtt,  AIO_USERNAME "/feeds/pressure");
*/
#ifdef ETHERNET_ENABLED
/* The MQTT_private_feeds.h file needs to include the following definitions
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

  // Setup serial for status printing.
  Serial.begin(9600);
  Serial.println(" ");
  Serial.println("Sensor receiver hub with CC110L.");
  delay(500);

#ifdef LCD_ENABLED
  myLCD.init();
#ifdef ETHERNET_ENABLED
  myLCD.displayText("ETHER ");
  // Display the "!" LCD symbol to show we are initializing
  MarkStatus = 1;
  myLCD.showSymbol(LCD_SEG_MARK, MarkStatus);
  //  delay(500);
#endif
  Serial.println("Started LCD display");
#endif

#ifdef ETHERNET_ENABLED
  Serial.println("Starting Ethernet...");
  Ethernet.begin(mac);
  //  delay(1000);            // Give it a second to initialize
  Serial.println("Ethernet enabled.");
  MQTT_connect();
#endif

  // Setup CC110L data sstructure
  rxPacket.from = 0;
  memset(rxPacket.message, 0, sizeof(rxPacket.message));
  /// DEBUG  Serial.println("Start CC110L");
  /// DEBUG  Radio.begin(ADDRESS_LOCAL, CHANNEL_1, POWER_MAX);
  /// DEBUG  SPI.end();

  pinMode(BOARD_LED, OUTPUT);       // Use red LED to display message reception
  digitalWrite(BOARD_LED, HIGH);
  delay(500);
  digitalWrite(BOARD_LED, LOW);

  pinMode(PUSH1, INPUT_PULLUP);

#ifdef LCD_ENABLED
  for (int i = 0; i < (LAST_ADDRESS - 2); i++) {
    temperatures[i] = 0;
    batteries[i] = 0;
  }
  myLCD.displayText("RX ON ");
  RadioStatus = 1;
  myLCD.showSymbol(LCD_SEG_RADIO, RadioStatus);
  Serial.println("Waiting for first Rx message. ");
#endif
}

void loop()
{
#ifdef ETHERNET_ENABLED
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).
  MQTT_connect();
  int EthStatus, ClientStatus;
#ifdef PRINT_ALL_CLIENT_STATUS
  EthStatus = Ethernet.maintain();
  Serial.print("Ethernet Maintain status: ");
  Serial.println(EthStatus);
#endif
  ClientStatus = printClientStatus();
#endif

  int packetSize;

  // Turn on the receiver and listen for incoming data. Timeout after 1 seconds.
  // The receiverOn() method returns the number of bytes copied to rxData.
  // The radio library uses the SPI library internally, this call initializes
  // SPI/CSn and GDO0 lines. Also setup initial address, channel, and TX power.
  SPI.begin(18);
  Radio.begin(ADDRESS_LOCAL, CHANNEL_1, POWER_MAX);
  packetSize = Radio.receiverOn((unsigned char*)&rxPacket, sizeof(rxPacket), 1000);
  // Simulate a Radio.end() call, but don't want to disable Chip Select pin
  // Radio.end();
  while (Radio.busy()) ; // Empty loop statement
  detachInterrupt(RF_GDO0);
  pinMode(CC110L_CS, OUTPUT);    // Need to pull radio CS high to keep it off the SPI bus

  if (packetSize > 0) {
    digitalWrite(BOARD_LED, HIGH);
#ifdef LCD_ENABLED
    myLCD.showSymbol(LCD_SEG_RX, 1);
#endif
    Serial.println("--");
    Serial.print("Received packet from device: ");
    Serial.print(rxPacket.from);
    Serial.print(", bytes: ");
    Serial.println(packetSize);

    switch (rxPacket.from) {
      case (ADDRESS_WEATHER):
        process_weatherdata();
        break;
      case (ADDRESS_G2):
        process_G2data();
        break;
      default:
        Serial.println("Message received from unknown sensor.");
        break;
    }

    Serial.print("RSSI: ");
    Serial.println(Radio.getRssi());
    Serial.print("LQI: ");
    Serial.println(Radio.getLqi());
    Serial.println(F("--"));
    digitalWrite(BOARD_LED, LOW);
#ifdef LCD_ENABLED
    myLCD.showSymbol(LCD_SEG_RX, 0);
#endif
  }
  else {
    Serial.println("Nothing received.");
  }

#ifdef LCD_ENABLED
  if (digitalRead(PUSH1) == 0) {
    myLCD.clear();
    currentDisplay++;
    if (currentDisplay > LAST_ADDRESS) currentDisplay = 2;
    displayTempOnLCD(temperatures[currentDisplay - 2]);
    displayBattOnLCD(batteries[currentDisplay - 2]);
    switch (currentDisplay) {
      case ADDRESS_WEATHER:
        myLCD.showSymbol(LCD_SEG_CLOCK, 1);
        break;
      case ADDRESS_G2:
        myLCD.showSymbol(LCD_SEG_HEART, 1);
        break;
      default:
        break;
    }
  }
#endif

#ifdef ETHERNET_ENABLED
  // ping the server to keep the mqtt connection alive
  if (! mqtt.ping()) {
    mqtt.disconnect();
    Serial.println("MQTT ping failed, disconnecting.");
#ifdef LCD_ENABLED
    MarkStatus = 1;
    myLCD.showSymbol(LCD_SEG_MARK, MarkStatus);
#endif
  }
#endif
}

void process_weatherdata() {
  memcpy(&weatherdata, &rxPacket.message, sizeof(weatherdata));
  Serial.println("Temperature (F): ");
  Serial.print("    BMP180:  ");
  Serial.print(weatherdata.BMP180_T / 10);
  Serial.print(".");
  Serial.println(weatherdata.BMP180_T % 10);
  Serial.print("    TMP106:  ");
  Serial.print(weatherdata.TMP106_Ti / 10);
  Serial.print(".");
  Serial.println(weatherdata.TMP106_Ti % 10);
  Serial.print("    SHT21:   ");
  Serial.print(weatherdata.SHT_T / 10);
  Serial.print(".");
  Serial.println(weatherdata.SHT_T % 10);
  Serial.print("    MSP Die: ");
  Serial.print(weatherdata.MSP_T / 10);
  Serial.print(".");
  Serial.println(weatherdata.MSP_T % 10);
  Serial.print("Pressure (inHg): ");
  Serial.print(weatherdata.BMP180_P / 100);
  Serial.print(".");
  Serial.print((weatherdata.BMP180_P / 10) % 10);
  Serial.println(weatherdata.BMP180_P % 10);
  Serial.print("%RH: ");
  Serial.print(weatherdata.SHT_H / 10);
  Serial.print(".");
  Serial.println(weatherdata.SHT_H % 10);
  Serial.print("Lux: ");
  Serial.println(weatherdata.LUX);
  Serial.print("Battery V: ");
  Serial.print(weatherdata.Batt_mV / 1000);
  Serial.print(".");
  Serial.print((weatherdata.Batt_mV / 100) % 10);
  Serial.print((weatherdata.Batt_mV / 10) % 10);
  Serial.print(weatherdata.Batt_mV % 10);
  if (weatherdata.Batt_mV < 2200) {
    Serial.print("   *** Out of Spec ***");
  }
  Serial.println(" ");
  Serial.print("Loops: ");
  Serial.println(weatherdata.Loops);
  Serial.print("Millis: ");
  Serial.println(weatherdata.Millis);
  Serial.print("Resets: ");
  Serial.println(weatherdata.Resets);

#ifdef LCD_ENABLED
  displayTempOnLCD(weatherdata.BMP180_T);
  myLCD.showSymbol(LCD_SEG_CLOCK, 1);
  displayBattOnLCD(weatherdata.Batt_mV);
  currentDisplay = ADDRESS_WEATHER;
  temperatures[ADDRESS_WEATHER - 2] = weatherdata.BMP180_T;
  batteries[ADDRESS_WEATHER - 2]    = weatherdata.Batt_mV;
#endif

#ifdef ETHERNET_ENABLED
#ifdef LCD_ENABLED
  myLCD.showSymbol(LCD_SEG_TX, 1);
#endif
  Serial.println("Sending data to MQTT...");
  if (! Weather_T_BMP180.publish((int32_t)weatherdata.BMP180_T)) {
    Serial.println(F("BMP180T Failed"));
  }
  if (! Weather_T_SHT21.publish((int32_t)weatherdata.SHT_T)) {
    Serial.println(F("SHT21T Failed"));
  }
  if (! Weather_P.publish((uint32_t)weatherdata.BMP180_P)) {
    Serial.println(F("BMP180P Failed"));
  }
  if (! Weather_RH.publish((uint32_t)weatherdata.SHT_H)) {
    Serial.println(F("RH Failed"));
  }
  if (! Weather_LUX.publish((uint32_t)weatherdata.LUX)) {
    Serial.println(F("LUX Failed"));
  }
  if (! Weather_Batt.publish((uint32_t)weatherdata.Batt_mV)) {
    Serial.println(F("Batt Failed"));
  }
  if (! Weather_Loops.publish((uint32_t)weatherdata.Loops)) {
    Serial.println(F("Loops Failed"));
  }
#ifdef LCD_ENABLED
  myLCD.showSymbol(LCD_SEG_TX, 0);
#endif
#endif
}

void process_G2data() {
  memcpy(&sensordata, &rxPacket.message, sizeof(sensordata));
  Serial.println("Received packet from G2");
  Serial.print("Temperature (F): ");
  Serial.print(sensordata.MSP_T / 10);
  Serial.print(".");
  Serial.println(sensordata.MSP_T % 10);
  Serial.print("Battery V: ");
  Serial.print(sensordata.Batt_mV / 1000);
  Serial.print(".");
  Serial.print((sensordata.Batt_mV / 100) % 10);
  Serial.print((sensordata.Batt_mV / 10) % 10);
  Serial.print(sensordata.Batt_mV % 10);
  if (sensordata.Batt_mV < 2200) {
    Serial.print("   *** Out of Spec ***");
  }
  Serial.println(" ");
  Serial.print("Loops: ");
  Serial.println(sensordata.Loops);
  Serial.print("Millis: ");
  Serial.println(sensordata.Millis);
#ifdef LCD_ENABLED
  displayTempOnLCD(sensordata.MSP_T);
  myLCD.showSymbol(LCD_SEG_HEART, 1);
  displayBattOnLCD(sensordata.Batt_mV);
  currentDisplay = ADDRESS_G2;
  temperatures[ADDRESS_G2 - 2] = sensordata.MSP_T;
  batteries[ADDRESS_G2 - 2]    = sensordata.Batt_mV;
#endif
#ifdef ETHERNET_ENABLED
#ifdef LCD_ENABLED
  myLCD.showSymbol(LCD_SEG_TX, 1);
#endif
  Serial.println("Sending data to MQTT...");
  if (! Slim_T.publish((int32_t)sensordata.MSP_T)) {
    Serial.println(F("MSP_T Failed"));
  }
  if (! Slim_Batt.publish((uint32_t)sensordata.Batt_mV)) {
    Serial.println(F("Batt Failed"));
  }
#ifdef LCD_ENABLED
  myLCD.showSymbol(LCD_SEG_TX, 0);
#endif
#endif
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
void MQTT_connect() {
  int8_t ret, clientStatus;

  // Return if already connected.
  if (mqtt.connected()) {
#ifdef LCD_ENABLED
    MarkStatus = 0;
    myLCD.showSymbol(LCD_SEG_MARK, MarkStatus);
#endif
    return;
  }

  Serial.println("MQTT Disconnected.");
#ifdef LCD_ENABLED
  MarkStatus = 1;
  myLCD.showSymbol(LCD_SEG_MARK, MarkStatus);
#endif
  printClientStatus();
  Serial.print("Attempting reconnect to MQTT... ");
  ret = mqtt.connect();
  clientStatus = printClientStatus();
  // W5200 chip seems to have a problem of getting stuck in CLOSE_WAIT state
  // If in CLOSE_WAIT, then force a non-DNS connection to clear
  if (clientStatus == 0x1c) { // 0x1c == CLOSE_WAIT
    IPAddress ip(127, 0, 0, 1);
    client.connect(ip, 1024);
    client.stop();
  }
  Serial.println(mqtt.connectErrorString(ret));
#ifdef LCD_ENABLED
  if (ret == 0) {
    MarkStatus = 0;
    myLCD.showSymbol(LCD_SEG_MARK, MarkStatus);
  }
#endif
  /* Comment out the loop; just try once and let loop() take care of reconnecting
    while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
      Serial.println(mqtt.connectErrorString(ret));
      Serial.println("Retrying MQTT connection in 5 seconds...");
      mqtt.disconnect();
      delay(5000);  // wait 5 seconds
    }
    Serial.println("MQTT Connected!");
  */
}
#endif

#ifdef ETHERNET_ENABLED
int printClientStatus() {
  int ClientStatus;
  ClientStatus = client.status();
  if (ClientStatus != 0x17) {
    Serial.print("Ethernet Client Status: ");
    Serial.print(" - ");
  }
  switch (ClientStatus) {
    case 0:
      Serial.println("CLOSED");
      break;
    case 0x13:
      Serial.println("INIT");
      break;
    case 0x14:
      Serial.println("LISTEN");
      break;
    case 0x15:
      Serial.println("SYNSENT");
      break;
    case 0x16:
      Serial.println("SYNRECV");
      break;
    case 0x17:
#ifdef PRINT_ALL_CLIENT_STATUS
      Serial.print("Ethernet Client Status: ");
      Serial.print(" - ");
      Serial.println("ESTABLISHED");
#endif
      break;
    case 0x18:
      Serial.println("FIN_WAIT");
      break;
    case 0x1a:
      Serial.println("CLOSING");
      break;
    case 0x1b:
      Serial.println("TIME_WAIT");
      break;
    case 0x1c:
      Serial.println("CLOSE_WAIT");
      break;
    case 0x1d:
      Serial.println("LAST_ACK");
      break;
    case 0x22:
      Serial.println("UTP");
      break;
    case 0x32:
      Serial.println("IPRAW");
      break;
    case 0x42:
      Serial.println("MACRAW");
      break;
    case 0x5f:
      Serial.println("PPPOE");
      break;
    default:
      Serial.print("Unknown State: ");
      Serial.println(ClientStatus, 16);
      break;
  }
  return ClientStatus;
}
#endif
