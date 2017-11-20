/**
   Sensor Receiver Station
   1.0 - 11/19/17 - A.T. - Original
   1.1 - 11/19/17 - A.T. - Display info on built-in LCD
*/

/**
   Receiver hub for various sensor transmitting modules.
   Uses Anaren CC110L BoosterPacks

   Currently defined transmitters
   - Outdoor weather station using SENSORHUB BoosterPack
   - Simple temp monitor using MSP430G2553 internal sensor
   - Other transmitters may be defined in the future

   The receiving station code needs to be modified whenever
   a new sensor is added:
   - Sensor CC110L device address
   - Data payload structure
   - Output stream (Serial, LCD, and/or IP)
*/
#include <SPI.h>
#include <AIR430BoostFCC.h>
#include "LCD_Launchpad.h"

// -----------------------------------------------------------------------------


#define ADDRESS_LOCAL    0x01
#define ADDRESS_WEATHER  0x02
#define ADDRESS_G2       0x03

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
LCD_LAUNCHPAD myLCD;

void setup()
{

  // Setup serial for debug printing.
  Serial.begin(9600);
  Serial.println("CC110L receiver");
  delay(500);

  rxPacket.from = 0;
  memset(rxPacket.message, 0, sizeof(rxPacket.message));
  Radio.begin(ADDRESS_LOCAL, CHANNEL_1, POWER_MAX);

  pinMode(RED_LED, OUTPUT);       // Use red LED to display message reception
  digitalWrite(RED_LED, HIGH);
  delay(500);
  digitalWrite(RED_LED, LOW);

  myLCD.init();
}

void loop()
{
  int packetSize;
  // Turn on the receiver and listen for incoming data. Timeout after 1 seconds.
  // The receiverOn() method returns the number of bytes copied to rxData.
  // The radio library uses the SPI library internally, this call initializes
  // SPI/CSn and GDO0 lines. Also setup initial address, channel, and TX power.
  packetSize = Radio.receiverOn((unsigned char*)&rxPacket, sizeof(rxPacket), 1000);

  if (packetSize > 0)
  {
    digitalWrite(RED_LED, HIGH);
    Serial.println("--");
    if (rxPacket.from == ADDRESS_WEATHER) {
      memcpy(&weatherdata, &rxPacket.message, sizeof(weatherdata));
      Serial.print("From device: ");
      Serial.print(rxPacket.from);
      Serial.print(", bytes: ");
      Serial.println(packetSize);
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

      displayTempOnLCD(weatherdata.BMP180_T);
      myLCD.showSymbol(LCD_SEG_CLOCK, 1);
      displayBattOnLCD(weatherdata.Batt_mV);
    }
    if (rxPacket.from == ADDRESS_G2) {
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
      displayTempOnLCD(sensordata.MSP_T);
      myLCD.showSymbol(LCD_SEG_HEART, 1);
      displayBattOnLCD(sensordata.Batt_mV);
    }
    Serial.print("RSSI: ");
    Serial.println(Radio.getRssi());
    Serial.print("LQI: ");
    Serial.println(Radio.getLqi());
    Serial.println(F("--"));
    digitalWrite(RED_LED, LOW);
  }
  else {
    Serial.println("Nothing received.");
  }
}

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
    tempChar[2] = '/0';
    tempLen = 2;
  }
  for (int i = 0; i < tempLen; i++) {
    myLCD.showChar(tempChar[i], 5 - tempLen + i);
  }
  myLCD.showSymbol(LCD_SEG_DOT4, 1);
  myLCD.showSymbol(LCD_SEG_DEG5, 1);
  myLCD.showSymbol(LCD_SEG_MINUS1, tempSign);
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

