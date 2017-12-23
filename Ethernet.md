WizNet Ethernet Library For Energia
===================================

The SeeedStudio Ethernet Shield used in the Wireless Sensor Receiver Hub
project uses a W5200 ethernet chip from WIZnet.

The WIZnet-provided Arduino library for the W5200 chip requires some changes
to work with Energia and MSP430 controllers.

### Installing WIZnet Ethernet Library for Energia ###

1. Download ZIP from GitHub: https://github.com/Wiznet/WIZ_Ethernet_Library
2. Navigate to "WIZ_Ethernet_Library-master/Arduino IDE 1.5.x" inside the ZIP folder.
3. Copy the "Ethernet" to your Energia libraries folder inside your sketch folder.
* Navigate to "WIZ_Ethernet_Library-master\Arduino IDE 1.0.x\Ethernet" inside the ZIP folder.
* Copy the entire Examples folder as-is to the Ethernet folder created above
* Download and extract "additional files" from http://wizwiki.net/wiki/lib/exe/fetch.php?media=osh:ioshield-l:updatelib:additional_files_from_arduino.zip
* Copy the "additional files" into the "src" folder of the Ethernet library you copied above
    * Note: "IPAddress.h" and "IPAddress.cpp" may not be needed and may cause a conflict when compiling. If you get an error related to "IPAddress" when compiling, then delete these two files.
* Modify the following files in the new Ethernet/src/utility folder
    * w5100.h - Change the the following lines from:  
         `//#define W5200_ETHERNET_SHIELD`
         `#define W5500_ETHERNET_SHIELD`         
    * To:  
          `#define W5200_ETHERNET_SHIELD`
          `//#define W5500_ETHERNET_SHIELD`
    * That is, uncomment the W5200 definition, and comment out W5500
    * w5200.h
          * In lines 315-339, delete or comment out all the #ifdef structures and leave only the following as active code:  
              `inline static void initSS()    { pinMode(SS, OUTPUT); \`  
              `                                 digitalWrite(SS, HIGH); };`  
              `inline static void setSS()     { digitalWrite(SS, LOW); };`  
              `inline static void resetSS()   { digitalWrite(SS, HIGH); };`  
    * w5200.cpp
          * After the following line:  
              `#define SPI_CS 10`
              * Add these lines:  
                  `#define CS 10`  
                  `#define ARDUINO_ARCH_AVR`  
          * After the following lines:  
                `#if defined(ARDUINO_ARCH_AVR)`  
                `SPI.begin();`  
                `initSS();`
              * Add this line  
                  `SPI.setClockDivider(SPI_CLOCK_DIV8);`
          * In the second-to-last line of the file (right before the final #endif), add the following  
                  `#undef ARDUINO_ARCH_AVR`

### Notes on the library ###

* While the file w5200.cpp defines a symbol "SPI_CS", the actual symbol used in the code as the SPI chip select for the W5200 chip is actually just called "SS" (see w5200.h).
* A better approach would be to require that a symbol called "W5200_CS" be defined in the application code (e.g. the constructor), not hardcoded in the library
    * This would allow the application to decide on the chip select pin, not hardcoded in the library
* The library currently hard codes the spi clock divider as DIV8. It may be possible to speed this up with additional testing.
