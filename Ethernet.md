# WizNet Ethernet Library For MSP430

A [specific updated verson][4] of the Ethernet library is needed to work with Wireless Sensor Receiver Hub.

The updated version contains changes to work specifically with the W5200-based SeeedStudio Ethernet Shield used in the Wireless Sensor Receiver Hub. In addition some source code changes were needed to work with the MSP430 compiler and to fix a memory leak.

## Modifications to Official WIZnet Ethernet Library

* w5100.h  
  Change the the following lines from:

  ```cpp
  //#define W5200_ETHERNET_SHIELD 
  #define W5500_ETHERNET_SHIELD
  ```

  To:

  ```cpp
  #define W5200_ETHERNET_SHIELD 
  //#define W5500_ETHERNET_SHIELD
  ```

  That is, uncomment the W5200 definition, and comment out W5500

* w5200.h

  In lines 315-339, delete or comment out all the #ifdef structures and leave only the following as active code:  

  ```cpp
  inline static void initSS()    { pinMode(SS, OUTPUT); \
                                   digitalWrite(SS, HIGH); };
  inline static void setSS()     { digitalWrite(SS, LOW); };
  inline static void resetSS()   { digitalWrite(SS, HIGH); };
  ```

* w5200.cpp
  
  1. Change the  line:  

      ```cpp
      #define SPI_CS 10
      ```

      To:  

      ```cpp
      #define SPI_CS 8
      #define ARDUINO_ARCH_AVR 
      ```

  2. After the following lines:  

      ```cpp
      #if defined(ARDUINO_ARCH_AVR)
      SPI.begin();
      initSS();
      ```

      Add this line  

      ```cpp
      SPI.setClockDivider(SPI_CLOCK_DIV8);
      ```

  3. In the second-to-last line of the file (right before the final #endif), add the following  

      ```cpp
      #undef ARDUINO_ARCH_AVR
      ```

* Ethernet.h

  Add a constructor prototype in the public section:  

  ```cpp
  EthernetClass();
  ```

* Ethernet.cpp

  Define the constructor, which initializes the `_dhcp` pointer to zero:

  ```cpp
  EthernetClass::EthernetClass() {
      _dhcp = 0;
  }
  ```

  In the two places where a new `DhcpClass` object is created, replace this:

  ```cpp
  _dhcp = new DhcpClass();
  ```

  with this (to check if an object was already created):

  ```cpp
  if (_dhcp == 0) {
      _dhcp = new DhcpClass();
  }
  ```

* Added file `Udp.h` from the standard Arduino Ethernet library.

## Notes on the library

* While the file w5200.cpp defines a symbol `SPI_CS`, other parts of the code use the internally-defined `SS` symbol as the SPI chip select.
* A better approach would be to use a single symbol and require that its value be defined in the application code (e.g. the constructor), not hardcoded in the library. This would allow the application to decide on the chip select pin, not hardcoded in the library
* The library currently hard codes the spi clock divider as DIV8. It may be possible to speed this up with additional testing.

[1]: https://github.com/Wiznet/WIZ_Ethernet_Library
[3]: https://github.com/Seeed-Studio/Ethernet_Shield_W5200
[4]: https://github.com/Andy4495/WIZ_Ethernet_Library
[200]: https://github.com/Andy4495/Wireless-Sensor-Receiver-Hub

[//]: # (Dead link - previously pointed to "additional files" needed for Ethernet library: http://wizwiki.net/wiki/lib/exe/fetch.php?media=osh:ioshield-l:updatelib:additional_files_from_arduino.zip)
