# WizNet Ethernet Library For MSP430

The SeeedStudio Ethernet Shield used in the Wireless Sensor Receiver Hub
project uses a W5200 ethernet chip from WIZnet.

The WIZnet-provided Arduino library for the W5200 chip requires some changes
to work with MSP430 controllers.

Additional changes are also required to fix a memory leak in sketches where Ethernet.begin() is run more than once.

## Installing and Modifying the WIZnet Ethernet Library for MSP430

1. Download [library ZIP][1] from GitHub.
2. Navigate to "WIZ_Ethernet_Library-master/Arduino IDE 1.5.x" inside the ZIP folder.
3. Copy the "Ethernet" folder to your libraries folder inside your sketch folder.
4. Navigate to "WIZ_Ethernet_Library-master/Arduino IDE 1.0.x/Ethernet" inside the ZIP folder.
5. Copy the entire Examples folder as-is to the Ethernet folder which was copied to the libraries folder above.
6. Download and extract [additional files][2] from Wiznet.
7. Copy the individual "additional files" into the "src" folder of the Ethernet library you copied above.

    Note: "IPAddress.h" and "IPAddress.cpp" may not be needed and may cause a conflict when compiling. If you get an error related to "IPAddress" when compiling, then delete these two files.

8. Modify the following files in the new Ethernet/src/utility folder

* w5100.h  
  * Change the the following lines from:

    `//#define W5200_ETHERNET_SHIELD`  
    `#define W5500_ETHERNET_SHIELD`

  * To:

    `#define W5200_ETHERNET_SHIELD`  
    `//#define W5500_ETHERNET_SHIELD`

    That is, uncomment the W5200 definition, and comment out W5500

* w5200.h
  * In lines 315-339, delete or comment out all the #ifdef structures and leave only the following as active code:  

    ```cpp
      inline static void initSS()    { pinMode(SS, OUTPUT); \
                                       digitalWrite(SS, HIGH); };
      inline static void setSS()     { digitalWrite(SS, LOW); };
      inline static void resetSS()   { digitalWrite(SS, HIGH); };
    ```

* w5200.cpp
  * Change the  line:  
    `#define SPI_CS 10`
  * To:  
    `#define SPI_CS 8`  
    `#define ARDUINO_ARCH_AVR`  
  * After the following lines:  
    `#if defined(ARDUINO_ARCH_AVR)`  
    `SPI.begin();`  
    `initSS();`
  * Add this line  
    `SPI.setClockDivider(SPI_CLOCK_DIV8);`
  * In the second-to-last line of the file (right before the final #endif), add the following  
    `#undef ARDUINO_ARCH_AVR`

## Fixing the Memory Leak

The `Ethernet.begin()` method creates a new `DhcpClass` object every time it is run, without first checking if a `DchpClass` object already exists. This causes a memory leak in sketches where `Ethernet.begin()` is called more than once, since the previous `DhcpClass` object still exists but is no longer used (or reachable).

To fix this, edit the following files:

### Ethernet.h

Add a constructor prototype in the public section:  

```cpp
EthernetClass();
```

### Ethernet.cpp

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

Additional checks can be added to make sure that `_dhcp` is non-null before using the pointer.

## Notes on the library

* While the file w5200.cpp defines a symbol "SPI_CS", other parts of the code use the internally-defined "SS" symbol as the SPI chip select.
* A better approach would be to use a single symbol and require that its value be defined in the application code (e.g. the constructor), not hardcoded in the library. This would allow the application to decide on the chip select pin, not hardcoded in the library
* The library currently hard codes the spi clock divider as DIV8. It may be possible to speed this up with additional testing.

[1]: https://github.com/Wiznet/WIZ_Ethernet_Library
[2]: http://wizwiki.net/wiki/lib/exe/fetch.php?media=osh:ioshield-l:updatelib:additional_files_from_arduino.zip
[200]: https://github.com/Andy4495/Wireless-Sensor-Receiver-Hub
