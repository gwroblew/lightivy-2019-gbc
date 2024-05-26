# Light Ivy Firmware Repository

## Flash Memory layout

### boot.bin

 - 40200000  text (two iram sections), rodata

### firmware.bin

 - 4020C000  text, data, rodata
 - 40216000  irom0
 - 40257000  config

### oc.bin

 - 40258000  irom0

### Working code memory

 - 40280000  games / apps

### Preloaded content

 - 40300000  assets / files / etc.

### System configuration

 - 405F8000  clock / calibration data

## Compiling and flashing

"make" should compile given binary. "make full_flash" performs clean flashing over the serial interface.
"make flash" performs incremental flash, which sometimes does not work properly.
"esp-terminal" under "esp-utils" opens simple terminal, for an example "./esp-terminal /dev/ttyUSB0".
Speed for flashing and terminal is hardcoded to 921600.

## Coding Guidelines

 - All functions must have ROCODE attribute to be placed in flash.
 - All constant data, including strings, must have RODATA.
 - Size of the stack is very limited, avoid using stack.
 - Also avoid using RAM.

## Notes

Boot.bin is based on Arduino eboot bootloader code. Holding Y button (closest to LCD on the right side) while powering up
initiates "forced" update - re-flashing files that are present on the SD card.

Firmware.bin uses split IRAM mode (half of IRAM is used for system heap). It reserves largest possible block for use by oc.bin.
Currently it is a static 40kB block in DRAM (allocated in bss) and dynamic IRAM block of 12kB.
This leaves about 26kB for system, which drops to 12kB after connecting to a WiFi station.
If needed, the size of the fixed block should be adjusted to leave more memory for network operations.

Oc.bin uses additional 2kB of DRAM at 0x3FFFE000 (right below the stack area), based on https://bbs.espressif.com/viewtopic.php?t=8879.
To place variables in this section use RWDATA macro.

Serial debug output on TXD pin 16 can be used in general, but not at the same time when SD card transmission happens. SD card operation
and buttons reading might generate noise on debug output.

### Pin assignment:

1. RST - only on power up
2. ADC - battery level: actual value / 4
3. EN
4. GPIO16 LCD_CS BUTTONS_LOAD
5. GPIO14 HSPI_CLK LCD_CLK SD_CLK RAM_CLK
6. GPIO12 HSPI_MISO SD_MISO RAM_MISO
7. GPIO13 HSPI_MOSI LCD_DATA SD_MOSI RAM_MOSI
8. VCC
9. GND
10. GPIO15 HSPI_CS I2SO_BCK - audio clock
11. GPIO2 I2SO_WS - audio channels
12. GPIO0 LCD_RS BUTTON8 SD_POWER
13. GPIO4 RAM_CS
14. GPIO5 BUTTONS_DATA 
15. RXD ESP_RX I2SO_DATA - audio data
16. TXD ESP_TX SD_CS BUTTONS_CLK

### Useful commands to test UDP broadcast

echo test | socat - UDP-DATAGRAM:192.168.1.255:5555,broadcast

socat udp-recvfrom:5555,broadcast,fork -
