# matrixclock
8x8xN matrix clock with scroll and effects

Raspberry Pi 3 Model B Pinout
=============================

http://pi4j.com/pins/model-3b-plus-rev1.html

Power
-----

```
 3.3 VDC - 01  02 - 5.0 VDC
               04 - 5.0 VDC
               06 - GND
           07
     GND - 09  
           11  
           13  14 - GND
```

UART
----

```
               06 - GND
               08 - TxD (to CP2102 TXD)
               10 - RxD (to CP2102 RXD)
```

SPI
---

```
MOSI - 19  20 - GND
MISO - 21
SCLK - 23  24 - CE0
 GND - 25  26 - CE1
```


MAX7219
=======

