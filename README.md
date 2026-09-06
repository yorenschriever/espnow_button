# ESPNOW button
This project consist of 2 parts. One runs on an esp32s3 with a button connected gpio3. It waits in deep sleep until the button changes value, and then broadcasts it over esp now.
The other part is a receiver that acts as a midi usb device that emits the button pressed as not on/off signals.

# Programming
Both - sender and receiver - can be programmed over the otg port. Before programming you need to connect the port bootloader by holding down BOOT and then pressing reset. 
The sender is likely in deep sleep otherwise, and wont listen for your flash command. The receiver is acting a midi device and also wont listen.

# Primary master key

Add a file named `common/pmk.h` with a 16 byte master key, like this:
```
#define CONFIG_ESPNOW_PMK "F9x5WYR^VByLPyA5" 
```

# Hardware
I used esp32-s3-zero modules this project. 

On the sender side (button), make sure to debounce the button with a 4k7 resistor and 100nf cap, otherwise the bounce noise might trigger and error when going in deep sleep. This hardware debounce is still necessary on top of the software debounce.

Also make sure to remove the WS2812B led from the module. It consumes the most power, and will quickly drain the battery, even when off.

The 3v3 pin can be directly connected to a LiFePo4 battery (with a BMS in between). No step down module required.

For testing i used the ESP32-S3-DevKitC, because uart USB connector would remain connected to the log output, even when the receiver is in USB host mode, or the sender is in deep sleep mode.

## vapes
For the vapes i used the esp32s3 super mini.
This one is more power efficient. it also comes with built-in charger. to increase the charging current from 100mA to 300mA, connect the boost jumper on the back (close to the batt solder pads)

Remove the RGB led and also a resistor est to the tx pin
also i soldered a 32cm wire to the pcb antenna (board edge side) to improve the range