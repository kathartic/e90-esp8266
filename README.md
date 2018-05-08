# e90-esp8266
Code for the ESP8266 side of the E90.

This project relies heavily on the following libraries:
* [Miguel Balboa's MFRC522 library](https://github.com/miguelbalboa/rfid)
* [knolleary's MQTT library](https://github.com/knolleary/pubsubclient)
* [ESP8266 WiFi library](http://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/readme.html)

To run this script, you may need to install additional drivers. 

Note that this code requires an associated C header file named "Secrets.h", with the following syntax

```C
#if !defined(SECRETS)
#define SECRETS 1

const char* ssid = "<ssid-name>";
const char* password = "<ssid-password>";
const int ip[4] = {<block0>, <block1>, <block2>, <block3>};

#endif
```
