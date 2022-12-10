# balloon-payload
Arduino meteorological sonde using Arduino BLE 33 Sense, Arduino Mega 2650 &amp; Botletics Sim7000A. 

## Libraries
This project relies on multiple libraries to function, including some custom libraries.
I have linked all of the libraries needed to compile the code for each component below.
Note that these are open source and thus they may not be up to date at the time you download this.
This was a class project. I will likely not maintain this regularly, so functionality could become depreciated. 
If you notice this, open an issue and I will go back and fix it when I have the time.

* Arduino Nano 33 BLE Sense
    - Sensor libraries can be found in the Arduino IDE library manager. You need:
        - Arduino_HTS221
        - Arduino_LPS22HB
        - Arduino_LSM9DS1
    - [Flash Memory](https://gitlab.eecs.umich.edu/deroo/nano33ble-flash)
    - [Simple Kalman Filter] (https://github.com/ashishverma2614/SimpleKalmanFilter)

* Arduino Mega 2650:
    - Arduino IDE included libraries:
        - SoftwareSerial
        - EEPROM
        - Wire
    - Botletics Shield: https://github.com/botletics/Botletics-SIM7000/tree/main/src
    - MCP9808 Temp Sensor: https://github.com/adafruit/Adafruit_MCP9808_Library
Note that the Botletics SIM7000A is based on the Adafruit FONA library, so make sure you
check the repo for the most up to date version of this, or your board will *not* work.

A yml file for the python post-processing script can be found in the 
