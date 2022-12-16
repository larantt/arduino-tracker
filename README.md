# balloon-payload
Arduino meteorological sonde using Arduino BLE 33 Sense, Arduino Mega 2650 &amp; Botletics Sim7000A. 

## Libraries
This project relies on multiple libraries to function, including some custom libraries.
I have linked all of the libraries needed to compile the code for each component below.
Note that these are open source and thus they may not be up to date at the time you download this.
This was a class project and is still under development. I will likely not maintain this regularly once
I have a printed PCB, so functionality could become depreciated. 
If you notice this, open an issue and I will go back and fix it when I have the time.

* Arduino Nano 33 BLE Sense
    - Sensor libraries can be found in the Arduino IDE library manager. You need:
        - Arduino_HTS221
        - Arduino_LPS22HB
        - Arduino_LSM9DS1
    - [Flash Memory](https://gitlab.eecs.umich.edu/deroo/nano33ble-flash)
    - [Simple Kalman Filter](https://github.com/ashishverma2614/SimpleKalmanFilter)

* Arduino Mega 2650:
    - Arduino IDE included libraries:
        - SoftwareSerial
        - EEPROM
        - Wire
    - [Botletics Shield](https://github.com/botletics/Botletics-SIM7000/tree/main/src)
    - [MCP9808 Temp Sensor](https://github.com/adafruit/Adafruit_MCP9808_Library)
Note that the Botletics SIM7000A is based on the Adafruit FONA library, so make sure you
check the repo for the most up to date version of this, or your board will *not* work.

For hardware debugging, I recommend that you use the Botletics SIM7000 LTE example
as this allows you to test each component through serial commands. One of the most common
issues is that the sim slot gets filled and prevents you from sending or recieveing an SMS.
This can also be cleared using the `m` command in the serial monitor from either of the Mega
payload sketches.

A .yaml file for the python post-processing script can be found in the main repository. I
use anaconda for everything so this is a clone of my Conda environment. 

To reproduce my environment, you can clone this file then run the command
`conda env create -f environment.yml`


You will at a minimum need the Arduino IDE to run this project, or at least you need to configure
your text editor to be able to interface with the Serial Monitor.

## Payload Operations
The payload_sketches directory contains the two files needed to run the Arduino balloon payload. 
These are relatively self explanatory and I believe are adequately commented. They are also highly
customisable, just by following the same syntax.

The full software is contained in the MegaPayload.ino sketch, however a less complex version of 
the sonde can be found in the simpleMegaPayload.ino. If you wish to add additonal sensors, I
recommend you add these to the BLE Sense for code simplicity. 

## Post Processing
This is still under development, as it needs cleaning up. There are currently multiple Kalman filters
and plotting routines in random places, and documentation is poor. Soon this will contain a function
module and a main script containing examples.

I have provided sample data, courtesy of Eric Andrechek's sounding launch from the University of Michigan's
ENGR100 section. This is used to demonstrate the functionality of each of these plotting routines.


