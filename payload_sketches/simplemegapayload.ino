/* 
 * AUTHOR: Lara Tobias-Tarsh (laratt@umich.edu)
 * LAST MODIFIED: 09/12/2022 - functionalised much of the code for readability. Removed redundancies. Added Kalman filter modes.
 * 
 * DESCRIPTION:
 * Code for Arduino Mega payload acting as a sim communications module for the 
 * Nano BLE Sense and the Botletics SIM7000 on the Hologram IOT network.
 * Collects GPS data every 3 minutes and saves this to the EEPROM memory. 
 * Texts all data back on request through the hologram REST API SMS systems.
 * Also allows for the remote control of the Nano 33 BLE Sense acting as a sensor payload.
 * 
 * USER INPUTS: GPS lock required for running, sms count for the number of sensor readings to be recieved.
 * 
 * For further information, including syntax for SMS commands, refer to the README in the GitLab repo found at
 * https://gitlab.eecs.umich.edu/laratt/balloon-payload . You must request access to this, which I can grant in 
 * accordance with the Michigan Engineering Honor Code on a case by case basis.
 * 
 */


//============================================================================================================================
// USER INPUTS
//============================================================================================================================
float smsCount = 1500; // make sure this matches SZ_ARRAY in the BLE Sense module!
float gpsSetup = 1; // bool for GPS data. TRUE value is 1. This will not allow the exit of the setup loop without a GPS lock.
const char* phoneNumber = "+17349343326";
//============================================================================================================================
// SIM 7000A SETUP
//============================================================================================================================

// DO NOT EDIT THIS

#include "BotleticsSIM7000.h" 
#include <EEPROM.h>
 
// #include "Botletics_modem.h" 

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

#define SIMCOM_7000

// For botletics SIM7000 shield
#define BOTLETICS_PWRKEY 6
#define RST 7
#define TX 10 // Microcontroller RX
#define RX 11 // Microcontroller TX


int LAST_SELECTED_EEPROM_ADDR = 0;
#include <SoftwareSerial.h>
SoftwareSerial modemSS = SoftwareSerial(TX, RX);

SoftwareSerial *modemSerial = &modemSS;


// Use this for 2G modules
#ifdef SIMCOM_2G
  Botletics_modem modem = Botletics_modem(RST);
  
// Use this one for 3G modules
#elif defined(SIMCOM_3G)
  Botletics_modem_3G modem = Botletics_modem_3G(RST);
  
// Use this one for LTE CAT-M/NB-IoT modules (like SIM7000)

#elif defined(SIMCOM_7000) || defined(SIMCOM_7070) || defined(SIMCOM_7500) || defined(SIMCOM_7600)
Botletics_modem_LTE modem = Botletics_modem_LTE();
#endif

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
char imei[16] = {0};
float latitude, longitude, speed_kph, heading, altitude, second;
float init_latitude, init_longitude, init_speed_kph, init_heading, init_altitude; // used for testing if the balloon is ascending & calibration in post processing
uint16_t year;
uint8_t month, day, hour, minute;
uint8_t counter = 0;
char latBuff[12], longBuff[12], locBuff[50], speedBuff[12],
     headBuff[12], altBuff[12];

//============================================================================================================================
// SERIAL COMMUNICATIONS
//============================================================================================================================
const byte numChars = 100;
char receivedChars[numChars];
char tempChars[numChars];    // temporary array for use when parsing

// variables recieved from Arduino Nano BLE Sense
char BLE_Dict[numChars] = {0};

boolean newData = false;
float kalmanStatus = 0; // bool for kalman filter control. TRUE value is 1. This turns the Kalman filter on the BLE Sense on.
//============================================================================================================================
// SMS
//============================================================================================================================
  
char modemNotificationBuffer[64];  //for notifications from the modem
char smsBuffer[250];
char callerIDbuffer[32];  //we'll store the SMS sender number in here

int slot = 0;            //this will be the slot number of the SMS

//============================================================================================================================
// SETUP TASKS
//============================================================================================================================
void setup() {

  Serial2.begin(9600);
  pinMode(RST, OUTPUT);
  digitalWrite(RST, HIGH); // Default state
  
  // Turn on the module by pulsing PWRKEY low for a little bit
  // This amount of time depends on the specific module that's used
  modem.powerOn(BOTLETICS_PWRKEY); // Power on the module

  Serial.begin(9600);
  Serial.println(F("SMS Response Test"));
  Serial.println(F("Initializing...."));

  // SIM7000 takes about 3s to turn on
  // Press Arduino reset button if the module is still turning on and the board doesn't find it.
  // When the module is on it should communicate right after pressing reset

  // Software serial:
  modemSS.begin(115200); // Default SIM7000 shield baud rate

  Serial.println(F("Configuring to 9600 baud"));
  modemSS.println("AT+IPR=9600"); // Set baud rate
  delay(100); // Short pause to let the command run
  modemSS.begin(9600);
  if (! modem.begin(modemSS)) {
    Serial.println(F("Couldn't find modem"));
    while (1); // Don't proceed if it couldn't find the device
  }

  type = modem.type();
  Serial.println(F("Modem is OK"));
  Serial.print(F("Found "));
  switch (type) {
    case SIM800L:
      Serial.println(F("SIM800L")); break;
    case SIM800H:
      Serial.println(F("SIM800H")); break;
    case SIM808_V1:
      Serial.println(F("SIM808 (v1)")); break;
    case SIM808_V2:
      Serial.println(F("SIM808 (v2)")); break;
    case SIM5320A:
      Serial.println(F("SIM5320A (American)")); break;
    case SIM5320E:
      Serial.println(F("SIM5320E (European)")); break;
    case SIM7000:
      Serial.println(F("SIM7000")); break;
    case SIM7070:
      Serial.println(F("SIM7070")); break;
    case SIM7500:
      Serial.println(F("SIM7500")); break;
    case SIM7600:
      Serial.println(F("SIM7600")); break;
    default:
      Serial.println(F("???")); break;
  }
  
  // Print module IMEI number.
  uint8_t imeiLen = modem.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: "); Serial.println(imei);
  }

  // Set modem to full functionality
  modem.setFunctionality(1); // AT+CFUN=1
  modem.setNetworkSettings(F("hologram")); // For Hologram SIM card

  while (!modem.enableGPS(true)) {
    Serial.println(F("Failed to turn on GPS, retrying..."));
    delay(2000); // Retry every 2s
  }
  Serial.println(F("Turned on GPS!"));

  // Set the network status LED blinking pattern while connected to a network (see AT+SLEDS command)
  modem.setNetLED(true, 2, 64, 300); // on/off, mode, timer_on, timer_off
  


  modemSerial->print("AT+CNMI=2,1\r\n");  // Set up the modem to send a +CMTI notification when an SMS is received

  Serial.println("Modem Ready");

  // Get initial positions to allow post processing calibration. This also prevents us from starting without a GPS lock.
  // also acts as a test of initialisation. To turn off, set gpsSetup to 0.
}


//============================================================================================================================
static const uint32_t DELAY_1_S      = 1000UL;
static const uint32_t DELAY_1_MINUTE = DELAY_1_S * 60UL;
unsigned long lastMillis; // allow GPS recording every 1 mins only
float lastlat = init_latitude;
float lastlon = init_longitude;
float notmoving = 0;
float ctr = smsCount;
//============================================================================================================================


void loop() {
  char* bufPtr = modemNotificationBuffer;    //handy buffer pointer

  if (modem.available())      //any data available from the modem?
  {
    int charCount = 0;
    //Read the notification into modemInBuffer
    do  {
      *bufPtr = modem.read();
      Serial.write(*bufPtr);
      delay(1);
    } while ((*bufPtr++ != '\n') && (modem.available()) && (++charCount < (sizeof(modemNotificationBuffer)-1)));
    
    //Add a terminal NULL to the notification string
    *bufPtr = 0;

    //Scan the notification string for an SMS received notification.
    //  If it's an SMS message, we'll get the slot number in 'slot'
    if (1 == sscanf(modemNotificationBuffer, "+CMTI: " MODEM_PREF_SMS_STORAGE ",%d", &slot)) {
      Serial.print("slot: "); Serial.println(slot);
            
      // Retrieve SMS sender address/phone number.
      if (! modem.getSMSSender(slot, callerIDbuffer, 31)) {
        Serial.println("Didn't find SMS message in slot!");
      }
      Serial.print(F("FROM: ")); Serial.println(callerIDbuffer);
      
      // Retrieve SMS value.
      uint16_t smslen;
      if (modem.readSMS(slot, smsBuffer, 250, &smslen)) { // pass in buffer and max len!
        Serial.println(smsBuffer); 
               
        const char* begins = "b";       // begin sensor recording task
        const char* comms = "c";        // begin communication task
        const char* erase = "e";        // erase flash memory
        const char* reset = "n";        // reset commsavail
        const char* commsreset = "n";   // reset comms
        const char* commsstatus  = "a"; // trigger comms status LED on the BLE
        const char* kalmanOn = "k";     // turn BLE's inbuilt kalman filter on
        const char* kalmanOff = "x";    // turn BLE's inbuilt kalman filter off
        const char* findme = "find me"; // find the payload
        const char* init = "i";
     // make sure all SMS messages you send are exact or nothing will happen!


       // Perform actions based on sms commands
        
        if (strcmp(smsBuffer, begins) == 0) {
          Serial.print("STARTING ARDUINO");
          Serial2.write('b'); // begins BLE data recording protocol
          if (modem.deleteSMS(slot)) {
          Serial.println(F("OK!"));
           } else {
          Serial.print(F("Couldn't delete SMS in slot ")); Serial.println(slot);
          modem.print(F("AT+CMGD=?\r\n"));
          }
          
        }
        
        else if (strcmp(smsBuffer, comms) == 0) {
          Serial2.write('c'); // initiates serial communication task with BLE, then sends data via SMS
          if (modem.deleteSMS(slot)) {
          Serial.println(F("OK!"));
          delay(1000);
          Serial.print("Starting communications");
          // keeps scanning until the specified number of SMS needed has been sent
          while (ctr != 0) { //ctr is modified in recWithStartEndMarkers() function, ctr-- when newData == True.
            serialParse();
          }
           } else {
          Serial.print(F("Couldn't delete SMS in slot ")); Serial.println(slot);
          modem.print(F("AT+CMGD=?\r\n"));
          }
        }
        
        else if (strcmp(smsBuffer, erase) == 0) {
          Serial2.write('e'); // erases flash arrays from BLE 
         if (modem.deleteSMS(slot)) {
          Serial.println(F("OK!"));
           } else {
          Serial.print(F("Couldn't delete SMS in slot ")); Serial.println(slot);
          modem.print(F("AT+CMGD=?\r\n"));
          }
        }
        
        else if (strcmp(smsBuffer, commsreset) == 0) {
          Serial2.write('n'); // resets commsavail on the BLE for serial comms
          if (modem.deleteSMS(slot)) {
          Serial.println(F("OK!"));
          ctr = smsCount;
           } else {
          Serial.print(F("Couldn't delete SMS in slot ")); Serial.println(slot);
          modem.print(F("AT+CMGD=?\r\n"));
          }
        }
        
        else if (strcmp(smsBuffer, commsstatus) == 0) {
          Serial2.write('a'); // triggers status LED response for serial comms
          if (modem.deleteSMS(slot)) {
          Serial.println(F("OK!"));
           } else {
          Serial.print(F("Couldn't delete SMS in slot ")); Serial.println(slot);
          modem.print(F("AT+CMGD=?\r\n"));
          }
        }
        reqGPS();

      }
    }
  }

  // allows erasing flash and resetting comms via serial monitor
    if (Serial.available()) {
    char c = Serial.read();
    c=tolower(c);
    if (c=='e') Serial2.write("e");
    Serial.println("done, erasing flash");
    if (c=='n') Serial2.write("n");
    Serial.println("commsavail reset. Good to go!");
    if (c=='m') clearSlot();
    
  }
}

// end of loop

//============================================================================================================================
// SMS FUNCTIONS
//============================================================================================================================

// Send an SMS response
void sendText(const char* textMessage) {
  Serial.println("Sending reponse...");
  
  if (!modem.sendSMS(callerIDbuffer, textMessage)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("Sent!"));
  }
  
  // Delete the original message after it is processed.
  // Otherwise we will fill up all the slots and
  // then we won't be able to receive any more!
  if (modem.deleteSMS(slot)) {
    Serial.println(F("OK!"));
  } else {
    Serial.print(F("Couldn't delete SMS in slot ")); Serial.println(slot);
    modem.print(F("AT+CMGD=?\r\n"));
  }
}

void sendTextPhone(const char* num, const char* textMessage) {
  Serial.println("Sending reponse...");
  
  if (!modem.sendSMS(num, textMessage)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("Sent!"));
  }
  
  // Delete the original message after it is processed.
  // Otherwise we will fill up all the slots and
  // then we won't be able to receive any more!
  if (modem.deleteSMS(slot)) {
    Serial.println(F("OK!"));
  } else {
    Serial.print(F("Couldn't delete SMS in slot ")); Serial.println(slot);
    modem.print(F("AT+CMGD=?\r\n"));
  }
}


//============================================================================================================================
// SERIAL COMMUNICATION FUNCTIONS
//============================================================================================================================

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial2.available() > 0 && newData == false) {
    rc = Serial2.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {

        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }

      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
        ctr-1;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}


void serialParse() {
  recvWithStartEndMarkers();
  if (newData == true) {
      strcpy(tempChars, receivedChars);
      parseData();
      newData = false;
      char sensorBuff[300];
              
  
      strcat(sensorBuff,BLE_Dict);
      sendText(sensorBuff);
      
     }
}


void parseData() {      // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, "!");     // get the first part - the string
  strcpy(BLE_Dict, strtokIndx);            // copy it to BLE_DICT

}


void showParsedData() {
  Serial.println(BLE_Dict);
}


//============================================================================================================================
// GPS FUNCTIONS
//============================================================================================================================

void gpsInit(){
// initialises GPS and will not allow exit of setup loop until there is a GPS lock.
  while (!modem.getGPS(&init_latitude, &init_longitude, &init_speed_kph, &init_heading, &init_altitude)) {
            Serial.println(F("Failed to get GPS location, retrying..."));
            delay(2000); // Retry every 2s
            sendTextPhone(phoneNumber, "establishing lock\0");
          }
          
           
            dtostrf(init_latitude, 1, 6, latBuff); // float_val, min_width, digits_after_decimal, char_buffer
            dtostrf(init_longitude, 1, 6, longBuff);
            dtostrf(init_speed_kph, 1, 0, speedBuff);
            dtostrf(init_heading, 1, 0, headBuff);
            dtostrf(init_altitude, 1, 1, altBuff);
  
            sprintf(locBuff, "%s,%s,%s,%s", speedBuff, latBuff, longBuff, altBuff);
  
            char textMessage[400];
            memset(textMessage, 0, 400);
            memset(locBuff, 0, 50); 
            strcat(locBuff," Found initial GPS");
            
            strcat(textMessage,locBuff);
            sendTextPhone(phoneNumber,textMessage);
            EEPROM.write(LAST_SELECTED_EEPROM_ADDR, locBuff); // save to EEPROM
            LAST_SELECTED_EEPROM_ADDR++;
            modem.setNetLED(true, 3, 64, 3000);
            gpsSetup = 0;
  }



void reqGPS() {
// Gets GPS Data and writes to EEPROM to retrieve later
      modem.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
      dtostrf(latitude, 1, 6, latBuff); // float_val, min_width, digits_after_decimal, char_buffer
      dtostrf(longitude, 1, 6, longBuff);
      dtostrf(speed_kph, 1, 0, speedBuff);
      dtostrf(heading, 1, 0, headBuff);
      dtostrf(altitude, 1, 1, altBuff);

      sprintf(locBuff, "%s,%s,%s,%s", speedBuff, latBuff, longBuff, altBuff);

      EEPROM.write(LAST_SELECTED_EEPROM_ADDR, locBuff); // save to EEPROM
      LAST_SELECTED_EEPROM_ADDR++;
      Serial.print(locBuff);
 }

void findlanding () {
// Turns on GPS if disabled somehow, after 15 minutes with no movement will text landing coordinates.
  #ifdef turnOffShield
        while (!modem.enableGPS(true)) {
          Serial.println(F("Failed to turn on GPS, retrying..."));
          delay(2000); // Retry every 2s
        }
          Serial.println(F("Turned on GPS!"));
        #endif
          
        while (!modem.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude)) {
          Serial.println(F("Failed to get GPS location, retrying..."));
          delay(2000); // Retry every 2s
        }
          dtostrf(latitude, 1, 6, latBuff); // float_val, min_width, digits_after_decimal, char_buffer
          dtostrf(longitude, 1, 6, longBuff);
          dtostrf(speed_kph, 1, 0, speedBuff);
          dtostrf(heading, 1, 0, headBuff);
          dtostrf(altitude, 1, 1, altBuff);

          sprintf(locBuff, "%s,%s,%s,%s", speedBuff, latBuff, longBuff, altBuff);

          char textMessage[400]; // Make sure this is long enough!
          char sensorBuff[400];
          strcat(sensorBuff, "FLIGHT COMPLETE. FINDING PAYLOAD...");
          sendTextPhone(phoneNumber,sensorBuff);

          strcat(textMessage,locBuff);
          sendTextPhone(phoneNumber,textMessage);
          memset(locBuff, 0, 50);
          memset(textMessage, 0, 400);
          memset(sensorBuff, 0, 400);
          
          
}


 void checkPosition() {
  // record GPS every 1 mins, checks to see if the ballon has stopped moving.
    
        if (millis() - lastMillis >= DELAY_1_MINUTE ) // record and save GPS every 3 mins
        {
         if (!modem.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude)) {
              return;
          }
         lastMillis = millis();  //get ready for the next iteration
         reqGPS();
         
         // Check location of the payload - has it stopped moving?
         if ( latitude==lastlat && longitude == lastlon) {
            notmoving++;
            return;
         }
         else {
            lastlat = latitude;
            lastlon = longitude;
            notmoving = 0; // reset landing monitor
         }
        
      
        if (notmoving == 5) { // not moved for 15 minutes
          findlanding();
          return;
         }  
   }
   else {
    return;
   }
 }

void clearSlot() {
  if (modem.deleteSMS(slot)) {
          Serial.println(F("OK!"));
  }
}
