
#include "BotleticsSIM7000.h" 
#include <EEPROM.h>
 
// #include "Botletics_modem.h" 

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
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
char imei[16] = {0};// MUST use a 16 character buffer for IMEI!
float latitude, longitude, speed_kph, heading, altitude, second;
float init_latitude, init_longitude, init_speed_kph, init_heading, init_altitude; // used for testing if the balloon is ascending & calibration in post processing
uint16_t year;
uint8_t month, day, hour, minute;
uint8_t counter = 0;
char latBuff[12], longBuff[12], locBuff[50], speedBuff[12],
     headBuff[12], altBuff[12];
int ctr = 10;
//============================================================================================================================
// SERIAL COMMUNICATIONS
//============================================================================================================================
const byte numChars = 100;
char receivedChars[numChars];
char tempChars[numChars];    // temporary array for use when parsing

// variables recieved from Arduino Nano BLE Sense
char BLE_Dict[numChars] = {0};

boolean newData = false;

  
char modemNotificationBuffer[64];  //for notifications from the modem
char smsBuffer[250];
char callerIDbuffer[32];  //we'll store the SMS sender number in here

int slot = 0;            //this will be the slot number of the SMS

//============================================================================================================================
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
  modem.setNetLED(true, 2, 64, 3000); // on/off, mode, timer_on, timer_off
  


  modemSerial->print("AT+CNMI=2,1\r\n");  // Set up the modem to send a +CMTI notification when an SMS is received

  Serial.println("Modem Ready");

  // Get initial positions to allow post processing calibration. This also prevents us from starting without a GPS lock.
  // also acts as a test of initialisation
  while (!modem.getGPS(&init_latitude, &init_longitude, &init_speed_kph, &init_heading, &init_altitude)) {
          Serial.println(F("Failed to get GPS location, retrying..."));
          delay(2000); // Retry every 2s
        }
          dtostrf(init_latitude, 1, 6, latBuff); // float_val, min_width, digits_after_decimal, char_buffer
          dtostrf(init_longitude, 1, 6, longBuff);
          dtostrf(init_speed_kph, 1, 0, speedBuff);
          dtostrf(init_heading, 1, 0, headBuff);
          dtostrf(init_altitude, 1, 1, altBuff);

          sprintf(locBuff, "%s,%s,%s,%s", speedBuff, latBuff, longBuff, altBuff);

          char textMessage[400]; 
          strcat(locBuff," initial GPS");

          strcat(textMessage,locBuff);
          sendText(textMessage);
          EEPROM.write(LAST_SELECTED_EEPROM_ADDR, locBuff); // save to EEPROM
          LAST_SELECTED_EEPROM_ADDR++;

}
//============================================================================================================================
static const uint32_t DELAY_1_S      = 1000UL;
static const uint32_t DELAY_1_MINUTE = DELAY_1_S * 60UL;
unsigned long lastMillis; // allow GPS recording every 5 mins only
//============================================================================================================================
void loop() {
  
  char* bufPtr = modemNotificationBuffer;    //handy buffer pointer

  //is the time up for this task?
  if (millis() - lastMillis >= DELAY_1_MINUTE * 3) // record and save GPS every 3 mins
  {
   lastMillis = millis();  //get ready for the next iteration
   reqGPS();
  }
  
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
        const char* begins = "b"; 
        const char* comms = "c";
        const char* erase = "e";
        const char* reset = "n";
        const char* commsreset = "n";
        const char* commsstatus  = "a";
        const char* findme = "find me";
        
        if (strcmp(smsBuffer, begins) == 0) {
          Serial.print("STARTING ARDUINO");
          Serial2.write('b');
          // then we won't be able to receive any more!
          if (modem.deleteSMS(slot)) {
          Serial.println(F("OK!"));
           } else {
          Serial.print(F("Couldn't delete SMS in slot ")); Serial.println(slot);
          modem.print(F("AT+CMGD=?\r\n"));
          }
          
        }
        
        else if (strcmp(smsBuffer, comms) == 0) {
          // Do stuff like digitalWrite(door, HIGH)
          Serial2.write('c');
          if (modem.deleteSMS(slot)) {
          Serial.println(F("OK!"));
           } else {
          Serial.print(F("Couldn't delete SMS in slot ")); Serial.println(slot);
          modem.print(F("AT+CMGD=?\r\n"));
          }
          delay(1000);
          Serial.print("Starting communications");
          
          while (ctr != 0) {
            recvWithStartEndMarkers();
          if (newData == true) {
            strcpy(tempChars, receivedChars);
            parseData();
            newData = false;
            char sensorBuff[300];
            

            strcat(sensorBuff,BLE_Dict);
            sendText(sensorBuff);
            }
            ctr-1;
           
          }
          
        }
        
        else if (strcmp(smsBuffer, erase) == 0) {
          // Do stuff like digitalWrite(door, HIGH)
          Serial2.write('e');
         if (modem.deleteSMS(slot)) {
          Serial.println(F("OK!"));
           } else {
          Serial.print(F("Couldn't delete SMS in slot ")); Serial.println(slot);
          modem.print(F("AT+CMGD=?\r\n"));
          }
        }
        
        else if (strcmp(smsBuffer, commsreset) == 0) {
          // Do stuff like digitalWrite(door, HIGH)
          Serial2.write('n');
          if (modem.deleteSMS(slot)) {
          Serial.println(F("OK!"));
           } else {
          Serial.print(F("Couldn't delete SMS in slot ")); Serial.println(slot);
          modem.print(F("AT+CMGD=?\r\n"));
          }
        }
        
        else if (strcmp(smsBuffer, commsstatus) == 0) {
          // Do stuff like digitalWrite(door, HIGH)
          Serial2.write('a');
          if (modem.deleteSMS(slot)) {
          Serial.println(F("OK!"));
           } else {
          Serial.print(F("Couldn't delete SMS in slot ")); Serial.println(slot);
          modem.print(F("AT+CMGD=?\r\n"));
          }
        }
        
        else if (strcmp(smsBuffer, findme) == 0) {
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
          //char sensorBuff[300];

          strcat(textMessage,locBuff);
          sendText(textMessage);

        }
      }
    }
  }
}

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
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}
//============================================================================================================================

void parseData() {      // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, "!");     // get the first part - the string
  strcpy(BLE_Dict, strtokIndx);            // copy it to BLE_DICT

}

//============================================================================================================================

void showParsedData() {
  Serial.println(BLE_Dict);
}

//============================================================================================================================
// Gets GPS Data and writes to EEPROM to retrieve later
void reqGPS() {
while (!modem.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude)) {
          Serial.println(F("Failed to get GPS location, retrying..."));
          delay(2000); // Retry every 2s
        }
          dtostrf(init_latitude, 1, 6, latBuff); // float_val, min_width, digits_after_decimal, char_buffer
          dtostrf(init_longitude, 1, 6, longBuff);
          dtostrf(init_speed_kph, 1, 0, speedBuff);
          dtostrf(init_heading, 1, 0, headBuff);
          dtostrf(init_altitude, 1, 1, altBuff);

          sprintf(locBuff, "%s,%s,%s,%s", speedBuff, latBuff, longBuff, altBuff);

          EEPROM.write(LAST_SELECTED_EEPROM_ADDR, locBuff); // save to EEPROM
          LAST_SELECTED_EEPROM_ADDR++;

}
