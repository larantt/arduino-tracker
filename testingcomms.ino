// Author: Lara Tobias-Tarsh
// Sketch communicates between Arduino Nano BLE 33 Sense and Arduino Mega 2560
// Shares temperature, pressure, humidity and acceleration data
// Read sensors, parses them to a string and sends them to the RX2 port on the Arduino Mega
// Saves readings in the flash memory of the Arduino Nano BLE 33 Sense for safety
// Starts recording either through serial monitor (or BLE input - coming soon)

// include libraries
#include <Arduino_HTS221.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_LSM9DS1.h>
#include <Nano33BLEflash.h>

//-------------------------------------------------------------------------------------------
// INITIALISATION
//-------------------------------------------------------------------------------------------
#define SZ_ARRAY 24
#define RED 22
#define BLUE 24
#define GREEN 23
#define LED_PWR 25

// Declare flash arrays
NANO33BLE_DECLARE(float, pressArray)
NANO33BLE_DECLARE(float, humidArray)
NANO33BLE_DECLARE(float, tempArray)
NANO33BLE_DECLARE(float, accelArray_x)
NANO33BLE_DECLARE(float, accelArray_y)
NANO33BLE_DECLARE(float, accelArray_z)

// Declare communication string
char buf[300]; // character buffer
int commsavail = 1; // ensures flash is not overwritten
float countdownTime = 40000; // length of countdown before recording begins, change as needed

// declare sensor floats
float pressure;
float humidity;
float temps;
float accel_x;
float accel_y;
float accel_z;
float x,y,z;
int j = 0; // for communications indexing
  
//-------------------------------------------------------------------------------------------
// SENSOR FUNCTIONS
//-------------------------------------------------------------------------------------------
void sensortask(){
  // Read each sensor and write to flash memory
  for (int i = 0; i < SZ_ARRAY; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    flashMode(FLASH_WRITE);
    pressArray[i] = round(BARO.readPressure() * 1000);
    humidArray[i] = round(HTS.readHumidity());
    tempArray[i] = round(HTS.readTemperature());
    IMU.readAcceleration(x,y,z);
    accel_x = x*9.8066;
    accel_y = y*9.8066;
    accel_z = z*9.8066;
    accelArray_x[i] = accel_x;
    accelArray_y[i] = accel_y;
    accelArray_z[i] = accel_z;
    flashMode(FLASH_READONLY);
    digitalWrite(LED_BUILTIN, LOW);
    delay(600);
  }
  flashMode(FLASH_READONLY);
  digitalWrite(LED_BUILTIN, LOW);
}

void commstask() {
  // Parse sensor reading to csv and send to Mega, uses markers for mega reading ease
  digitalWrite(LED_BUILTIN, HIGH);
  char startMarker = '<';
  char endMarker = '>';
  char excl = '!';
  char comma = ',';
  snprintf(buf, sizeof(buf)-1, "%c%f%c%f%c%f%c%f%c%f%c%f%c%c", 
                              startMarker,pressArray[j], comma, humidArray[j], comma, tempArray[j], comma,
                              accelArray_x[j], comma, accelArray_y[j], comma, accelArray_x[j], excl,endMarker);

  j++;
  Serial1.write(buf,300);
  digitalWrite(LED_BUILTIN, LOW);
  delay(6000);
}

//-------------------------------------------------------------------------------------------
// MAIN EXECUTION
//-------------------------------------------------------------------------------------------
  void setup() {
    //commsavail = 1;
    // setup arduino pins
    pinMode(1, OUTPUT); 
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(RED, OUTPUT);
    pinMode(BLUE, OUTPUT);
    pinMode(GREEN, OUTPUT);
    pinMode(LED_PWR, OUTPUT);
    
    // begin sensors
    IMU.begin();
    BARO.begin();
    HTS.begin();

    // put arrays in flash
    NANO33BLE_PUT_ARRAY_IN_FLASH(float,pressArray,SZ_ARRAY);
    NANO33BLE_PUT_ARRAY_IN_FLASH(float,humidArray,SZ_ARRAY);
    NANO33BLE_PUT_ARRAY_IN_FLASH(float,tempArray,SZ_ARRAY);
    NANO33BLE_PUT_ARRAY_IN_FLASH(float,accelArray_x,SZ_ARRAY);
    NANO33BLE_PUT_ARRAY_IN_FLASH(float,accelArray_y,SZ_ARRAY);
    NANO33BLE_PUT_ARRAY_IN_FLASH(float,accelArray_z,SZ_ARRAY);

    delay( 10 );

    if ( !HTS.begin() ) {
      Serial.println( "Failed to initialize humidity temperature sensor!" );
      while ( 1 );
    }
  
    if ( !BARO.begin() ) {
      Serial.println( "Failed to initialize pressure sensor!" );
      while ( 1 );
    }

    // ensure flash is empty (ERASE BEFORE LAUNCH)
    Serial.begin(9600);
    Serial1.begin(9600);
    delay(5000);

  }



  void loop() { 
    // allows reading and erasing flash
    if (Serial.available()) {
    char c = Serial.read();
    c=tolower(c);
    if (c=='e') eraseArrays(); // erases flash memory
    if (c=='r') readArrays(); // reads flash memory
    if (c=='b') recordBegin(); // begins recording. DO NOT DO WITHOUT ERASING FLASH.
    if (c=='n') commsReset(); // resets comms available so flash can be read again
    if (c=='a') commsStatus(); // checks the comms status
    if (c=='c') communicate();
  }
  if (Serial1.available()) {
    char c = Serial1.read();
    c=tolower(c);
    if (c=='e') eraseArrays(); // erases flash memory
    if (c=='r') readArrays(); // reads flash memory
    if (c=='b') recordBegin(); // begins recording. DO NOT DO WITHOUT ERASING FLASH.
    if (c=='n') commsReset(); // resets comms available so flash can be read again
    if (c=='a') commsStatus(); // checks the comms status
    if (c=='c') communicate();
  }
  

  }

//-------------------------------------------------------------------------------------------
// HELPER FUNCTIONS
//-------------------------------------------------------------------------------------------

  void eraseArrays(){
  // erase the arrays in flash:  set all the bits to one
  // the next write will faithfully save what was written
  // this function does nothing with the arrays in RAM.  
  // Just write to RAM to erase them.
  int page;
  flashMode(FLASH_ERASE);
  for (page=NANO33BLE_FLASH_LOWEST_PAGE;page<flashNumberOfPages;page++) {
    flashErasePage(page);
  }
  flashMode(FLASH_READONLY);
  // alternative way to erase all of flash:
  flashEraseAll();
  // yet another alternative, erasing the arrays separately.  
  // can be done this way only if FLASH_TIGHT isn't defined.
  // can be done here only if the arrays are global.

  
  Serial.println("Flash arrays erased");
}

void readArrays(){
// precision 6dp, outputs sensor data in CSV format
  int idx;
  Serial.println("index, Pressure, Humidity, Acceleration x, Acceleration y,Acceleration z");
  for (idx=0;idx<SZ_ARRAY;idx++) {
    Serial.print(idx);
    Serial.print("  ,    ");
    Serial.print(pressArray[idx],6);
    Serial.print("  ,  ");
    Serial.print(humidArray[idx],6);
    Serial.print("  ,  ");
    Serial.print(   accelArray_x[idx],6);
    Serial.print("  ,  ");
    Serial.print( accelArray_y[idx],6);
    Serial.print("  ,  ");
    Serial.print( accelArray_z[idx],6);
    Serial.println();
  }
}

void recordBegin() {
  digitalWrite(RED,HIGH);
  ledBlue();
  delay(countdownTime);
  digitalWrite(BLUE,HIGH);
  if (commsavail == 1) { // only begin recording if comms status is 1, this must be manually set
    ledGreen();
    sensortask();
  }
  else {
    ledRed();
    Serial.print(" interrupt occured, recording stopped at reading ");
    Serial.println( j );
  }
  }


void communicate() {
  while(j < SZ_ARRAY) {
      commstask();
      delay(1000);
    }
   commsavail = 0;
   digitalWrite(GREEN, HIGH);
  }


void commsStatus() {
  Serial.print(" comms status: ");
  Serial.println(commsavail);
  Serial.println();
  Serial.println(" to reset, enter n then erase flash by entering e ");
}

void commsReset() {
  // resets commsavail to allow recording
  commsavail = 1;
  digitalWrite(RED, HIGH);
  digitalWrite(GREEN, HIGH);
  digitalWrite(BLUE, HIGH);
}


// functions to change on board RGB status LED
void ledRed() {
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, HIGH);
  digitalWrite(BLUE, HIGH);
}

void ledBlue() {
  digitalWrite(RED, HIGH);
  digitalWrite(GREEN, HIGH);
  digitalWrite(BLUE, LOW);
}

void ledGreen() {
  digitalWrite(RED, HIGH);
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, HIGH);
}
