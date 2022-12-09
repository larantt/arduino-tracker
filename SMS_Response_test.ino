/* 
 * AUTHOR: Lara Tobias-Tarsh (laratt@umich.edu)
 * LAST MODIFIED: 09/12/2022 - functionalised much of the code for readability. Removed redundancies. Added Kalman filter modes.
 * 
 * DESCRIPTION:
 * Code for Arduino 33 BLE Sense payload acting as a sensor package
 * Nano BLE Sense and the Botletics SIM7000 on the Hologram IOT network.
 * Records data from onboard sensors necessary for an atmospheric sounding, then communicates via serial
 * with the communications module (here an Arduino Mega with a Botletics SIM7000 shield) for remote recovery.
 * Saves all data to flash memory for safety should power cycle during recording.
 * Has an inbuilt kalman filter than can be controlled via SMS or the serial monitor.
 * 
 * USER INPUTS: SZ_ARRAY will define the number of data points recorded during execution of the code. No more than
 * this will be recorded so choose carefully! Countdown time will allow a certain amount of time before recording begins.
 * Uncertainty in the Kalman filter parameters can be edited with regards to the sensor in use.
 * 
 * 
 * For further information, including syntax for SMS commands, refer to the README in the GitLab repo found at
 * https://gitlab.eecs.umich.edu/laratt/balloon-payload . You must request access to this, which I can grant in 
 * accordance with the Michigan Engineering Honor Code on a case by case basis.
 * 
 */



// include libraries
#include <Arduino_HTS221.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_LSM9DS1.h>
#include <Nano33BLEflash.h>
#include <SimpleKalmanFilter.h>

//-------------------------------------------------------------------------------------------
// INITIALISATION
//-------------------------------------------------------------------------------------------
#define SZ_ARRAY 25 //3600 for exactly 2hrs, add safety
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
char buf[300];                // character buffer
int commsavail = 1;           // ensures flash is not overwritten
float countdownTime = 40000;  // length of countdown before recording begins, change as needed

// declare sensor floats
float pressure;
float humidity;
float temps;
float accel_x;
float accel_y;
float accel_z;
float x,y,z;
int j = 0; // for communications indexing

// baseline measurements
float baselinePressure;
float baselineTemperature;
float baselineHumidity;
float baselineAccelx;
float baselineAccely;
float baselineAccelz; 

  
//-------------------------------------------------------------------------------------------
// KALMAN FILTERS
//-------------------------------------------------------------------------------------------
/*
 syntax for: SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
 */

// currently all the same, can change based on calibration
float kalmanStatus = 0; // status of onboard kalman filter. ON value is 1.

SimpleKalmanFilter pressureKalmanFilter(0.1, 0.1, 0.01);    // kalman filter for pressure
SimpleKalmanFilter temperatureKalmanFilter(0.1, 0.1, 0.01); // kalman filter for temperature
SimpleKalmanFilter humidityKalmanFilter(0.1, 0.1, 0.01);    // kalman filter for humidity
SimpleKalmanFilter accelxKalmanFilter(0.1, 0.1, 0.01);      // kalman filter for x acceleration
SimpleKalmanFilter accelyKalmanFilter(0.1, 0.1, 0.01);      // kalman filter for y acceleration
SimpleKalmanFilter accelzKalmanFilter(0.1, 0.1, 0.01);      // kalman filter for z acceleration

void kalmanOn() {
  kalmanStatus = 1;
  ledOrange();
}

void kalmanOff() {
  kalmanStatus = 0;
  ledWhite();
}

void getBaselines() {
    baselinePressure = round(BARO.readPressure() * 1000);
    baselineHumidity = round(HTS.readHumidity());
    baselineTemperature = round(HTS.readTemperature());
    IMU.readAcceleration(x,y,z);
    baselineAccelx = x*9.8066;
    baselineAccely = y*9.8066;
    baselineAccelz = z*9.8066;
}

void kalman_sensortask(){
  // Read each sensor and write to flash memory
  for (int i = 0; i < SZ_ARRAY; i++) {
    getBaselines();
    digitalWrite(LED_BUILTIN, HIGH);
    ledPurple();
    // read the sensors
    pressure = round(BARO.readPressure() * 1000);
    humidity = round(HTS.readHumidity());
    temps = round(HTS.readTemperature());
    IMU.readAcceleration(x,y,z);
    accel_x = x*9.8066;
    accel_y = y*9.8066;
    accel_z = z*9.8066;
    // write kalman filtered value to memory
    flashMode(FLASH_WRITE);
    pressArray[i] = pressureKalmanFilter.updateEstimate(pressure);
    humidArray[i] = humidityKalmanFilter.updateEstimate(humidity);
    tempArray[i] = temperatureKalmanFilter.updateEstimate(temps);
    accelArray_x[i] = accelxKalmanFilter.updateEstimate(accel_x);
    accelArray_y[i] = accelyKalmanFilter.updateEstimate(accel_y);
    accelArray_z[i] = accelzKalmanFilter.updateEstimate(accel_z);
    flashMode(FLASH_READONLY);
    
    digitalWrite(LED_BUILTIN, LOW);
    ledBlue();
    delay(2000); //every 2 ms
  }
  flashMode(FLASH_READONLY);
  digitalWrite(LED_BUILTIN, LOW);
  ledGreen();
}


//-------------------------------------------------------------------------------------------
// SENSOR FUNCTIONS
//-------------------------------------------------------------------------------------------
void sensortask(){
  
  // Read each sensor and write to flash memory
  for (int i = 0; i < SZ_ARRAY; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    ledPurple();
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
    ledBlue();
    delay(2000); //every 2 ms
  }
  flashMode(FLASH_READONLY);
  digitalWrite(LED_BUILTIN, LOW);
  ledGreen();
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
}

//-------------------------------------------------------------------------------------------
// MAIN EXECUTION
//-------------------------------------------------------------------------------------------
  void setup() {
    // redundancy included: force these to a value on restart
    commsavail = 1;
    kalmanOff();
    
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
    if (c=='e') eraseArrays();   // erases flash memory
    if (c=='r') readArrays();    // reads flash memory
    if (c=='b') recordBegin();   // begins recording. DO NOT DO WITHOUT ERASING FLASH.
    if (c=='n') commsReset();    // resets comms available so flash can be read again
    if (c=='a') commsStatus();   // checks the comms status
    if (c=='c') communicate();   // starts communications with mega
    if (c=='k') kalmanOn();      // turns the inbuilt kalman filter on
    if (c=='x') kalmanOff();     // turns the inbuilt kalman filter off
  }
  if (Serial1.available()) {
    char c = Serial1.read();
    c=tolower(c);
    if (c=='e') eraseArrays();   // erases flash memory
    if (c=='r') readArrays();    // reads flash memory
    if (c=='b') recordBegin();   // begins recording. DO NOT DO WITHOUT ERASING FLASH.
    if (c=='n') commsReset();    // resets comms available so flash can be read again
    if (c=='a') commsStatus();   // checks the comms status
    if (c=='c') communicate();   // starts communications with mega
    if (c=='k') kalmanOn();      // turns the inbuilt kalman filter on
    if (c=='x') kalmanOff();     // turns the inbuilt kalman filter off
  }
  

  }

//-------------------------------------------------------------------------------------------
// HELPER FUNCTIONS
//-------------------------------------------------------------------------------------------

  void eraseArrays(){
  if (commsavail == 0) {
    int page;
    flashMode(FLASH_ERASE);
    for (page=NANO33BLE_FLASH_LOWEST_PAGE;page<flashNumberOfPages;page++) {
      flashErasePage(page);
    }
    flashMode(FLASH_READONLY);
    // alternative way to erase all of flash:
    flashEraseAll();
    Serial.println("Flash arrays erased");
  }
  else {
    ledTeal();
    Serial.println(" data has not been retrieved. Reset commsavail first");
  }
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
    // record sensor readings based on kalman filter status
    if (kalmanStatus == 1){
      kalman_sensortask();
    }
    else {
    sensortask();
    }
  }
  else {
    ledRed();
    Serial.print(" interrupt occured, recording stopped at reading ");
    Serial.println( j );
  }
  }


void communicate() {
  while(j < SZ_ARRAY) {
      ledOrange();
      commstask();
      delay(30000);
      ledPurple();
    }
   commsavail = 0;
   digitalWrite(GREEN, HIGH);
   delay(2000);
   ledTeal();
  }


void commsStatus() {
  Serial.print(" comms status: ");
  Serial.println(commsavail);
  if (commsavail==1) {
    ledGreen();
    delay(2000);
    ledWhite();
  }
  else {
    ledTeal();
  }
  Serial.println();
  Serial.println(" to reset, enter n then erase flash by entering e ");
}

void commsReset() {
  // resets commsavail to allow recording
  commsavail = 1;
  ledWhite();
}

//=============================================================
// functions to change on board RGB status LED       
//=============================================================
void ledRed() {
  // used to indicate errors
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, HIGH);
  digitalWrite(BLUE, HIGH);
}

void ledBlue() {
  // used for countdown timers
  digitalWrite(RED, HIGH);
  digitalWrite(GREEN, HIGH);
  digitalWrite(BLUE, LOW);
}

void ledGreen() {
  // used to indicate successfully completed task
  digitalWrite(RED, HIGH);
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, HIGH);
}

void ledPurple() {
  // used to indicate task in process
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, HIGH);
  digitalWrite(BLUE, LOW);
}

void ledOrange() {
  // used to indicate that kalman filter is on
  digitalWrite(RED, HIGH);
  digitalWrite(GREEN, HIGH);
  digitalWrite(BLUE, LOW);
}

void ledTeal() {
  // used to indicate task communications status
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, HIGH);
  digitalWrite(BLUE, HIGH);
}


void ledWhite() {
  // used to indicate readiness for flight
  digitalWrite(RED, HIGH);
  digitalWrite(GREEN, HIGH);
  digitalWrite(BLUE, HIGH);
}
