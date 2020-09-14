/*
 * University of Delaware ASAE 2020-2021 Controls Team
 * Data Acquisition System
 * September 12, 2020
 * 
 * TODO:
 * Include servos for payload
 * Incluide LEDs for statuses
 * Include code to collect all needed data
 * Finish sendDASData()
 */
//--------LIBRARIES--------------------------
#include <SPI.h>
//#include <RH_RF95.h> //LoRa Radio library
//#include <Servo.h> //Servo Library
#include <Wire.h>     //I2C library
//#include <Adafruit_LSM9DS1.h> //LSM library
//#include <Adafruit_Sensor.h>  
//#include <Adafruit_GPS.h> //GPS Library
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "Adafruit_BMP3XX.h" //Altimeter library
#include <LoRa.h>
#include <SparkFunLSM9DS1.h>

//--------LoRa Setup--------------------------------
#define ss 8
#define rst 4
#define dio0 3
/*
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
#define RF95_FREQ 915.0     //915MHz
RH_RF95 rf95(RFM95_CS, RFM95_INT);
*/
int16_t packetnum = 0;  //packet counter
char outputData[512]; //packet that will be sent to ground station
//String outputData;
//uint8_t recvDataPacket[RH_RF95_MAX_MESSAGE_LEN];  //packet that will be received from ground station
//uint8_t recvLen = sizeof(recvDataPacket);

//--------Altimeter Setup---------------------------
Adafruit_BMP3XX bmp; // I2C
float height = 0.0, initHeight = 0.0, pressure, temp;
bool startUp = true;
#define SEALEVELPRESSURE_HPA (1013.25)
/*
//--------Accelerometer Setup-----------------------
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
sensors_event_t accel, magneto, gyro, lsmTemp;
*/
float accelx,accely,accelz, magx, magy, magz, gyrox, gyroy, gyroz;
LSM9DS1 imu;
//Function definitions
void printGyro();
void printAccel();
void printMag();
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);
// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.
//--------GPS Setup---------------------------------
/*
//GPS TX pin to feather pin 9
//GPS RX pin to feather pin 10
SoftwareSerial GPSSerial(9, 10);
Adafruit_GPS GPS(&GPSSerial);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true

uint8_t GPShour, GPSmin, GPSsec;
float GPSlat,GPSlong, GPSangle, GPSspeed;
char latDir, longDir;
*/
static const int RXPin = 10, TXPin = 9;
TinyGPSPlus gps;
SoftwareSerial GPSSerial(RXPin, TXPin);
uint8_t GPShour, GPSmin, GPSsec;
float GPSlat,GPSlong, GPScourse, GPSspeed;
//--------Servos Setup------------------------------
uint8_t habDropped = 0, watDropped = 0, cdaDropped = 0;
//--------LEDs Setup--------------------------------

//--------Battery Monitor Setup---------------------

/*
 * Function to send all relevant data to the ground station
 * Altutude:     Current altitude of the plane in feet, taken from Altimeter
 * Temperature:  Current temperature of the plane, taken from Altimeter
 * Latitude:     Current latitude of the plane, taken from GPS
 * Longitude:    Current longitude of the plane, taken from GPS
 * Heading:      Current Heading of the plane, taken from GPS
 * Speed:        Current speed of the plane, taken from GPS
 * Acceleration: Current acceleration of the plane in x y z, taken from LSM
 * Gyroscope:    Current rotation of the plane in x y z, taken from LSM
 * Magnetometer: Current magnetics of the plane in x y z, taken from LSM
 * Battery:      Current remaining battery power
 */
bool sendDASData(){
  //Should be in format:
  //{"alt":XX.XX,"temp":XX.XX,"lat":XX.XXXXXX,"lon":XX.XXXXXX,"heading":XX.XX,"speed":XX.XX,"accelx":XX.XX,"accely":XX.XX,"accelz":XX.XX,"gyrox":XX.XX,"gyroy":XX.XX,"gyros":XX.XX,"magx":XX.XX,"magy":XX.XX,"magz":XX.XX,"hour":XX,"min":XX,"sec":XX}
  snprintf(outputData,512, "{\"alt\":%d.%02d,\"temp\":%d.%02d,\"lat\":%d.%06ld,\"lon\":%d.%06ld,\"heading\":%d.%02d,\"speed\":%d.%02d,\"accelx\":%d.%02d,\"accely\":%d.%02d,\"accelz\":%d.%02d,\"gyrox\":%d.%02d,\"gyroy\":%d.%02d,\"gyroz\":%d.%02d,\"magx\":%d.%02d,\"magy\":%d.%02d,\"magz\":%d.%02d,\"hour\":%d,\"min\":%d,\"sec\":%d}",
  int(height),int(abs(height)*100)%100,int(temp),int(abs(temp)*100)%100,int(GPSlat),long(abs(GPSlat)*1000000)%1000000,int(GPSlong),long(abs(GPSlong)*1000000)%1000000,
  int(GPScourse),int(abs(GPScourse)*100)%100,int(GPSspeed),int(abs(GPSspeed)*100)%100,int(accelx),int(abs(accelx)*100)%100,int(accely),int(abs(accely)*100)%100,
  int(accelz),int(abs(accelz)*100)%100,int(magx),int(abs(magx)*100)%100,int(magy),int(abs(magy)*100)%100,
  int(magz),int(abs(magz)*100)%100,int(gyrox),int(abs(gyrox)*100)%100,int(gyroy),int(abs(gyroy)*100)%100,
  int(gyroz),int(abs(gyroz)*100)%100,int(GPShour),int(GPSmin),int(GPSsec));
/*
  outputData = "{\"alt\":";      outputData += String(height,2);
  outputData += ",\"temp\":";    outputData += String(temp,2);
  outputData += ",\"lat\":";     outputData += String(GPSlat,6);
  outputData += ",\"lon\":";     outputData += String(GPSlong,6);
  outputData += ",\"heading\":"; outputData += String(GPSangle,2);
  outputData += ",\"speed\":";   outputData += String(GPSspeed,2);
  outputData += ",\"accelx\":";  outputData += String(accel.acceleration.x,2);
  outputData += ",\"accely\":";  outputData += String(accel.acceleration.y,2);
  outputData += ",\"accelz\":";  outputData += String(accel.acceleration.z,2);
  outputData += ",\"magx\":";    outputData += String(magneto.magnetic.x,2);
  outputData += ",\"magy\":";    outputData += String(magneto.magnetic.y,2);
  outputData += ",\"magz\":";    outputData += String(magneto.magnetic.z,2);
  outputData += ",\"gyrox\":";   outputData += String(gyro.gyro.x,2);
  outputData += ",\"gyroy\":";   outputData += String(gyro.gyro.y,2);
  outputData += ",\"gyroz\":";   outputData += String(gyro.gyro.z,2);
  outputData += ",\"hour\":";    outputData += String(GPShour);
  outputData += ",\"min\":";     outputData += String(GPSmin);
  outputData += ",\"sec\":";     outputData += String(GPSsec);
  outputData +="}";
  */
  Serial.print("Sending Packet:");
  Serial.println(outputData);
  //uint8_t outputBuf[sizeof(outputData) + 1];
  //outputData.toCharArray(outputBuf, sizeof(outputBuf) - 1);
  if(!LoRa.beginPacket()){
    return false;
  }
  else{
    LoRa.print(outputData);
    LoRa.endPacket();
    return true;
  }
}

/*
 * Drops payload, using servos
 * payload: The specific payload to drop
 *    "HAB": Habitat
 *    "WAT": Water bottle
 *    "CDA": CDA
 */
bool drop(String payload){
  if(payload == "HAB"){      //Drop the hab
    Serial.println("Drop Habitats");
    habDropped = 1;
    return true;
  }
  else if(payload == "WAT"){ //Drop the water bottles
    Serial.println("Drop Water Bottles");
    watDropped = 1;
    return true;
  }
  else if(payload == "CDA"){ //Drop the cda
    Serial.println("Drop CDA");
    cdaDropped = 1;
    return true;
  }
  else{   //Unacceptable drop type
    Serial.println("Bad Drop");
    return false;
  }
}
void setup() {
  Serial.begin(9600);
  while(!Serial){delay(1);}
  Serial.println("Initializing Data Acquisition System");
  Wire.begin(); //Begin I2C
  
  //--------LoRa Init--------------------------------------------
  Serial.println("LoRa Initializing");
  LoRa.setPins(ss, rst, dio0);
  while (!LoRa.begin(915E6)) {
    //Serial.println(".");
    delay(500);
  }
  //LoRa.setSyncWord(0xAA); //Think of sync word as the "channel" for the LoRa module.  Renages from 0x00-0xFF (0-255)
 Serial.println("LoRa Initialized OK");

  //--------Accelerometer Init--------------------------------------------
  Serial.println("LSM Initializing");
  /*
  if (!lsm.begin()){
    Serial.println("LSM init failed, check wiring and restart system");
    while (1);
  }
  //Lower ranges/gains/scales = higher sensitivity
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);    //2G,4G,8G, or 16G
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);   //4GAUSS,8GAUSS,12GAUSS, or 16GAUSS
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS); //245DPS, 500DPS, or 2000DPS
  */
  if(!imu.begin()){
    Serial.println("IMU init failed, check wiring and restart system");
  }
  Serial.println("LSM Initialized OK");

  //--------GPS Init------------------------------------------------------
  Serial.println("GPS Initializing");
  GPSSerial.begin(9600);
  while(!GPSSerial);
  Serial.println("GPS Initialized OK");

  //--------Altimeter Init-------------------------------------------------
  Serial.println("Altimeter Initializing");
  if(!bmp.begin()){
    Serial.println("Altimeter init failed, check wiring and restart system");
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  //bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  Serial.println("Altimeter Initialized OK");

  Serial.println("Data Acquisition System Initialized OK");
}

uint32_t timer = millis();
uint32_t sendDataTimer = millis();
void loop() {
  //--------Receive any commands from ground station------------------------------
  /*
  if(rf95.recv(recvDataPacket, &recvLen)){ //Message from ground station
    Serial.println("Received Packet:");
    Serial.println((char*)recvDataPacket);
    Serial.print("RSSI: ");Serial.println(rf95.lastRssi(), DEC);    
    if(drop(recvDataPacket)){
      uint8_t dropResponse[] = "Good Drop";
      rf95.send(dropResponse, sizeof(dropResponse));
      }
    else{
      uint8_t dropResponse[] = "Bad Drop";
      rf95.send(dropResponse, sizeof(dropResponse));
    }
    
    rf95.waitPacketSent();
  }
  */
  //--------Update GPS Data-------------------------------------------------------
   while(GPSSerial.available() > 0)
      gps.encode(GPSSerial.read());
    if(gps.location.isUpdated()){
      GPSlat = gps.location.lat();
      GPSlong = gps.location.lng();
      //Serial.write(GPSSerial.read());
      //Serial.print("Lat: ");Serial.println(latitude,6);
      //Serial.print("Lon: ");Serial.println(longitude,6);
    }
    if(gps.speed.isValid()){
      GPSspeed = gps.speed.kmph();
    }
    if(gps.course.isValid()){
      GPScourse = gps.course.deg();
    }
    if(gps.time.isValid()){
      GPShour = gps.time.hour();
      GPSmin = gps.time.minute();
      GPSsec = gps.time.second();
    }
    

  //--------Update Altimeter Data---------------------------------------------------
  if(!bmp.performReading()){
    Serial.println("BMP Failed to Read Data");
  }
  else{
    if(startUp){ //First measurement, set initial height
      temp = bmp.temperature; //temp in C
      pressure = bmp.pressure / 100.0; //pressure in hPa
      initHeight = bmp.readAltitude(SEALEVELPRESSURE_HPA) * 3.28084; //convert meters to feet
      startUp = false;
    }
    temp = bmp.temperature; //temp in C
    pressure = bmp.pressure / 100.0; //pressure in hPa
    height = (bmp.readAltitude(SEALEVELPRESSURE_HPA) * 3.28084) - initHeight; 
  }

  //--------Update LSM Data---------------------------------------------------------
  /*
  lsm.read(); //read new data
  lsm.getEvent(&accel, &magneto, &gyro, &lsmTemp);
*/
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
    gyrox = imu.calcGyro(imu.gx);
    gyroy = imu.calcGyro(imu.gy);
    gyroz = imu.calcGyro(imu.gz);
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
    accelx = imu.calcAccel(imu.ax);
    accely = imu.calcAccel(imu.ay);
    accelz = imu.calcAccel(imu.az);
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
    magx = imu.calcMag(imu.mx);
    magy = imu.calcMag(imu.my);
    magz = imu.calcMag(imu.mz);
  }
  
  //--------Send Data---------------------------------------------------------------
  if(millis() - sendDataTimer > 500){ //Send data every 500ms (0.5s)
    if(sendDASData()){
      Serial.println("Good Send");
    }
    else{
      Serial.println("Bad Send");
    }
  }
}
