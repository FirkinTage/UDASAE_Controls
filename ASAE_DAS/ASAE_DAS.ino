/*
 * University of Delaware ASAE 2020-2021 Controls Team
 * Eric Rivas, Tage Firkin, Dylan Frasher, James Polemeni
 * Data Acquisition System
 * September 12, 2020
 * 
 * TODO:
 * Include servos for payload
 * Incluide LEDs for statuses
 * Test
 */
//--------LIBRARIES--------------------------
#include <SPI.h>
#include <RH_RF95.h> //LoRa Radio library
#include <Servo.h> //Servo Library
#include <Wire.h>     //I2C library
#include <Adafruit_LSM9DS1.h> //LSM library
#include <Adafruit_Sensor.h>  
#include <Adafruit_GPS.h> //GPS Library
#include "Adafruit_BMP3XX.h" //Altimeter library

//--------LoRa Setup--------------------------------
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3 //32u4 = 7 M0 = 3
#define RF95_FREQ 915.0     //915MHz
RH_RF95 rf95(RFM95_CS, RFM95_INT);
int16_t packetNum = 0;  //packet counter
uint8_t recvDataPacket[RH_RF95_MAX_MESSAGE_LEN];  //packet that will be received from ground station
uint8_t recvLen = sizeof(recvDataPacket);
  String outputData;

//--------Altimeter Setup---------------------------
Adafruit_BMP3XX bmp; // I2C
bool startUp = true;
#define SEALEVELPRESSURE_HPA (1013.25)
float height = 0.0, initHeight = 0.0, habHeight = 0.0, cdaHeight = 0.0, watHeight = 0.0, pressure, BMPtemp;

//--------Accelerometer Setup-----------------------
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
sensors_event_t accl, magneto, gyo, lsmTemp;

//--------GPS Setup---------------------------------
//GPS TX pin to feather pin RX
//GPS RX pin to feather pin TX
Adafruit_GPS GPS(&Serial1);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  false

uint8_t GPShour, GPSmin, GPSsec;
float GPSlat,GPSlong, GPSangle, GPSspeed;
char latDir, longDir;
//--------Servos Setup------------------------------
#define servo1Pin 10
#define servo2Pin 11
#define servoGPin 12
Servo servo1, servo2, servoG;
uint8_t habDropped = 0, watDropped = 0, cdaDropped = 0;
//--------LEDs Setup--------------------------------
#define redLED A1
#define greLED A2
#define bluLED A3
//--------Battery Monitor Setup---------------------

/*
 * Function to send all relevant data to the ground station
 * Packet:       Number of times a packet has been sent
 * Drop:         Satus of payload drops. hab,wat,cda where 000 is non dropped, 111 is all dropped, and 010 is water dropped
 * Altutude:     Current altitude of the plane in feet, taken from Altimeter
 * Temperature:  Current temperature of the plane, taken from Altimeter
 * Latitude:     Current latitude of the plane, taken from GPS
 * Longitude:    Current longitude of the plane, taken from GPS
 * Heading:      Current Heading of the plane, taken from GPS
 * Speed:        Current speed of the plane, taken from GPS
 * Acceleration: Current acceleration of the plane in x y z, taken from LSM
 * Gyroscope:    Current rotation of the plane in x y z, taken from LSM
 * Magnetometer: Current magnetics of the plane in x y z, taken from LSM
 * Hour:         Hour when last GPS position taken
 * Minute:       Minute when last GPS position taken
 * Second:       Second when last GPS position taken
 */
bool sendDASData(){
  digitalWrite(bluLED, HIGH);
  outputData = "{\"pk\":";  outputData += String(packetNum);
  outputData += ",\"dp\":"; outputData += String(habDropped); outputData += String(cdaDropped); outputData += String(watDropped);
  outputData += ",\"al\":"; outputData += String(height,2);
  outputData += ",\"lt\":"; outputData += String(GPSlat,6);
  outputData += ",\"ln\":"; outputData += String(GPSlong,6);
  outputData += ",\"hd\":"; outputData += String(GPSangle,2);
  outputData += ",\"sp\":"; outputData += String(GPSspeed,2);
  outputData += ",\"ax\":"; outputData += String(accl.acceleration.x,2);
  outputData += ",\"ay\":"; outputData += String(accl.acceleration.y,2);
  outputData += ",\"az\":"; outputData += String(accl.acceleration.z,2);
  /*
  outputData += ",\"mx\":"; outputData += String(magneto.magnetic.x,2);
  outputData += ",\"my\":"; outputData += String(magneto.magnetic.y,2);
  outputData += ",\"mz\":"; outputData += String(magneto.magnetic.z,2);
  */
  outputData += ",\"gx\":"; outputData += String(gyo.gyro.x,2);
  outputData += ",\"gy\":"; outputData += String(gyo.gyro.y,2);
  outputData += ",\"gz\":"; outputData += String(gyo.gyro.z,2);
  outputData += ",\"hH\":"; outputData += String(habHeight,2);
  outputData += ",\"cH\":"; outputData += String(cdaHeight,2);
  outputData += ",\"wH\":"; outputData += String(watHeight,2);
  outputData +="}";

  uint8_t dataOut[outputData.length()];
  outputData.getBytes(dataOut,outputData.length()+1);
  bool sendPacketConfirm = rf95.send(dataOut, sizeof(dataOut));
  
  rf95.waitPacketSent();
  Serial.println("Sent:");
  Serial.println(outputData);
  if(sendPacketConfirm){ 
    packetNum++;
  }
  delay(100);
  digitalWrite(bluLED, LOW);
  return sendPacketConfirm;
}

/*
 * Drops payload, using servos
 * payload: The specific payload to drop
 *    "HAB": Habitat
 *    "WAT": Water bottle
 *    "CDA": CDA
 */
bool drop(uint8_t* payload){
  if(strncmp((char*)payload,"PAY",3) == 0){      //Drop the hab
    Serial.println("Drop Habitats/Water Bottles");
    habDropped = 1;
    watDropped = 1;
    servo1.write(180);  //Turn servo to 180 degrees
    servo2.write(180);  //Turn servo to 180 degrees
    BMPtemp = bmp.temperature; //temp in C
    pressure = bmp.pressure / 100.0; //pressure in hPa
    habHeight = (bmp.readAltitude(SEALEVELPRESSURE_HPA) * 3.28084) - initHeight; 
    watHeight = habHeight;
    if(sendDASData()){
      Serial.println("Good Send");
      delay(100);
      return true;
    }
    else{
      Serial.println("Bad Send");
      delay(100);
      return false;
    }
  }
  /*
  else if(payload == (uint8_t*)"WAT"){ //Drop the water bottles
    Serial.println("Drop Water Bottles");
    watDropped = 1;
    if(sendDASData()){
      Serial.println("Good Send");
    }
    else{
      Serial.println("Bad Send");
    }
  }
  */
  else if((strncmp((char*)payload,"CDA",3) == 0)){ //Drop the cda
    Serial.println("Drop CDA");
    cdaDropped = 1;
    servoG.write(180);  //Turn servo to 180 degrees
    BMPtemp = bmp.temperature; //temp in C
    pressure = bmp.pressure / 100.0; //pressure in hPa
    cdaHeight = (bmp.readAltitude(SEALEVELPRESSURE_HPA) * 3.28084) - initHeight; 
    if(sendDASData()){
      Serial.println("Good Send");
      delay(100);
      return true;
    }
    else{
      Serial.println("Bad Send");
      delay(100);
      return false;
    }
  }
  else{   //Unacceptable drop type
    Serial.println("Bad Drop");
    delay(100);
    return false;
  }
}
void setup() {
  Serial.begin(9600);
  //while(!Serial){delay(1);} //Used for debugging
  Serial.println("Initializing Data Acquisition System");
  Wire.begin();

  //--------Servo Init-------------------------------------------
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servoG.attach(servoGPin);
  servo1.write(0);        //Set to 0 degrees
  servo2.write(0);        //Set to 0 degrees
  servoG.write(0);        //Set to 0 degrees
  
  //--------LED Init---------------------------------------------
  pinMode(redLED, OUTPUT);
  pinMode(greLED, OUTPUT);
  pinMode(bluLED, OUTPUT);
  digitalWrite(redLED, LOW);
  digitalWrite(greLED, LOW);
  digitalWrite(bluLED, LOW);
  
  //--------LoRa Init--------------------------------------------
  Serial.println("LoRa Initializing");
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  while (!rf95.init()) {
    digitalWrite(redLED, HIGH);
    digitalWrite(greLED, LOW);
    digitalWrite(bluLED, LOW);
    Serial.println("LoRa radio init failed, restart system");
    while (1);
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    digitalWrite(redLED, LOW);
    digitalWrite(greLED, HIGH);
    digitalWrite(bluLED, LOW);
    Serial.println("setFrequency failed, restart system");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);     //you can set transmitter powers from 5 to 23 dBm:
  Serial.println("LoRa Initialized OK");

  //--------Accelerometer Init--------------------------------------------
  Serial.println("LSM Initializing");
  if (!lsm.begin()){
    digitalWrite(redLED, LOW);
    digitalWrite(greLED, LOW);
    digitalWrite(bluLED, HIGH);
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
  Serial.println("LSM Initialized OK");

  //--------GPS Init------------------------------------------------------
  Serial.println("GPS Initializing");
  GPS.begin(9600);
  //uncomment below line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA); // Request updates on antenna status, comment out to keep quiet
  Serial.println("GPS Initialized OK");

  //--------Altimeter Init-------------------------------------------------
  Serial.println("Altimeter Initializing");
  if(!bmp.begin()){
  digitalWrite(redLED, HIGH);
  digitalWrite(greLED, HIGH);
  digitalWrite(bluLED, LOW);
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

uint32_t GPStimer = millis();
uint32_t sendDataTimer = millis();
void loop() {
  //--------Receive any commands from ground station------------------------------
  if(rf95.available()){
    if(rf95.recv(recvDataPacket, &recvLen)){ //Message from ground station
      if(strlen((char*)recvDataPacket) > 0){
        Serial.println(strlen((char*)recvDataPacket));
        Serial.println("Received Packet:");
        Serial.println((char*)recvDataPacket);
        Serial.print("RSSI: ");Serial.println(rf95.lastRssi(), DEC);    
        drop(recvDataPacket);
      }
    }
  }
  
  //--------Update GPS Data-------------------------------------------------------
  char GPSRaw = GPS.read(); //Read raw GPS data
  /*
  if(GPSECHO){  //Used for debugging, comment out for running
    Serial.write(GPSRaw);
  }
*/
  if(GPS.newNMEAreceived()){
    if(!GPS.parse(GPS.lastNMEA())){
    }
  }
  if(millis() - GPStimer > 1000){  //Update GPS data every second    
    GPStimer = millis(); //reset timer
    GPShour = GPS.hour;
    GPSmin = GPS.minute;
    GPSsec = GPS.seconds;
    if(GPS.fix){
      GPSlat = GPS.latitude_fixed/10000000.0;
      GPSlong = GPS.longitude_fixed/10000000.0;
      latDir = GPS.lat;
      longDir = GPS.lon;
      GPSspeed = GPS.speed * 1.15078; //To convert knots to mph
      GPSangle = GPS.angle;
    }
  }

  //--------Update Altimeter Data---------------------------------------------------
  if(!bmp.performReading()){
    Serial.println("BMP Failed to Read Data");
  }
  else{
    if(startUp){ //First measurement, set initial height
      delay(500);
      BMPtemp = bmp.temperature; //temp in C
      pressure = bmp.pressure / 100.0; //pressure in hPa
      initHeight = bmp.readAltitude(SEALEVELPRESSURE_HPA) * 3.28084; //convert meters to feet
      startUp = false;
      delay(500);
    }
    BMPtemp = bmp.temperature; //temp in C
    pressure = bmp.pressure / 100.0; //pressure in hPa
    height = (bmp.readAltitude(SEALEVELPRESSURE_HPA) * 3.28084) - initHeight; 
  }

  //--------Update LSM Data---------------------------------------------------------
  lsm.read(); //read new data
  lsm.getEvent(&accl, &magneto, &gyo, &lsmTemp);

  //--------Send Data---------------------------------------------------------------
  if(millis() - sendDataTimer > 500){ //Send data every 100ms (0.1s)
    sendDataTimer = millis();
    sendDASData();
  }
}
