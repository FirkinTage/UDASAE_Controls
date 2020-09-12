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
#include <RH_RF95.h> //LoRa Radio library
#include <Servo.h> //Servo Library
#include <Wire.h>     //I2C library
#include <Adafruit_LSM9DS1.h> //Acceleromter library
#include <Adafruit_Sensor.h>  
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "Adafruit_BMP3XX.h"

//--------LoRa Setup--------------------------------
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
#define RF95_FREQ 915.0     //915MHz
RH_RF95 rf95(RFM95_CS, RFM95_INT);
int16_t packetnum = 0;  //packet counter
char sendDataPacket[200]; //packet that will be sent to ground station
uint8_t recvDataPacket[RH_RF95_MAX_MESSAGE_LEN];  //packet that will be received from ground station
uint8_t recvLen = sizeof(buf);

//--------Altimeter Setup---------------------------
Adafruit_BMP3XX bmp; // I2C
float height = 0.0, initHeight = 0.0;
#define SEALEVELPRESSURE_HPA (1013.25)

//--------Accelerometer Setup-----------------------
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
sensors_event_t accel, magneto, gyro, temp;

//--------GPS Setup---------------------------------
//GPS TX pin to feather pin 9
//GPS RX pin to feather pin 10
SoftwareSerial GPSSerial(9, 10);
uint32_t timer = millis();
Adafruit_GPS GPS(&GPSSerial);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true
float lati,longi; 
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
  snprintf(outputData,200, "{\"alt\":%d.%02d,\"temp\":%d.%02d,\"lat\":%d.%06ld,\"lon\":%d.%06ld}",
  int(height),int(abs(height)*100)%100,int(temp),int(abs(temp)*100)%100,int(lati),long(abs(lati)*1000000)%1000000,int(longi),long(abs(longi)*1000000)%1000000);
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
    Serial.println("LoRa radio init failed, restart system");
    while (1);
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed, restart system");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);     //you can set transmitter powers from 5 to 23 dBm:
  Serial.println("LoRa Initialized OK");

  //--------Accelerometer Init--------------------------------------------
  Serial.println("LSM Initializing");
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

void loop() {
  if(rf95.recv(recvDataPacket, &recvLen)){ //Message from ground station
    Serial.println("Received Packet:");
    serial.println((char*)recvDataPacket);
    Serial.print("RSSI: ");Serial.println(rf95.lastRssi(), DEC);    
    if(drop(recvDataPacket)){uint8_t dropResponse[] = "Good Drop";}
    else{uint8_t dropResponse[] = "Bad Drop";}
    rf95.send(dropResponse, sizeof(dropResponse));
    rf95.waitPacketSent();
  }
}
