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
#define RFM95_INT 3 //7
#define RF95_FREQ 915.0     //915MHz
RH_RF95 rf95(RFM95_CS, RFM95_INT);
int16_t packetNum = 0;  //packet counter
char outputData[512]; //packet that will be sent to ground station
//String outputData;
uint8_t recvDataPacket[RH_RF95_MAX_MESSAGE_LEN];  //packet that will be received from ground station
uint8_t recvLen = sizeof(recvDataPacket);

//--------Altimeter Setup---------------------------
Adafruit_BMP3XX bmp; // I2C
float height = 1.09, initHeight = 0.0, pressure, BMPtemp;
bool startUp = true;
#define SEALEVELPRESSURE_HPA (1013.25)

//--------Accelerometer Setup-----------------------
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
sensors_event_t accl, magneto, gyo, lsmTemp;

//--------GPS Setup---------------------------------
//GPS TX pin to feather pin RX
//GPS RX pin to feather pin TX
Adafruit_GPS GPS(&Serial1);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true

uint8_t GPShour, GPSmin, GPSsec;
float GPSlat,GPSlong, GPSangle, GPSspeed;
char latDir, longDir;
//--------Servos Setup------------------------------
#define servo1Pin 11
#define servo2Pin 12
#define servoGPin 13
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
  //Should be in format:
  //{"pkt":XX,"drp":XXX,"alt":XX.XX,"temp":XX.XX,"lat":XX.XXXXXX,"lon":XX.XXXXXX,"hdng":XX.XX,"spd":XX.XX,"aclx":XX.XX,"acly":XX.XX,"aclz":XX.XX,"gyrox":XX.XX,"gyroy":XX.XX,"gyroz":XX.XX,"magx":XX.XX,"magy":XX.XX,"magz":XX.XX,"hour":XX,"min":XX,"sec":XX}
  
  snprintf(outputData,512, "{\"pkt\":%d,\"drp\":%u%u%u,\"alt\":%d.%02d,\"temp\":%d.%02d,\"lat\":%d.%06ld,\"lon\":%d.%06ld,\"hdng\":%d.%02d,\"spd\":%d.%02d,\"aclx\":%d.%02d,\"acly\":%d.%02d,\"aclz\":%d.%02d,\"gyrox\":%d.%02d,\"gyroy\":%d.%02d,\"gyroz\":%d.%02d,\"magx\":%d.%02d,\"magy\":%d.%02d,\"magz\":%d.%02d,\"hour\":%d,\"min\":%d,\"sec\":%d}",
  int(packetNum),habDropped,watDropped,cdaDropped,int(height),int(fabs(height)*100)%100,int(BMPtemp),int(fabs(BMPtemp)*100)%100,int(GPSlat),long(fabs(GPSlat)*1000000)%1000000,int(GPSlong),long(fabs(GPSlong)*1000000)%1000000,
  int(GPSangle),int(fabs(GPSangle)*100)%100,int(GPSspeed),int(fabs(GPSspeed)*100)%100,int(accl.acceleration.x),int(fabs(accl.acceleration.x)*100)%100,int(accl.acceleration.y),int(fabs(accl.acceleration.y)*100)%100,
  int(accl.acceleration.z),int(fabs(accl.acceleration.z)*100)%100,int(magneto.magnetic.x),int(fabs(magneto.magnetic.x)*100)%100,int(magneto.magnetic.y),int(fabs(magneto.magnetic.y)*100)%100,
  int(magneto.magnetic.z),int(fabs(magneto.magnetic.z)*100)%100,int(gyo.gyro.x),int(fabs(gyo.gyro.x)*100)%100,int(gyo.gyro.y),int(fabs(gyo.gyro.y)*100)%100,
  int(gyo.gyro.z),int(fabs(gyo.gyro.z)*100)%100,int(GPShour),int(GPSmin),int(GPSsec));
  
/*
   //{"pkt":XX,"drp":XXX,"alt":XX.XX,"temp":XX.XX,"lat":XX.XXXXXX,"lon":XX.XXXXXX,"hdng":XX.XX,"spd":XX.XX,"aclx":XX.XX,"acly":XX.XX,"aclz":XX.XX,"gyrox":XX.XX,"gyroy":XX.XX,"gyroz":XX.XX,"magx":XX.XX,"magy":XX.XX,"magz":XX.XX,"hour":XX,"min":XX,"sec":XX}
  snprintf(outputData,512, "{\"pkt\":%d,\"drp\":%u%u%u,\"alt\":%d.%d,\"temp\":%d.%02d,\"lat\":%d.%06ld,\"lon\":%d.%06ld,\"hdng\":%d.%02d,\"spd\":%d.%02d,\"aclx\":%d.%02d,\"acly\":%d.%02d,\"aclz\":%d.%02d,\"gyrox\":%d.%02d,\"gyroy\":%d.%02d,\"gyroz\":%d.%02d,\"magx\":%d.%02d,\"magy\":%d.%02d,\"magz\":%d.%02d,\"hour\":%d,\"min\":%d,\"sec\":%d}",
  int(packetNum),habDropped,watDropped,cdaDropped,int(height),int(fabs(height)*100)%100,int(BMPtemp),int(fabs(BMPtemp)*100)%100,int(GPSlat),long(fabs(GPSlat)*1000000)%1000000,int(GPSlong),long(fabs(GPSlong)*1000000)%1000000,
  int(GPSangle),int(fabs(GPSangle)*100)%100,int(GPSspeed),int(fabs(GPSspeed)*100)%100,int(3.4),int(fabs(3.4)*100)%100,int(0.5),int(fabs(0.5)*100)%100,
  int(1.2),int(fabs(1.2)*100)%100,int(10.23),int(fabs(10.23)*100)%100,int(90.5),int(fabs(90.5)*100)%100,
  int(30.3),int(fabs(30.3)*100)%100,int(20.3),int(fabs(20.3)*100)%100,int(34.65),int(fabs(34.65)*100)%100,
  int(-45.45),int(fabs(-45.45)*100)%100,int(GPShour),int(GPSmin),int(GPSsec));
*/ 
/*
  outputData = "{\"pkt\":";    outputData += String(packetNum);
  outputData += ",\"alt\":";   outputData += String(height,2);
  outputData += ",\"temp\":";  outputData += String(temp,2);
  outputData += ",\"lat\":";   outputData += String(GPSlat,6);
  outputData += ",\"lon\":";   outputData += String(GPSlong,6);
  outputData += ",\"hdng\":";  outputData += String(GPSangle,2);
  outputData += ",\"spd\":";   outputData += String(GPSspeed,2);
  outputData += ",\"aclx\":";  outputData += String(accel.acceleration.x,2);
  outputData += ",\"acly\":";  outputData += String(accel.acceleration.y,2);
  outputData += ",\"aclz\":";  outputData += String(accel.acceleration.z,2);
  outputData += ",\"magx\":";  outputData += String(magneto.magnetic.x,2);
  outputData += ",\"magy\":";  outputData += String(magneto.magnetic.y,2);
  outputData += ",\"magz\":";  outputData += String(magneto.magnetic.z,2);
  outputData += ",\"gyrox\":"; outputData += String(gyro.gyro.x,2);
  outputData += ",\"gyroy\":"; outputData += String(gyro.gyro.y,2);
  outputData += ",\"gyroz\":"; outputData += String(gyro.gyro.z,2);
  outputData += ",\"hour\":";  outputData += String(GPShour);
  outputData += ",\"min\":";   outputData += String(GPSmin);
  outputData += ",\"sec\":";   outputData += String(GPSsec);
  outputData +="}";
  */
  Serial.print("Sending Packet:");
  Serial.println(outputData);
  //uint8_t outputBuf[sizeof(outputData) + 1];
  //outputData.toCharArray(outputBuf, sizeof(outputBuf) - 1);
  if(!rf95.send((uint8_t*)outputData, sizeof((uint8_t*)outputData))){
    return false;
  }
  else{
    packetNum++;
    return true;
  }
  memset(outputData, 0, 512); //clear output data for future transmissions
}

/*
 * Drops payload, using servos
 * payload: The specific payload to drop
 *    "HAB": Habitat
 *    "WAT": Water bottle
 *    "CDA": CDA
 */
bool drop(uint8_t* payload){
  if(payload == (uint8_t*)"PAY"){      //Drop the hab
    Serial.println("Drop Habitats/Water Bottles");
    habDropped = 1;
    watDropped = 1;
    servo1.write(180);  //Turn servo to 180 degrees
    servo2.write(180);  //Turn servo to 180 degrees
    if(sendDASData()){
      Serial.println("Good Send");
    }
    else{
      Serial.println("Bad Send");
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
  else if(payload == (uint8_t*)"CDA"){ //Drop the cda
    Serial.println("Drop CDA");
    cdaDropped = 1;
    servoG.write(180);  //Turn servo to 180 degrees
    if(sendDASData()){
      Serial.println("Good Send");
    }
    else{
      Serial.println("Bad Send");
    }
  }
  else{   //Unacceptable drop type
    Serial.println("Bad Drop");
  }
}
void setup() {
  Serial.begin(9600);
  while(!Serial){delay(1);}
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
  if(rf95.recv(recvDataPacket, &recvLen)){ //Message from ground station
    if(strlen((char*)recvDataPacket) > 0){
      Serial.println(strlen((char*)recvDataPacket));
      Serial.println("Received Packet:");
      Serial.println((char*)recvDataPacket);
      Serial.print("RSSI: ");Serial.println(rf95.lastRssi(), DEC);    
      drop(recvDataPacket);
    }
  }
  
  //--------Update GPS Data-------------------------------------------------------
  char GPSRaw = GPS.read(); //Read raw GPS data
  if((GPSRaw) && (GPSECHO)){  //Used for debugging, comment out for running
    Serial.write(GPSRaw);
  }

  if(GPS.newNMEAreceived()){
    if(!GPS.parse(GPS.lastNMEA())){
      Serial.println("Bad GPS Parse");
    }
  }
  if(millis() - GPStimer > 1000){  //Update GPS data every second    
    GPStimer = millis(); //reset timer
    GPShour = GPS.hour;
    GPSmin = GPS.minute;
    GPSsec = GPS.seconds;
    if(GPS.fix){
      GPSlat = GPS.latitude;
      GPSlong = GPS.longitude;
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
      delay(100);
      BMPtemp = bmp.temperature; //temp in C
      pressure = bmp.pressure / 100.0; //pressure in hPa
      initHeight = bmp.readAltitude(SEALEVELPRESSURE_HPA) * 3.28084; //convert meters to feet
      startUp = false;
      delay(100);
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
    if(sendDASData()){
      Serial.println("Good Send");
    }
    else{
      Serial.println("Bad Send");
    }
  }
}
