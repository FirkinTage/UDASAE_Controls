//--------LIBRARIES--------------------------
#include <SPI.h>
#include <Wire.h>     //I2C library
#include <Servo.h> //Servo Library
#include <Adafruit_GPS.h> //GPS Library
#include <Adafruit_MPL3115A2.h> //Altimeter library

//--------GPS Setup---------------------------------
//GPS TX pin to micro pin 5
//GPS RX pin to micro pin 6
SoftwareSerial GPSSerial(5,6); //RX TX
Adafruit_GPS GPS(&GPSSerial);
float GPSlat,GPSlong, GPSangle, GPSspeed;

//--------Altimeter Setup---------------------------
Adafruit_MPL3115A2 alt = Adafruit_MPL3115A2();
bool startUp = true;
float height = 0.0, initHeight = 0.0;

//--------Servos Setup------------------------------
#define servoLPin 10
#define servoRPin 11
Servo servoL, servoR;

void setup() {
  Serial.begin(115200);
  //while(!Serial){delay(1);} //Used for debugging
  Serial.println("Initializing Glider");
  Wire.begin();
  
  //--------Servo Init-------------------------------------------
  servoL.attach(servoLPin);
  servoR.attach(servoRPin);
  servoL.write(0);        //Set to 0 degrees
  servoR.write(0);        //Set to 0 degrees

  //--------GPS Init------------------------------------------------------
  Serial.println("GPS Initializing");
  GPS.begin(9600);
  //uncomment below line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA); // Request updates on antenna status, comment out to keep quiet
  delay(1000);
  Serial.println("GPS Initialized OK");
  Serial.println("Glider Initialized OK");
}

uint32_t GPStimer = millis();
uint32_t Alttimer = millis();
void loop() {
  //--------Update GPS Data-------------------------------------------------------
  char GPSRaw = GPS.read(); //Read raw GPS data
  if(GPS.newNMEAreceived()){
    if(!GPS.parse(GPS.lastNMEA())){
      return;
    }
  }
  if(millis() - GPStimer > 1000){  //Update GPS data every second    
    GPStimer = millis(); //reset timer
    if(GPS.fix){
      GPSlat = GPS.latitudeDegrees;
      GPSlong = GPS.longitudeDegrees;
      GPSspeed = GPS.speed * 1.15078; //To convert knots to mph
      GPSangle = GPS.angle;
    }
  }
  
  //--------Update Altimeter Data-------------------------------------------------
  if(startUp){
    if (!alt.begin()) {
      Serial.println("Couldnt find sensor");
      return;
    }
    else{
      delay(500);
      initHeight = alt.getAltitude();
      delay(500);
    }
  }
  else if(millis() - Alttimer > 100){ //Update every 100ms
    Alttimer = millis();
    if (!alt.begin()) {
      Serial.println("Couldnt find sensor");
      return;
    }
    else{
      height = alt.getAltitude() - initHeight;
    }
  }
}
