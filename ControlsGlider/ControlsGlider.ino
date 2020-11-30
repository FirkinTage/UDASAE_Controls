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
float targetLat = 40.164495; //Drop zone latitude in degrees decimal
float targetLon = -74.591218; //Drop zone longitude in degrees decimal

//--------Altimeter Setup---------------------------
Adafruit_MPL3115A2 alt = Adafruit_MPL3115A2();
bool startUp = true;
float height = 0.0, initHeight = 0.0, lastHeight = 0.0, descendSpd;

//--------Servos Setup------------------------------
#define servoLPin 10
#define servoRPin 11
#define servoRstPin 9
Servo servoL, servoR, servoRst;
int servoLangle = 90, servoRangle = 90; //Servos go from 0 to 180 degrees where 0 is "up" and 180 is "down"

void setup() {
  Serial.begin(115200);
  //while(!Serial){delay(1);} //Used for debugging
  Serial.println("Initializing Glider");
  Wire.begin();
  
  //--------Servo Init-------------------------------------------
  servoL.attach(servoLPin);
  servoR.attach(servoRPin);
  servoRst.attach(servoRstPin);
  servoL.write(servoLangle);        //Set to 90 degrees
  servoR.write(servoRangle);        //Set to 90 degrees

  //--------GPS Init------------------------------------------------------
  Serial.println("GPS Initializing");
  GPS.begin(9600);
  //uncomment below line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);   // 5 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA); // Request updates on antenna status, comment out to keep quiet
  delay(1000);
  Serial.println("GPS Initialized OK");
  Serial.println("Glider Initialized OK");
}

uint32_t GPStimer = millis();
uint32_t Alttimer = millis();
void loop() {

  //---------Manual override---------------------------------------------------
  int servoRstAngle = servoRst.read(); //Read manual override
  if( servoRstAngle >= 90 ){ //90 is aplace holder, need to test actual given angles from radio receiver
    Serial.println(servoRstAngle);
    servoL.write(180);  //Set servos to full pitch up
    servoR.write(180);
  }
  //--------Update GPS Data-------------------------------------------------------
  char GPSRaw = GPS.read(); //Read raw GPS data
  if(GPS.newNMEAreceived()){
    if(!GPS.parse(GPS.lastNMEA())){
      return;
    }
  }
  if(millis() - GPStimer > 200){  //Update GPS data every second    
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
      startUp = false;
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
      lastHeight = height;
      height = alt.getAltitude() - initHeight;
      descendSpd = (lastHeight - height) / 0.1; //Determine speed of descent in feet/second
    }
  }

  //---Determine to how to pitch/turn------------------------------------------------
  if(GPSlat != 0 && GPSlong != 0){
    /*
    float xDis = ((targetLon - GPSlong) * cos((GPSlat + targetLat)/2.0));
    float yDis = (targetLat - GPSlat);
    float distanceToTarget = 6371e3 * sqrt((xDis*xDis) + (yDis*yDis));
    */
    float distanceToTarget = 6371e3 * sqrt((((targetLon - GPSlong) * cos((GPSlat + targetLat)/2.0))*((targetLon - GPSlong) * cos((GPSlat + targetLat)/2.0))) + ((targetLat - GPSlat)*(targetLat - GPSlat)));
    float timeToTarget = distanceToTarget/GPSspeed;
    float idealDescentRate;
    /*
    float x = cos(targetLat) * sin(targetLon - GPSlong);
    float y = (cos(GPSlat) * sin(targetLat)) - (sin(GPSlat) * cos(targetLat) * cos(targetLon - GPSlong));
    float angleToTarget = (atan2(x,y)) * (180/3.14159);
    */
    float angleToTarget = (atan2(cos(targetLat) * sin(targetLon - GPSlong),(cos(GPSlat) * sin(targetLat)) - (sin(GPSlat) * cos(targetLat) * cos(targetLon - GPSlong)))) * (180/3.14159);
    int degreeToTurn = 0;
    servoLangle = 0;
    servoRangle = 0;
    if((GPSangle <= 90 && angleToTarget >= 270)){ //Turn left across 360
       degreeToTurn = GPSangle + (360 - angleToTarget);
    }
    else if(GPSangle >= 270 && angleToTarget <=0){ //Turn right across 360
      degreetoTurn = (360 - GPSangle) + angleToTarget;
    }
    else if(GPSangle < angleToTarget){  //Turn right
      degreetoTurn = GPSangle - angleToTarget;
    }
    else{ //Turn left
      degreetoTurn = angleToTarget - GPSangle;
    }
  }
}
