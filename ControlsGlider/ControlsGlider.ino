//--------LIBRARIES--------------------------
#include <SPI.h>
#include <Wire.h>     //I2C library
#include <Servo.h> //Servo Library
#include <Adafruit_GPS.h> //GPS Library
#include "Adafruit_BMP3XX.h" //Altimeter library
#include <Adafruit_Sensor.h>  

//--------GPS Setup---------------------------------
//GPS TX pin to micro pin RX
//GPS RX pin to micro pin TX
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
float GPSlat,GPSlong, GPSangle, GPSspeed;

//--------Altimeter Setup---------------------------
Adafruit_BMP3XX bmp; // I2C
bool startUp = true;
#define SEALEVELPRESSURE_HPA (1013.25)
float height = 0.0, initHeight = 0.0, pressure, BMPtemp;

//--------Servos Setup------------------------------
#define servoLPin 12
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

  //--------Altimeter Init-------------------------------------------------
  Serial.println("Altimeter Initializing");
  if(!bmp.begin_I2C()){
    Serial.println("Altimeter init failed, check wiring and restart system");
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  //bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  Serial.println("Altimeter Initialized OK");

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
    if(bmp.performReading()){
      delay(500);
      BMPtemp = bmp.temperature; //temp in C
      pressure = bmp.pressure / 100.0; //pressure in hPa
      initHeight = bmp.readAltitude(SEALEVELPRESSURE_HPA) * 3.28084; //convert meters to feet
      startUp = false;
      delay(500);
    }
  }
  else if(millis() - Alttimer > 100){ //Update every 100ms
    Alttimer = millis();
    if(bmp.performReading()){
      BMPtemp = bmp.temperature; //temp in C
      pressure = bmp.pressure / 100.0; //pressure in hPa
      height = (bmp.readAltitude(SEALEVELPRESSURE_HPA) * 3.28084) - initHeight; 
    }
  }
}
