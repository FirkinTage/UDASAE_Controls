/*
 * University of Delaware ASAE 2020-2021 Controls Team
 * Eric Rivas, Tage Firkin, Dylan Frasher, James Polemeni
 * Ground Station
 * September 19, 2020
 * 
 * TODO:
 */

#include <SPI.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <RH_RF95.h>

 //--------LoRa Setup--------------------------------
#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3
#define RF95_FREQ 915.0     //915MHz
RH_RF95 rf95(RFM95_CS, RFM95_INT);
int16_t packetNum = 0;  //packet counter
uint8_t recvDataPacket[RH_RF95_MAX_MESSAGE_LEN];  //packet that will be received from DAS
uint8_t recvLen = sizeof(recvDataPacket);

//---------Button Setup------------------------------
#define payBtn 5
#define cdaBtn 6
#define payDropLED 9
#define cdaDropLED 10
int payDropState = 0,cdaDropState = 0;             //Current state of the drop buttons
int payLEDState = LOW,cdaLEDState = LOW;           //Current state of the drop LEDs
int payDropConfirmed = 0,cdaDropConfirmed = 0; //Drops confirmed by DAS
unsigned long lastMillis = 0;
const long blinkInterval = 500;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Wire.begin();
  Serial.println("Groundstation Initializing");
  
  //---------------LoRa Init------------------------------------------------------
  Serial.println("LoRa Initializing");
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
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

  //--------------Button Init------------------------------------------------------
  pinMode(payBtn, INPUT);
  pinMode(cdaBtn, INPUT);
  pinMode(payDropLED, OUTPUT);
  pinMode(cdaDropLED, OUTPUT);
  
  Serial.println("Groundstation Initialized OK");
  

}

void loop() {
  //--------Receive any packets from DAS---------------------------------------
  if(rf95.recv(recvDataPacket, &recvLen)){ //Message from DAS
    Serial.println("Received Packet:");
    char receivedPacket[recvLen + 1];
    memcpy(receivedPacket, recvDataPacket,recvLen);
    StaticJsonDocument<RH_RF95_MAX_MESSAGE_LEN> inData;      //Create JsonDocument object to handle incomming data     
    StaticJsonDocument<RH_RF95_MAX_MESSAGE_LEN> outData;     //Create JsonDocument object for sending out data 
    DeserializationError error = deserializeJson(inData, receivedPacket);
    if(error){
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
    }
    if(inData["drp"][0] == "1"){
      payDropConfirmed = 1;
    }
    if(inData["drp"][1] == "1"){
      payDropConfirmed = 1;
    }
    if(inData["drp"][2] == "1"){
      cdaDropConfirmed = 1;
    }
    outData["pkt"]   = inData["pkt"];
    outData["drp"]   = inData["drp"];
    outData["alt"]   = inData["alt"];
    outData["temp"]  = inData["temp"];
    outData["lat"]   = inData["lat"];
    outData["lon"]   = inData["lon"];
    outData["hdng"]  = inData["hdng"];
    outData["spd"]   = inData["spd"];
    outData["aclx"]  = inData["aclx"];
    outData["acly"]  = inData["acly"];
    outData["aclz"]  = inData["aclz"];
    outData["magx"]  = inData["magx"];
    outData["magy"]  = inData["magy"];
    outData["magz"]  = inData["magz"];
    outData["gyrox"] = inData["gyrox"];
    outData["gyroy"] = inData["gyroy"];
    outData["gyroz"] = inData["gyroz"];
    outData["hour"]  = inData["hour"];
    outData["min"]   = inData["min"];
    outData["sec"]   = inData["sec"];
    outData["rssi"]  = String(rf95.lastRssi());
    serializeJson(outData, Serial);
    Serial.println();
  }

  //--------Buttons For Dropping Payloads----------------------------------------
  int payBtnState = digitalRead(payBtn);
  int cdaBtnState = digitalRead(cdaBtn);
  if(payBtnState == HIGH){
    uint8_t sendDrop[] = "PAY";
    rf95.send(sendDrop, sizeof(sendDrop));
    rf95.waitPacketSent();
    payDropState = 1;
  }
  if(cdaBtnState == HIGH){
    uint8_t sendDrop[] = "CDA";
    rf95.send(sendDrop, sizeof(sendDrop));
    rf95.waitPacketSent();
    cdaDropState = 1;
  }

  //--------Blink LEDs based on drop status---------------------------------------
  unsigned long currentMillis = millis();
  //If button never pressed, turn off. If drop button pressed blink every 500 ms. If drop confirmed, stay lit.
  if(currentMillis - lastMillis >= blinkInterval){
    lastMillis = millis();
    //Payload Drop LED Setting
    if(payDropConfirmed == 1){payLEDState = HIGH;
    }
    else if(payDropState == 1){
      if(payLEDState == HIGH){payLEDState = LOW;
      }
      else{payLEDState = HIGH;
      }
    }
    else{payLEDState = LOW;
    }
    //CDA Drop LED Setting
    if(cdaDropConfirmed == 1){cdaLEDState = HIGH;
    }
    else if(cdaDropState == 1){
      if(cdaLEDState == HIGH){cdaLEDState = LOW;
      }
      else{cdaLEDState = HIGH;
      }
    }
    else{cdaLEDState = LOW;
    }
    digitalWrite(payDropLED, payLEDState);
    digitalWrite(cdaDropLED, cdaLEDState);
  }
}
