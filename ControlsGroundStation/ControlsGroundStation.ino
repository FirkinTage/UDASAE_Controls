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
#include <RH_RF95.h>

 //--------LoRa Setup--------------------------------
#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3
#define RF95_FREQ 915.0     //915MHz
RH_RF95 rf95(RFM95_CS, RFM95_INT);
int16_t packetNum = 0;  //packet counter

//---------Button Setup------------------------------
#define payBtn 5
#define cdaBtn 6
#define payDropLED 8
#define cdaDropLED 9
int payDropState = 0,cdaDropState = 0;             //Current state of the drop buttons
int payLEDState = LOW,cdaLEDState = LOW;           //Current state of the drop LEDs
int payDropConfirmed = 0,cdaDropConfirmed = 0;     //Drops confirmed by DAS
int payBtnState, cdaBtnState;
int lastPayBtnState = LOW, lastCdaBtnState = LOW;   //Last button state for debounce
unsigned long debounceTime = 50;                  //Decounce delay time
unsigned long lastPayDebounceTime = 0, lastCdaDebounceTime = 0;
unsigned long lastMillis = 0;
const long blinkInterval = 500;
StaticJsonDocument<250> inData;      //Create JsonDocument object to handle incomming data     

void setup() {
  Serial.begin(9600);
  while (!Serial);
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
  if(rf95.available()){
    uint8_t recvDataPacket[RH_RF95_MAX_MESSAGE_LEN];  //packet that will be received from DAS
    uint8_t recvLen = sizeof(recvDataPacket);
    if(rf95.recv(recvDataPacket, &recvLen)){ //Message from DAS
      if(recvLen>0){
        recvDataPacket[recvLen] = 0;
        Serial.println("Received Packet:");
        char* receivedPacket = (char*) &recvDataPacket;
        //Serial.println(receivedPacket);
        DeserializationError error = deserializeJson(inData, receivedPacket);
        if(error){
          Serial.print(F("deserializeJson() failed: "));
          Serial.println(error.c_str());
        }
        int dropStatus = inData["dp"];
        if(dropStatus == 0){
          payDropConfirmed = 0;
          cdaDropConfirmed = 0;
        }
        else if(dropStatus == 101){
          payDropConfirmed = 1;
          cdaDropConfirmed = 0;
        }
        else if(dropStatus == 10){
          payDropConfirmed = 0;
          cdaDropConfirmed = 1;
        }
        else if(dropStatus == 111){
          payDropConfirmed = 1;
          cdaDropConfirmed = 1;
        }
        
        Serial.print("{");
        Serial.print("\"pkt\":");        serializeJson(inData["pk"], Serial);
        Serial.print(",\"drp\":");       serializeJson(inData["dp"], Serial);
        Serial.print(",\"alt\":");       serializeJson(inData["al"], Serial);
        Serial.print(",\"lat\":");       serializeJson(inData["lt"], Serial);
        Serial.print(",\"lon\":");       serializeJson(inData["ln"], Serial);
        Serial.print(",\"hdng\":");      serializeJson(inData["hd"], Serial);
        Serial.print(",\"spd\":");       serializeJson(inData["sp"], Serial);
        Serial.print(",\"aclx\":");      serializeJson(inData["ax"], Serial);
        Serial.print(",\"acly\":");      serializeJson(inData["ay"], Serial);
        Serial.print(",\"aclz\":");      serializeJson(inData["az"], Serial);
        /*
        Serial.print(",\"magx\":");      serializeJson(inData["mx"], Serial);
        Serial.print(",\"magy\":");      serializeJson(inData["my"], Serial);
        Serial.print(",\"magz\":");      serializeJson(inData["mz"], Serial);
        */
        Serial.print(",\"gyrox\":");     serializeJson(inData["gx"], Serial);
        Serial.print(",\"gyroy\":");     serializeJson(inData["gy"], Serial);
        Serial.print(",\"gyroz\":");     serializeJson(inData["gz"], Serial);
        Serial.print(",\"habHeight\":"); serializeJson(inData["hH"], Serial);
        Serial.print(",\"cdaHeight\":"); serializeJson(inData["cH"], Serial);
        Serial.print(",\"watHeight\":"); serializeJson(inData["wH"], Serial);
        Serial.print(",\"rssi\":");      Serial.print(rf95.lastRssi());
        Serial.println("}");
      }
    }
  }

  //--------Buttons For Dropping Payloads----------------------------------------
  int payBtnReading = digitalRead(payBtn);
  int cdaBtnReading = digitalRead(cdaBtn);
  if(payBtnReading != lastPayBtnState){
    lastPayDebounceTime = millis();
  }
  if(cdaBtnReading != lastCdaBtnState){
    lastCdaDebounceTime = millis();
  }
  if((millis() - lastPayDebounceTime) > debounceTime){
    if(payBtnReading != payBtnState){
      payBtnState = payBtnReading;
    }
    if(payBtnState == HIGH){
      uint8_t sendDrop[] = "PAY";
      rf95.send(sendDrop, sizeof(sendDrop));
      rf95.waitPacketSent();
      payDropState = 1;
    }
  }
  if((millis() - lastCdaDebounceTime) > debounceTime){
    if(cdaBtnReading != cdaBtnState){
      cdaBtnState = cdaBtnReading;
    }
    if(cdaBtnState == HIGH){
      uint8_t sendDrop[] = "CDA";
      rf95.send(sendDrop, sizeof(sendDrop));
      rf95.waitPacketSent();
      cdaDropState = 1;
    }
  }
  lastPayBtnState = payBtnReading;
  lastCdaBtnState = cdaBtnReading;
  
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
