#include <TinyGPS++.h>
#include <SoftwareSerial.h>

void displayInfo();

static const uint8_t RXPin = A0;
static const uint8_t TXpin = A1;
static const long GPSbaud = 9600;

TinyGPSPlus gps;

SoftwareSerial ss(RXPin, TXpin);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  ss.begin(GPSbaud);
}

void loop() {
  // put your main code here, to run repeatedly:

  while(ss.available() > 0){
    if(gps.encode(ss.read())){
      displayInfo();
    }
    if(millis() > 5000 && gps.charsProcessed() < 10){
      Serial.println("No GPS detected!");
    }
  }
}

void displayInfo(){
  if(gps.location.isValid()){
    Serial.print("VALID LOCATION:");
    Serial.print(gps.location.lat(), 10);
    Serial.print(",");
    Serial.println(gps.location.lng(),10);
  }
  else{
    Serial.print("INVALID LOCATION:");
    Serial.print(gps.location.lat(), 10);
    Serial.print(",");
    Serial.println(gps.location.lng(), 10);
  }
  Serial.print("Number of satelites: ");
  Serial.println(gps.satellites.value(),10);

  Serial.print("Date: ");
  Serial.print(gps.date.day(),10);
  Serial.print(".");
  Serial.print(gps.date.month(),10);
  Serial.print(".");
  Serial.println(gps.date.year(),10);

  Serial.print("Time: ");
  Serial.print(gps.time.hour(),10);
  Serial.print(":");
  Serial.print(gps.time.minute(),10);
  Serial.print(":");
  Serial.print(gps.time.second(),10);
  Serial.println();
  Serial.println();
  Serial.println("-------------------------------");
}

