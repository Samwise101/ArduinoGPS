#include <TinyGPS++.h>
#include <SoftwareSerial.h>

void displayInfo();
void buttonGetGPSPoint();
void calculateGPSDistance();

static const uint8_t RXPin = A0;
static const uint8_t TXpin = A1;
static const uint8_t GPS_ButtonPin = 2;
static const long GPSbaud = 9600;

int gps_button_state = LOW;

double gps_point_lat_lon_array[2][2];
uint32_t satelite_number = 0;

uint8_t gps_point_counter = 0;

double fullDistanceTraveled = 0;

unsigned long lastTimeButtonStateChanged = millis();
static unsigned long debounceDuration = 100; // miliseconds
uint32_t btnDebounce_Counter = 0;
uint8_t debounceCheck = 0;

TinyGPSPlus gps;

SoftwareSerial ss(RXPin, TXpin);

void setup() {

  // put your setup code here, to run once:
  Serial.begin(GPSbaud);
  ss.begin(GPSbaud);
  pinMode(GPS_ButtonPin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (ss.available() > 0) {

    if(gps.encode(ss.read())){
      displayInfo();
      if(gps.location.isValid()){
        satelite_number = gps.satellites.value();
      }
    }

    if(millis() > 5000 && gps.charsProcessed() < 10){
      Serial.println("No GPS detected!");
    }

    buttonGetGPSPoint();
    calculateGPSDistance();

    Serial.print("Complete distance traveled: ");
    Serial.println(fullDistanceTraveled);
  }
}

void calculateGPSDistance(){
  if(gps_point_counter < 2){
    return;
  }
  double distance = gps.distanceBetween(gps_point_lat_lon_array[0][0] , gps_point_lat_lon_array[0][1], 
                             gps_point_lat_lon_array[1][0], gps_point_lat_lon_array[1][1]);
  
  if(distance != -1){
    fullDistanceTraveled += distance;
    Serial.print("GPS points: ");
    Serial.print("Lat: ");
    Serial.print(gps_point_lat_lon_array[0][0]);
    Serial.print("lon: ");
    Serial.println(gps_point_lat_lon_array[0][1]);
    Serial.print("Lat: ");
    Serial.print(gps_point_lat_lon_array[1][0]);
    Serial.print("lon: ");
    Serial.println(gps_point_lat_lon_array[1][1]);
    Serial.print("Distance between points: ");
    Serial.println(distance);
  }
}

void buttonGetGPSPoint(){
  gps_button_state = digitalRead(GPS_ButtonPin);

  if(gps_button_state == HIGH){
    btnDebounce_Counter++;
    delay(1);

    gps_button_state = digitalRead(GPS_ButtonPin);
    if(gps_button_state == HIGH){
      if(btnDebounce_Counter >= 20 && debounceCheck == 0){
        Serial.println(gps_button_state);
        if(gps_point_counter < 2){

          if (gps.location.isValid()) {
            gps_point_lat_lon_array[gps_point_counter][0] = gps.location.lat();
            gps_point_lat_lon_array[gps_point_counter][1] = gps.location.lng();
            gps_point_counter++;
            Serial.println("GPS location is valid!");
            Serial.print("GPS points count = ");
            Serial.println(gps_point_counter);
          }
          else{
            Serial.println("GPS location is INvalid!");
          }
        }
        else{
            gps_point_lat_lon_array[0][0] = gps_point_lat_lon_array[1][0];
            gps_point_lat_lon_array[0][1] = gps_point_lat_lon_array[1][1];
            gps_point_counter--;
        }
        btnDebounce_Counter = 0;
        debounceCheck = 1;
      }
    }
    else{
      btnDebounce_Counter = 0;
      debounceCheck = 0;
    }
  }
}

void displayInfo() {
  if (gps.location.isValid()) {
    Serial.print("VALID LOCATION: ");
    Serial.print(gps.location.lat(), 10);
    Serial.print(", ");
    Serial.println(gps.location.lng(), 10);
  } else {
    Serial.print("INVALID LOCATION: ");
    Serial.print(gps.location.lat(), 10);
    Serial.print(", ");
    Serial.println(gps.location.lng(), 10);
  }
  Serial.print("Number of satelites: ");
  Serial.println(gps.satellites.value(), 10);

  Serial.print("Date: ");
  Serial.print(gps.date.day(), 10);
  Serial.print(".");
  Serial.print(gps.date.month(), 10);
  Serial.print(".");
  Serial.println(gps.date.year(), 10);

  Serial.print("Time: ");
  Serial.print(gps.time.hour(), 10);
  Serial.print(":");
  Serial.print(gps.time.minute(), 10);
  Serial.print(":");
  Serial.print(gps.time.second(), 10);
  Serial.println();
  Serial.println();
  Serial.println("-------------------------------");
}
