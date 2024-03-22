#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <math.h>

void getGPSPoint();
void calculateGPSDistance();
double haversine(double lat1, double lon1, double lat2, double lon2);

static const uint8_t RXPin = A0;
static const uint8_t TXpin = A1;
static const long GPSbaud = 9600;

double gps_point_lat_lon_array[4][2];
uint32_t gps_satelite_number = 0;

uint8_t gps_point_counter = 0;

double fullDistanceTraveled = 0;

TinyGPSPlus gps;

SoftwareSerial ss(RXPin, TXpin);

unsigned long previousMillis = 0UL;
unsigned long interval = 1000UL;

void setup() {

  // put your setup code here, to run once:
  Serial.begin(GPSbaud);
  ss.begin(GPSbaud);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (ss.available() > 0) {

    if(gps.encode(ss.read())){
      if(gps.location.isValid()){
        unsigned long currentMillis = millis();

        if(currentMillis - previousMillis > interval){
          if(gps_point_counter < 4){
            getGPSPoint();
            calculateGPSDistance();
            Serial.println();
          }
        }
        
        if(gps_point_counter >= 4){
          gps_point_counter--;
          for(uint8_t i = 0; i < gps_point_counter; i++){
            gps_point_lat_lon_array[i][0] = gps_point_lat_lon_array[i+1][0];
            gps_point_lat_lon_array[i][1] = gps_point_lat_lon_array[i+1][1];
          }
        }
      }
    }

    if(millis() > 5000 && gps.charsProcessed() < 10){
      Serial.println("-1");
    }
  }
}

double haversine(double lat1, double lon1, double lat2, double lon2) {
    const double rEarth2 = 6372795.0;
    const double rEarth = 6371000.0; // in meters
    double x = pow( sin( ((lat2 - lat1)*M_PI/180.0) / 2.0), 2.0 );
    double y = cos(lat1*M_PI/180.0) * cos(lat2*M_PI/180.0);
    double z = pow( sin( ((lon2 - lon1)*M_PI/180.0) / 2.0), 2.0 );
    double a = x + y * z;
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0-a));
    double d = rEarth2 * c;
    return d; // in meters
}

void calculateGPSDistance(){
  if(gps_point_counter <= 1){
    return;
  }
  double distance = haversine(gps_point_lat_lon_array[gps_point_counter-2][0] , gps_point_lat_lon_array[gps_point_counter-2][1], 
                             gps_point_lat_lon_array[gps_point_counter-1][0], gps_point_lat_lon_array[gps_point_counter-1][1]);

  double distance2 = gps.distanceBetween(gps_point_lat_lon_array[gps_point_counter-2][0] , gps_point_lat_lon_array[gps_point_counter-2][1], 
                             gps_point_lat_lon_array[gps_point_counter-1][0], gps_point_lat_lon_array[gps_point_counter-1][1]);

  if(distance2 > 2.0 || distance > 2.0){
    distance = 0.0;
    distance2 = 0.0;
  }

  distance = (distance + distance2)/2.0;

  fullDistanceTraveled += distance;

  Serial.print(distance,10);
  Serial.print(",");
  Serial.print(fullDistanceTraveled,10);
}

void getGPSPoint(){
  if(gps_point_counter < 4){
    if (gps.location.isValid()) {
    //  displayInfo();
      Serial.print("1");
      Serial.print(",");
      
      gps_satelite_number = gps.satellites.value();
      Serial.print(gps_satelite_number);
      Serial.print(",");

      gps_point_lat_lon_array[gps_point_counter][0] = gps.location.lat();
      gps_point_lat_lon_array[gps_point_counter][1] = gps.location.lng();

      Serial.print(gps_point_lat_lon_array[gps_point_counter][0],10);
      Serial.print(",");

      Serial.print(gps_point_lat_lon_array[gps_point_counter][1],10);
      Serial.print(",");

      gps_point_counter++;
    }
    else{
      Serial.print("0");
      Serial.print(",");
    }
  }
}
