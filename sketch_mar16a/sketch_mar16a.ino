#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <math.h>

void getGPSPoint();
void gpsLoop();
double getNewestLat();
double getNewestLng();
void calculateGPSDistance();
double haversine(double lat1, double lon1, double lat2, double lon2);

static const uint8_t GPS_RXpin = 4;
static const uint8_t GPS_TXpin = 3;
static const long GPSbaud = 9600;

uint8_t gps_dataAvailable = 0;

double gps_point_lat_lon_array[4][2];
uint32_t gps_satelite_number = 0;
uint8_t gps_point_counter = 0; 
double gps_heading_degrees = 0.0;
double fullDistanceTraveled = 0;

TinyGPSPlus gps;
SoftwareSerial ss(GPS_RXpin, GPS_TXpin);

unsigned long gps_previousMillis = 0UL;
unsigned long gps_interval = 1000UL;

void setup() {
  Serial.begin(GPSbaud);
  ss.begin(GPSbaud);
}

void loop() {
  gpsLoop();
}

void gpsLoop(){
  if (ss.available() > 0) {
    if(gps.encode(ss.read())){
      if(gps.location.isValid()){
        unsigned long currentMillis = millis();

        if(currentMillis - gps_previousMillis > gps_interval){
          if(gps_point_counter < 4){
            getGPSPoint();
            if(gps_dataAvailable == 1){
              calculateGPSDistanceAndHeading();
            }
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
  }
  else{
    gps_dataAvailable = 0;
  }

  if(gps_dataAvailable){
    Serial.print("New lat=  ");
    Serial.println(getNewestLat(),10);
    Serial.print("New lng=  ");
    Serial.println(getNewestLng(),10);
    gps_dataAvailable = 0;
  }
}

double getNewestLat(){
  if(gps_point_counter < 1 || gps_point_counter > 4){
    return -1.0;
  }
  return gps_point_lat_lon_array[gps_point_counter][0];
}

double getNewestLng(){
  if(gps_point_counter < 1 || gps_point_counter > 4){
    return -1.0;
  }
  return gps_point_lat_lon_array[gps_point_counter][1];
}

double haversine(double lat1, double lon1, double lat2, double lon2) {
    const double rEarth2 = 6372795.0;
    const double rEarth = 6371000.0;
    double x = pow( sin( ((lat2 - lat1)*M_PI/180.0) / 2.0), 2.0 );
    double y = cos(lat1*M_PI/180.0) * cos(lat2*M_PI/180.0);
    double z = pow( sin( ((lon2 - lon1)*M_PI/180.0) / 2.0), 2.0 );
    double a = x + y * z;
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0-a));
    double d = rEarth2 * c;
    return d;
}

void getGPSPoint(){
  if(gps_point_counter < 4){
    if (gps.location.isValid() && gps.location.isUpdated()) {
      if(gps_point_counter > 1)
        gps_dataAvailable = 1;
      else
        gps_dataAvailable = 0;

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
      gps_dataAvailable = 0;
    }
  }
}

void calculateGPSDistanceAndHeading(){
  double distance = haversine(gps_point_lat_lon_array[gps_point_counter-2][0] , gps_point_lat_lon_array[gps_point_counter-2][1], 
                             gps_point_lat_lon_array[gps_point_counter-1][0], gps_point_lat_lon_array[gps_point_counter-1][1]);
  if(distance > 2.0){
    distance = 0.0;
  }

  if(distance != 0.0){
    fullDistanceTraveled += distance;
  }

  gps_heading_degrees = gps.courseTo(gps_point_lat_lon_array[gps_point_counter-2][0] , gps_point_lat_lon_array[gps_point_counter-2][1], 
                              gps_point_lat_lon_array[gps_point_counter-1][0], gps_point_lat_lon_array[gps_point_counter-1][1]);
  Serial.print(distance,5);
  Serial.print(",");
  Serial.print(fullDistanceTraveled,5);
  Serial.print(",");
  Serial.println(gps_heading_degrees);
}