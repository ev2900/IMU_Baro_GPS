//GPS
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

//IMU
#include "I2Cdev.h"
#include "MPU6050.h"

//Baro
#include "Wire.h"
#include "I2Cdev.h"
#include "BMP085.h"

//GPS
SoftwareSerial mySerial(4, 3);
Adafruit_GPS GPS(&mySerial);

#define GPSECHO  true

boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

//IMU
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define OUTPUT_READABLE_ACCELGYRO

#define LED_PIN 13
bool blinkState = false;

//Baro
BMP085 barometer;

float temperature;
float pressure;
float altitude;
int32_t lastMicros;

//------------------------------------------------------------------

void setup()  
{
  //GPS  
  Serial.begin(115200);
  
  GPS.begin(9600);
 
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
  
  GPS.sendCommand(PGCMD_ANTENNA);

  useInterrupt(true);
  
  delay(1000);
  
  //IMU
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
  pinMode(LED_PIN, OUTPUT);
  
  //Baro
   Wire.begin();
   
}

//GPS
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
 
  if (GPSECHO)
    if (c) UDR0 = c;  
 
}

void useInterrupt(boolean v) {
  
  if (v) {

    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } 
  else {

    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void loop(){

  //IMU
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    Serial.println("");
    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);
 
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
  
  //Baro
  barometer.setControl(BMP085_MODE_TEMPERATURE);
  
  lastMicros = micros();
  while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());
  
  temperature = barometer.getTemperatureC();
  
  barometer.setControl(BMP085_MODE_PRESSURE_3);
  while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());
  
  pressure = barometer.getPressure();
  
  altitude = barometer.getAltitude(pressure);
  
  Serial.println("");
  Serial.print("T/P/A\t");
  Serial.print(temperature); Serial.print("\t");
  Serial.print(pressure); Serial.print("\t");
  Serial.print(altitude);
  Serial.println("");
  
  delay(75);
  
}
