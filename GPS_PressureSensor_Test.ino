#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;
SoftwareSerial mySerial(8, 7);

#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_Q_RELEASE "$PMTK605*31"

int bmpConnect;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  mySerial.begin(9600);
  delay(2000);
  if (bmp.begin_I2C()) {
    bmpConnect = 1;
  } else {
    bmpConnect = 0;
  }

  mySerial.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  mySerial.println(PMTK_SET_NMEA_UPDATE_5HZ);
  Serial.println("Get version!");
  mySerial.println(PMTK_Q_RELEASE);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (bmpConnect && mySerial.available() > 0) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);   
  } 
}
