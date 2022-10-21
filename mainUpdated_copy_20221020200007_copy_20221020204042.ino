/**
 *  CANSAT Team 6 Flight Software
 *  Two Month, Fall 2022
 *  
 *  Author(s): Thomas Teper, Ian Kramer
 *
 *  Copyright (c) Space Hardware Club, 2022
 */

// sensor testing code
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_GPS.h>
#include <string.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// configure constants
#define SEALEVELPRESSURE_HPA 1013.25
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_Q_RELEASE "$PMTK605*31"

// mission params
#define REFRESH_RATE 200 // ms; 5 Hz
#define ALTITUDE_MAX 400 // altitude to trigger latch

#define VERSION 1.0

// DEBUG flag
#define SERIAL_DEBUG true

// global mission variables
uint32_t TIMESTAMP; // time since POWER ON
uint32_t START_TIME;           // mission start time
unsigned int FLIGHT_STATE = 0; // current flight state (DEFAULT: 0)
float ALT_START;               // variable to hold starting altitude -> calc relative altitude
unsigned int SENSOR_CNT = 0;   // count variable for LED and buzzer
int pos;                       // position of servo
char MISSION_START_CHAR;       // mission start character

// communication definitions (all pins are GP#)
SoftwareSerial GPSSerial(7, 6); // GPS TX -> 7; GPS RX -> 6
Adafruit_GPS GPS(&GPSSerial);
Adafruit_BMP3XX BMP;            // BMP SDA -> 4; SCL -> 5
// SD CARD:                     // SD TX -> 9; RX -> 8
const int BUZZER = 2;           // BUZZER Pico Pin -> 2
const int LED = 3;              // LED Pico Pin -> 3
Servo SERVO;                    // Servo Pico Pin -> 10

// data struct for mission information
struct params {
  const int TEAM_ID = 1006;       // assigned team ID
  uint32_t MISSION_TIME;          // mission time in format hh:mm:ss.ss
  unsigned int PACKET_COUNT = 0;  // total count of transmitted packets
  unsigned int FLIGHT_STATE = 0;  // operating state of the software (0 <--> 4)
  char PL_STATE = 'N';            // payload state (N -> not released; R -> released)
  int ALTITUDE;                   // altitude in meters relative to ground level
  float PRESSURE;                 // pressure in hPa
  float TEMP;                     // temperature in Celsius
  float VOLTAGE;                  
  double GPS_LAT;                 // latitude in decimal degrees
  double GPS_LON;                 // longitude in decimal degrees
};

// define MP structure
params MISSION_PARAMS;

void setup() {

  // setup Serial connections
  Serial.begin(9600);   // USB: Computer Serial (UART)
  Serial1.begin(9600);  // XBee: Serial1 (UART0)
  Serial2.begin(9600);  // SD: Serial2 (UART1)

  // print version number
  Serial.print("Version Number: ");
  Serial.print(VERSION);
  Serial.println();
  
  // GPS Configurations
  #ifdef SERIAL_DEBUG
    Serial.println("Initializing GPS");
  #endif
  GPS.begin(9600);      // GPS: Software Serial (UART)
  delay(2000);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  GPSSerial.println(PMTK_Q_RELEASE);
  #ifdef SERIAL_DEBUG
    Serial.println("Initializing GPS");
  #endif

  // BMP Configurations
  #ifdef SERIAL_DEBUG
    Serial.println("Initializing BMP");
  #endif
  if (!BMP.begin_I2C()) // try BMP config
  {
    Serial.println("BMP Connection Failed: Check Wiring!");
    return;
  }
  // oversampling and filter init
  BMP.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  BMP.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  BMP.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  BMP.setOutputDataRate(BMP3_ODR_50_HZ);

  delay(500);

  // basic configuration variables
  String packetString = ""; // packet string
  START_TIME = TIMESTAMP;   // set the start time to current time for mission start

  // Create file header
  String header = "TEAM_ID, MISSION_TIME, PACKET_COUNT, FLIGHT_STATE, PL_STATE, ALTITUDE, TEMP, VOLTAGE, GPS_LAT, GPS_LONG";

  // Write header to SD
  Serial2.println(header);

  // Servo configuration
  SERVO.attach(10, 500, 2350); // set servo pin and rotation range
  pos = 2350;
  SERVO.write(pos);
  pinMode(BUZZER, OUTPUT);     // set buzzer pins
  pinMode(LED, OUTPUT);        // set led pins
}

void loop() {
  
  // set refresh rate (DEFAULT: 5Hz)
  delay(REFRESH_RATE);

  // create write string and mission params struct
  String packetString;
  if (MISSION_PARAMS.PACKET_COUNT == 0)
    ALT_START = 0;
  else if (MISSION_PARAMS.PACKET_COUNT == 1)
    ALT_START = BMP.readAltitude(SEALEVELPRESSURE_HPA); // define altitude datum

  MISSION_PARAMS.PACKET_COUNT++;
  TIMESTAMP = millis();

  char c = GPS.read();

  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // update mission_params
  if (MISSION_PARAMS.FLIGHT_STATE > 0) {
    MISSION_PARAMS.MISSION_TIME = TIMESTAMP - START_TIME;
    MISSION_PARAMS.ALTITUDE = BMP.readAltitude(SEALEVELPRESSURE_HPA) - ALT_START;
    MISSION_PARAMS.PRESSURE = BMP.readPressure();
    MISSION_PARAMS.TEMP = BMP.readTemperature();
    MISSION_PARAMS.VOLTAGE = 0.0;
    MISSION_PARAMS.GPS_LAT = GPS.latitude;
    MISSION_PARAMS.GPS_LON = GPS.longitude;
    packetString = csvParams(MISSION_PARAMS);
  }

  // main execution code
  switch (MISSION_PARAMS.FLIGHT_STATE) {
    case 0: // IDLE
      // wait for command to set flight state
      if (Serial1.available() > 0) {
        MISSION_START_CHAR = Serial1.read();
        if (MISSION_START_CHAR == 'g') {
          MISSION_PARAMS.FLIGHT_STATE = 1;
          MISSION_PARAMS.PACKET_COUNT = 1;
          START_TIME = TIMESTAMP;
        }
      }
      // MISSION_PARAMS.FLIGHT_STATE = 1;
      #ifdef SERIAL_DEBUG
        Serial.println(packetString);
      #endif
      break;
    case 1: // ASCENT
      Serial1.println(packetString);
      Serial2.println(packetString);
      #ifdef SERIAL_DEBUG
        Serial.println(packetString);
      #endif
      
      // FS1 -> FS2
      if (MISSION_PARAMS.ALTITUDE > ALTITUDE_MAX) {
        MISSION_PARAMS.FLIGHT_STATE = 2; // drop stage

        // rotate the servo
        for (int pos = 2350; pos >= 500; --pos) {
          SERVO.write(pos);
          delay(0.5);
        }

        // TESTING
        // tone(BUZZER, 1000);
        // delay(1001);
        // noTone(BUZZER);      
      }
      break;
    case 2: // DESCENT
      Serial1.println(packetString);
      Serial2.println(packetString);
      #ifdef SERIAL_DEBUG
        Serial.println(packetString);
      #endif

      // FS3 -> FS4
      if (MISSION_PARAMS.ALTITUDE < 10) {
        MISSION_PARAMS.FLIGHT_STATE = 3; // landing stage
      }
      break;
    case 3: // LANDED
      Serial1.println(packetString);
      Serial2.println(packetString);
      #ifdef SERIAL_DEBUG
        Serial.println(packetString);
      #endif
      if (SENSOR_CNT % 7 == 0) {
        tone(BUZZER, 1000);
      }
      if (SENSOR_CNT % 10 == 0) {
        noTone(BUZZER);
      }
      break;
    default: // IDLE
      #ifdef SERIAL_DEBUG
        Serial.println(packetString);
      #endif
      break;
  };

  // activate LEDs and buzzer
  if (SENSOR_CNT % 7 == 0) {
    digitalWrite(LED, HIGH);
  }
  if (SENSOR_CNT % 10 == 0) {
    digitalWrite(LED, LOW);
    SENSOR_CNT = 0;
  }

  SENSOR_CNT++;
}

// create packet data
String csvParams(struct params MP) {
  String c = ",";
  
  // time variables
  // float ss = floor(MP.MISSION_TIME / 1000);
  // float mm = floor(ss / 60);
  // float hh = floor(mm / 60);
  // ss = ss / 60;
  // mm = (int) mm % 60;
  // char hours[2];
  // char min[2];
  // char sec[5];
  // sprintf(hours, "%2d", hh);
  // sprintf(min, "%2d", mm);
  // sprintf(sec, "%5f", ss);
  // char mTime[12];
  // strcat(mTime,hours);
  // strcat(mTime,":");
  // strcat(mTime,min);
  // strcat(mTime,":");
  // strcat(mTime,sec);

//   return MP.TEAM_ID + c + mTime + c + MP.PACKET_COUNT + c + MP.FLIGHT_STATE + c + MP.PL_STATE + c + MP.ALTITUDE + c + MP.PRESSURE + c + MP.TEMP + c + MP.VOLTAGE + c + MP.GPS_LAT + c + MP.GPS_LON;
// }

  return MP.TEAM_ID + c + MP.MISSION_TIME + c + MP.PACKET_COUNT + c + MP.FLIGHT_STATE + c + MP.PL_STATE + c + MP.ALTITUDE + c + MP.PRESSURE + c + MP.TEMP + c + MP.VOLTAGE + c + MP.GPS_LAT + c + MP.GPS_LON;
}

