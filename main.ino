/**
 *  CANSAT Team 6 Flight Software
 *  Two Month, Fall 2022
 *  
 *  Author(s): Thomas Teper, Ian Kramer
 *
 *  Copyright (c) Space Hardware Club, 2022
 */

#define VERSION 1.0

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

// configure constants
#define SEALEVELPRESSURE_HPA 1013.25
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_Q_RELEASE "$PMTK605*31"

// mission params
#define REFRESH_RATE 1000 / 1 // ms; 5 Hz
#define ALTITUDE_MAX 400 // altitude to trigger latch

// DEBUG flag
#define SERIAL_DEBUG true

// global mission variables
uint32_t TIMESTAMP = millis(); // time since POWER ON
uint32_t START_TIME;           // mission start time
unsigned int FLIGHT_STATE = 0; // current flight state (DEFAULT: 0)
float ALT_START;               // variable to hold starting altitude -> calc relative altitude

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
    system("pause");
    return;
  }
  // oversampling and filter init
  BMP.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  BMP.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  BMP.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  BMP.setOutputDataRate(BMP3_ODR_50_HZ);
  ALT_START = BMP.readAltitude(SEALEVELPRESSURE_HPA); // define altitude datum

  // basic configuration variables
  String packetString = ""; // packet string
  START_TIME = TIMESTAMP;   // set the start time to current time for mission start

  // Create file header
  String header = "TEAM_ID, MISSION_TIME, PACKET_COUNT, FLIGHT_STATE, PL_STATE, ALTITUDE, TEMP, VOLTAGE, GPS_LAT, GPS_LONG";

  // Write header to SD
  Serial2.println(header);

  // Servo configuration
  SERVO.attach(10); // set servo pin
}

void loop() {
  
  // set refresh rate (DEFAULT: 5Hz)
  delay(REFRESH_RATE);

  // create write string and mission params struct
  String packetString;

  // update mission_params
  MISSION_PARAMS.MISSION_TIME = TIMESTAMP - START_TIME;
  MISSION_PARAMS.PACKET_COUNT++;
  MISSION_PARAMS.ALTITUDE = BMP.readAltitude(SEALEVELPRESSURE_HPA);// - ALT_START;
  MISSION_PARAMS.PRESSURE = BMP.readPressure();
  MISSION_PARAMS.TEMP = BMP.readTemperature();
  MISSION_PARAMS.VOLTAGE = 0.0;
  MISSION_PARAMS.GPS_LAT = GPS.latitude;
  MISSION_PARAMS.GPS_LON = GPS.longitude;
  packetString = csvParams(MISSION_PARAMS);

  // main execution code
  switch (MISSION_PARAMS.FLIGHT_STATE) {
    case 0: // IDLE
      Serial1.println(packetString);
      MISSION_PARAMS.FLIGHT_STATE = 1;
      #ifdef SERIAL_DEBUG
        Serial.println(packetString);
      #endif
      // blink LED
      break;
    case 1: // ASCENT
      Serial1.println(packetString);
      Serial2.println(packetString);
      #ifdef SERIAL_DEBUG
        Serial.println(packetString);
      #endif
      // blink LED
      
      // FS1 -> FS2
      if (MISSION_PARAMS.ALTITUDE > ALTITUDE_MAX) {
        MISSION_PARAMS.FLIGHT_STATE = 2; // drop stage
      }
      break;
    case 2: // DROP
      Serial1.println(packetString);
      Serial2.println(packetString);
      #ifdef SERIAL_DEBUG
        Serial.println(packetString);
      #endif
      // blink LED

      // fill in with servo code

      // FS2 -> FS3
      if (SERVO.read() > 1000) {
        MISSION_PARAMS.FLIGHT_STATE = 3; // descent stage
        MISSION_PARAMS.PL_STATE = 'R';   // update payload state to "Released"
      }
      break;
    case 3: // DESCENT
      Serial1.println(packetString);
      Serial2.println(packetString);
      #ifdef SERIAL_DEBUG
        Serial.println(packetString);
      #endif
      // blink LED

      // FS3 -> FS4
      if (MISSION_PARAMS.ALTITUDE < 10) {
        MISSION_PARAMS.FLIGHT_STATE = 4; // landing stage
      }
      break;
    case 4: // LANDING
      Serial1.println(packetString);
      #ifdef SERIAL_DEBUG
        Serial.println(packetString);
      #endif

      break;
    default: // IDLE
      #ifdef SERIAL_DEBUG
        Serial.println(packetString);
      #endif
      break;
  };
}

// create packet data
String csvParams(struct params MP) {
  String c = ",";

  // strings to store data
  char teamID[4];
  char missionTime[11];
  char packetCount[5];
  char flightState[1];
  char plState[1];
  char altitude[5];
  char temp[4];
  char voltage[3];
  char lat[10];
  char lon[10];

  return MP.TEAM_ID + c + MP.MISSION_TIME + c + MP.PACKET_COUNT + c + MP.FLIGHT_STATE + c + MP.PL_STATE + c + MP.ALTITUDE + c + MP.PRESSURE + c + MP.TEMP + c + MP.VOLTAGE + c + MP.GPS_LAT + c + MP.GPS_LON;
}

