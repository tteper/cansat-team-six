// sensor testing code
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_GPS.h>
#include <String>
#include <SoftwareSerial.h>

// configure communication types here
#define GPSSerial Serial
#define SEALEVELPRESSURE_HPA (1013.25)

uint32_t TIMESTAMP = millis();
uint32_t START_TIME;
Adafruit_BMP3XX BMP;
Adafruit_GPS GPS(&GPSSerial);
unsigned int FLIGHT_STATE = 0;
bool MISSION_START = false;

// data struct for mission information
struct {
  const int TEAM_ID = 1006;
  uint32_t MISSION_TIME;
  unsigned int PACKET_COUNT = 0;
  unsigned int FLIGHT_STATE = 0;
  unsigned int PL_STATE = 0;
  int ALTITUDE;
  float TEMP;
  float VOLTAGE;
  double GPS_LAT;
  double GPS_LON;
} MISSION_PARAMS;

void setup() {

  // Configure serial ports on computer
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Sensor test: BMP, GPS, and SD Card");

  // configure bmp sensor
  Serial.println("Initializing BMP");
  if (!BMP.begin_I2C())
  {
    Serial.println("BMP Connection Failed: Check Wiring!");
    system("pause");
    return;
  }

  BMP.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  BMP.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  BMP.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  BMP.setOutputDataRate(BMP3_ODR_50_HZ);

  // configure GPS sensor
  Serial.println("Initializing GPS");
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ); // 5 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  // configure SD card reader
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
}

void loop() {
  // set 5Hz refresh rate
  delay(200);
  
  // configure file write as first action
  if (!MISSION_START) {
    // basic configuration variables
    String dataString = "";
    START_TIME = TIMESTAMP;

    // SD: read three sensors and append to the string:
    for (int analogPin = 0; analogPin < 3; analogPin++) {
      int sensor = analogRead(analogPin);
      dataString += String(sensor);
      if (analogPin < 2) {
        dataString += ",";
      }
    }

    // Open file
    File dataFile = SD.open("cansat-data.csv", FILE_WRITE);
    String header = "TEAM_ID, MISSION_TIME, PACKET_COUNT, FLIGHT_STATE, PL_STATE, ALTITUDE, TEMP, VOLTAGE, GPS_LAT, GPS_LONG";

    // Write header
    if (dataFile)
      dataFile.println(header);
    else 
      return;

    // set mission to start
    MISSION_START = true;
  }
  else {

    // update mission_params
    MISSION_PARAMS.MISSION_TIME = TIMESTAMP - START_TIME;
    MISSION_PARAMS.PACKET_COUNT++;
    MISSION_PARAMS.ALTITUDE = BMP.readAltitude(SEALEVELPRESSURE_HPA);
    MISSION_PARAMS.TEMP = BMP.readTemperature();
    // MISSION_PARAMS.VOLTAGE = something
    MISSION_PARAMS.GPS_LAT = GPS.latitude;
    MISSION_PARAMS.GPS_LON = GPS.longitude;
    dataString = csvParams(MISSION_PARAMS);

    // main execution code
    switch (MISSION_PARAMS.FLIGHT_STATE) {
      case 0: // IDLE
        break;
      case 1: // ASCENT
        dataFile.println(dataString);
        Serial2.println(dataString);
        break;
      case 2: // DROP
        dataFile.println(dataString);
        Serial2.println(dataString);
        break;
      case 3: // DESCENT
        dataFile.println(dataString);
        Serial2.println(dataString);
        break;
      case 4: // LANDING
        dataFile.close();
      default: // IDLE
        break;
    };
  }

  // FS1 -> FS2
  if (MISSION_PARAMS.ALTITUDE > 400) {
    MISSION_PARAMS.FLIGHT_STATE = 2; // drop stage
    // activate servo
  }
  // FS2 -> FS3 -> FS4
  else if () {}

  // FS4 -> END
  else if () {}

  // DEFAULT
  else

}

// create packet data
String csvParams(struct MP) {
  String c = ",";
  return MP.TEAM_ID + c + MP.MISSION_TIME + c + MP.PACKET_COUNT + c + MP.FLIGHT_STATE + c + MP.PL_STATE + c + MP.ALTITUDE + c + MP.TEMP + c + MP.VOLTAGE + c + MP.GPS_LAT + c + MP.GPS_LON + "\n";
}

