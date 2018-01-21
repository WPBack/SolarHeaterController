#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_MAX31865.h>
#include <Wire.h>

// Debug-mode
#define DEBUG

// Pin-defines
#define TANK_CS   10  //Tank chip select pin
#define PANEL_CS  11  //Panel chip select pin

// MAX31865 sensor settings
#define RREF 4300.0
#define NOMREF 1000

// I2C settings
#define SLAVE_ADDRESS 0x04

// Settings for how often the stuff should run
#define READ_TEMPS_TIME 10000

// MAX31865 sensors
Adafruit_MAX31865 tankSensor = Adafruit_MAX31865(TANK_CS);
Adafruit_MAX31865 panelSensor = Adafruit_MAX31865(PANEL_CS);

//Variables to hold the temperatures
double tankTemp = 0;
double panelTemp = 0;

//Variable to hold the last time it was reading the temperature
unsigned long readTempMillis = 0;

// Arrays to hold the data that should be transfered over I2C
char tankI2C[6];
char panelI2C[6];

// Method to send the data to the raspberry for datalogging
void sendData();


void setup() {
  // Serial for debug if enabled
  #ifdef DEBUG
    Serial.begin(115200);
    Serial.println("Program started!");
  #endif

  // Setting up the temp sensors
  tankSensor.begin(MAX31865_2WIRE);
  panelSensor.begin(MAX31865_2WIRE);

  // Setting up the I2C-bus
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(sendData);
}


void loop() {

  // Read the temperature every 10 seconds
  if (millis() - readTempMillis > READ_TEMPS_TIME) {
    // Read the temps
    tankTemp = tankSensor.temperature(NOMREF, RREF);
    panelTemp = tankSensor.temperature(NOMREF, RREF);

    // Saves the temps in char arrays to send over I2C
    dtostrf(tankTemp, 6, 2, tankI2C);
    dtostrf(panelTemp, 6, 2, panelI2C);

    // Saves the time
    readTempMillis = millis();

    // Prints to serial if debug is enabled
    #ifdef DEBUG
      Serial.print("Tank: ");
      Serial.print(tankTemp);
      Serial.print(" Panel: ");
      Serial.println(panelTemp);
    #endif
  }
}

void sendData() {
    // Write the temperatures to I2C
    Wire.write(tankI2C);
    Wire.write(panelI2C);
  }
