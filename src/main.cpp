#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_MAX31865.h>
#include <Wire.h>
#include <PID_v1.h>

// Debug-mode
#define DEBUG

// Pin-defines
#define TANK_CS   10  //Tank chip select pin
#define PANEL_CS  11  //Panel chip select pin
#define PUMP_PIN  3

// MAX31865 sensor settings
#define RREF 4300.0
#define NOMREF 1000

// I2C settings
#define SLAVE_ADDRESS 0x04

// Settings for how often the stuff should run
#define READ_TEMPS_TIME 10000
#define PID_TIME 10000

// Settings for the PID-controller
#define kP 5
#define kI 1
#define kD 0
#define SETPOINT 2
#define PID_MIN 14
#define PID_MAX 100

// MAX31865 sensors
Adafruit_MAX31865 tankSensor = Adafruit_MAX31865(TANK_CS);
Adafruit_MAX31865 panelSensor = Adafruit_MAX31865(PANEL_CS);

// PID for the pump speed
double setpoint = SETPOINT;
double input = 0;
double output = 0;
PID pumpPID(&setpoint, &output, &input, kP, kI, kD, DIRECT);

//Variables to hold the temperatures
double tankTemp = 0;
double panelTemp = 0;

//Variable to hold the last time it was doing something
unsigned long readTempMillis = 0;
unsigned long pidMillis = 0;

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

  // Setup the PID
  pumpPID.SetMode(AUTOMATIC);
  pumpPID.SetOutputLimits(PID_MIN, PID_MAX);

  // Setup the pump-pin
  pinMode(PUMP_PIN, OUTPUT);
}


void loop() {

  // Read the temperature
  if (millis() - readTempMillis >= READ_TEMPS_TIME) {

    // Saves the time
    readTempMillis = millis();

    // Read the temps
    tankTemp = tankSensor.temperature(NOMREF, RREF);
    panelTemp = tankSensor.temperature(NOMREF, RREF);

    // Saves the temps in char arrays to send over I2C
    dtostrf(tankTemp, 6, 2, tankI2C);
    dtostrf(panelTemp, 6, 2, panelI2C);

    // Prints to serial if debug is enabled
    #ifdef DEBUG
      Serial.print("Tank: ");
      Serial.print(tankTemp);
      Serial.print(" Panel: ");
      Serial.println(panelTemp);
    #endif
  }

  //compute the PID
  if (millis() - pidMillis >= PID_TIME) {

    // Saves the time
    pidMillis = millis();

    // Calculates the difference
    input = panelTemp - tankTemp;

    // Runs the PID
    pumpPID.Compute();

    // Writes the speed to the pin that controls the pump
    analogWrite(PUMP_PIN, output*2.55);
  }
}

void sendData() {
    // Write the temperatures to I2C
    Wire.write(tankI2C);
    Wire.write(panelI2C);
  }
