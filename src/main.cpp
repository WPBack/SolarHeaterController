#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_MAX31865.h>
#include <Wire.h>
#include "settings.h"
#include "myPID.h"

// Debug-mode
#define DEBUG

//-------------- Global variables ----------------------------------------------

// MAX31865 sensors
Adafruit_MAX31865 tankSensor = Adafruit_MAX31865(TANK_CS);
Adafruit_MAX31865 panelSensor = Adafruit_MAX31865(PANEL_CS);

// PID for the pump speed
double pidSetpoint = SETPOINT;
double pidInput = 0;
double pidOutput = 0;
myPID pumpPID(&pidSetpoint, &pidOutput, &pidInput, kP, kI, kD, REVERSE);

//Variables to hold the temperatures
double tankTemp = 0;
double panelTemp = 0;

//Variable to hold the last time it was doing something
unsigned long readTempMillis = 0;
unsigned long pidMillis = 0;

// Arrays to hold the data that should be transfered over I2C
char tankI2C[6];
char panelI2C[6];
char pumpI2C[6];

// Enum to hold the state
enum State {
  waiting,
  priming,
  running
};

//-------------- Help-methods --------------------------------------------------

void sendData();
void readTemps();
void runPID();

//-------------- Setup function ------------------------------------------------

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
  pumpPID.setOutputLimits(PWM_MIN, PWM_MAX);
  pumpPID.setSampleTime(PID_TIME);
  pumpPID.setDeadBand(PID_DB_MIN, PID_DB_PLUS);

  // Setup the pump-pin
  pinMode(PUMP_PIN, OUTPUT);
}

//-------------- Main loop -----------------------------------------------------

void loop() {

  // Read the temperature
  if (millis() - readTempMillis >= READ_TEMPS_TIME) {
    readTemps();
  }

  // Runs the PID
  pumpPID.calculate();
}

//-------------- Reades the tepmeratures ---------------------------------------

void readTemps() {
  // Saves the time
  readTempMillis = millis();

  // Read the temps
  tankTemp = tankSensor.temperature(NOMREF, RREF);
  panelTemp = tankSensor.temperature(NOMREF, RREF);

  // Calculates the temp difference
  pidInput = panelTemp - tankTemp;

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

// Writes data to I2C for logging
void sendData() {
    // Write the temperatures to I2C
    Wire.write(tankI2C);
    Wire.write(panelI2C);
    Wire.write(pumpI2C);
  }
