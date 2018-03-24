#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_MAX31865.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
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

// OneWire-sensors
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature sensors(&oneWire);

//Variables to hold the temperatures
double tankTemp = 0;
double panelTemp = 0;
double extraTemp0 = 0;
double extraTemp1 = 0;
double extraTemp2 = 0;
double extraTemp3 = 0;

// Variable to hold the pump speed
double pumpSpeed = 0;

//Variable to hold the last time it was doing something
unsigned long readTempMillis = 0;
unsigned long stateMillis = 0;
unsigned long pumpMillis = 0;
unsigned long primeMillis = 0;

// Arrays to hold the data that should be transfered over I2C
char tankI2C[6];
char panelI2C[6];
char pumpI2C[6];
char temp0I2C[6];
char temp1I2C[6];
char temp2I2C[6];
char temp3I2C[6];

// Enum to hold the state
enum State {
    waiting,
    priming,
    running
};
State state = waiting;

//-------------- Help-methods --------------------------------------------------

void sendData();
void receiveData(int number);
void updateState();
void readTemps();
void runPump();
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
    Wire.onReceive(receiveData);

    // Setup the PID
    pumpPID.setOutputLimits(PWM_MIN, PWM_MAX);
    pumpPID.setSampleTime(PID_TIME);
    pumpPID.setDeadBand(PID_DB_MIN, PID_DB_PLUS);

    // Setup the pump-pin
    pinMode(PUMP_PIN, OUTPUT);

    // Setup the onewire-sensors
    sensors.begin();
}

//-------------- Main loop -----------------------------------------------------

void loop() {

    // Checks if the state should be changed
    if (millis() - stateMillis >= STATE_TIME) {
        updateState();
    }

    // Read the temperature
    if (millis() - readTempMillis >= READ_TEMPS_TIME) {
        readTemps();
    }

    // Runs the PID
    pumpPID.calculate();

    // Sets the pump speed depending on the state
    if (millis() - pumpMillis >= PUMP_TIME) {
        runPump();
    }
}

//-------------- Updates the state ---------------------------------------------

void updateState() {
    // Save the time
    stateMillis = millis();

    // If the state is waiting we should prime if the temp difference is large
    if (state == waiting) {
        if (pidInput > STARTTEMP) {
            state = priming;
            primeMillis = millis();
            #ifdef DEBUG
                Serial.println("Priming...");
            #endif
        }
    }
    // If we are priming we wait for a specific priming time and then run
    else if (state == priming) {
        if (millis() - primeMillis >= PRIMETIME) {
            state = running;
            #ifdef DEBUG
                Serial.println("Running...");
            #endif
        }
    }
    // Otherwise we are running, and should stop if the temp gets low
    else {
        if (pidInput < STOPTEMP) {
            state = waiting;
            #ifdef DEBUG
                Serial.println("Waiting...");
            #endif
        }
    }
}

//-------------- Reads the tepmeratures ----------------------------------------

void readTemps() {
    // Saves the time
    readTempMillis = millis();

    // Read the temps
    tankTemp = tankSensor.temperature(NOMREF, RREF);
    panelTemp = tankSensor.temperature(NOMREF, RREF);

    // Reads the onewire temps
    sensors.requestTemperatures();
    extraTemp0 = sensors.getTempCByIndex(0);
    extraTemp1 = sensors.getTempCByIndex(1);
    extraTemp2 = sensors.getTempCByIndex(2);
    extraTemp3 = sensors.getTempCByIndex(3);

    // Calculates the temp difference
    pidInput = panelTemp - tankTemp;

    // Saves the temps in char arrays to send over I2C
    dtostrf(tankTemp, 6, 2, tankI2C);
    dtostrf(panelTemp, 6, 2, panelI2C);
    dtostrf(extraTemp0, 6, 2, temp0I2C);
    dtostrf(extraTemp1, 6, 2, temp1I2C);
    dtostrf(extraTemp2, 6, 2, temp2I2C);
    dtostrf(extraTemp3, 6, 2, temp3I2C);

    // Prints to serial if debug is enabled
    #ifdef DEBUG
        Serial.print("Tank: ");
        Serial.print(tankTemp);
        Serial.print(" Panel: ");
        Serial.print(panelTemp);
        Serial.print(" Extra0: ");
        Serial.print(extraTemp0);
        Serial.print(" Extra1: ");
        Serial.print(extraTemp2);
        Serial.print(" Extra2: ");
        Serial.print(extraTemp2);
        Serial.print(" Extra3: ");
        Serial.println(extraTemp3);
    #endif
}

//-------------- Runs the pump according to the state --------------------------

void runPump() {
    // Saves the time
    pumpMillis = millis();

    // Sets the correct pump speed according to the state
    if (state == waiting) {
        pumpSpeed = 0;
        digitalWrite(PUMP_ENABLE_PIN, false);
    }
    else if (state == priming) {
        pumpSpeed = PWM_MAX;
        digitalWrite(PUMP_ENABLE_PIN, true);
    }
    else {
        pumpSpeed = pidOutput;
        digitalWrite(PUMP_ENABLE_PIN, true);
    }

    // Save the pump speed for I2C and set the pin
    dtostrf(pumpSpeed, 6, 2, pumpI2C);
    analogWrite(PUMP_PIN, pumpSpeed*2.55);

    // Prints to serial monitor if debug is enabled
    #ifdef DEBUG
        Serial.print("Pump: ");
        Serial.println(pumpSpeed);
    #endif
}

//-------------- Writes data to I2C for logging --------------------------------

void sendData() {
    // Write the temperatures to I2C
    Wire.write(tankI2C);
    Wire.write(panelI2C);
    Wire.write(pumpI2C);
    Wire.write(temp0I2C);
    Wire.write(temp1I2C);
    Wire.write(temp2I2C);
    Wire.write(temp3I2C);
}

//-------------- Receive I2C-data. If it is 1, start the pump ------------------

void receiveData(int number) {
    while(Wire.available()) {
        int data = Wire.read();

        #ifdef DEBUG
            Serial.println(data);
        #endif

        if(data == 1) {
            state = priming;
            primeMillis = millis();
            #ifdef DEBUG
                Serial.println("I2C requested prime");
            #endif
        }
    }
}
