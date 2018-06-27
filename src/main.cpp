#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_MAX31865.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/wdt.h>
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
myPID pumpPID(&pidSetpoint, &pidOutput, &pidInput, kP, kI, kD, NORMAL);

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
unsigned long readExtraTempMillis = 0;
unsigned long stateMillis = 0;
unsigned long pumpMillis = 0;
unsigned long primeMillis = 0;
unsigned long comMillis = 0;

// Unions to hold the data that should be transfered over I2C
union {
    float fval;
    byte bval[4];
} tankI2C;
union {
    float fval;
    byte bval[4];
} panelI2C;
union {
    float fval;
    byte bval[4];
} pumpI2C;
union {
    float fval;
    byte bval[4];
} temp0I2C;
union {
    float fval;
    byte bval[4];
} temp1I2C;
union {
    float fval;
    byte bval[4];
} temp2I2C;
union {
    float fval;
    byte bval[4];
} temp3I2C;

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
void readExtraTemps();
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
    pinMode(PUMP_ENABLE_PIN, OUTPUT);

    // Setup the onewire-sensors
    sensors.begin();

    // Startup delay to notice restarts
    delay(STARTUP_DELAY);

    // Setup the watchdog
    wdt_enable(WDTO_8S);
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

    // Read the extra temperature
    if (millis() - readExtraTempMillis >= READ_EXTRA_TEMPS_TIME) {
        readExtraTemps();
    }

    // Runs the PID
    pumpPID.calculate();

    // Sets the pump speed depending on the state and resets the watchdog
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
    panelTemp = panelSensor.temperature(NOMREF, RREF);

    // Calculates the temp difference
    pidInput = panelTemp - tankTemp;

    // Saves the temps in char arrays to send over I2C
    tankI2C.fval = tankTemp;
    panelI2C.fval = panelTemp;

    // Prints to serial if debug is enabled
    #ifdef DEBUG
        Serial.print("Tank: ");
        Serial.print(tankTemp);
        Serial.print(" Panel: ");
        Serial.println(panelTemp);
    #endif
}

//-------------- Reads the extra tepmeratures ----------------------------------

void readExtraTemps() {
  // Saves the time
  readExtraTempMillis = millis();

  // Reads the onewire temps
  sensors.requestTemperatures();
  extraTemp0 = sensors.getTempCByIndex(0);
  extraTemp1 = sensors.getTempCByIndex(1);
  extraTemp2 = sensors.getTempCByIndex(2);
  extraTemp3 = sensors.getTempCByIndex(3);
  // Converts the temps for I2C
  temp0I2C.fval = extraTemp0;
  temp1I2C.fval = extraTemp1;
  temp2I2C.fval = extraTemp2;
  temp3I2C.fval = extraTemp3;
  // Prints to serial if debug is enabled
  #ifdef DEBUG
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
    pumpI2C.fval = pumpSpeed;
    analogWrite(PUMP_PIN, pumpSpeed*2.55);

    // Resets the watchdog
    wdt_reset();

    // Prints to serial monitor if debug is enabled
    #ifdef DEBUG
        Serial.print("Pump: ");
        Serial.println(pumpSpeed);
    #endif
}

//-------------- Writes data to I2C for logging --------------------------------

void sendData() {
  if (millis() - comMillis >= MIN_COM_TIME) {
      comMillis = millis();
      // Write the temperatures to I2C
      Wire.write(tankI2C.bval, 4);
      Wire.write(panelI2C.bval, 4);
      Wire.write(pumpI2C.bval, 4);
      Wire.write(temp0I2C.bval, 4);
      Wire.write(temp1I2C.bval, 4);
      Wire.write(temp2I2C.bval, 4);
      Wire.write(temp3I2C.bval, 4);
  }
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
