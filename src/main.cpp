/***************************************************
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_MAX31865.h>
#include <Wire.h>

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 max = Adafruit_MAX31865(10);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31865 max = Adafruit_MAX31865(10);

//Variable to hold the temperature
double lastTemp = 0;

//Variable to hold the last time it was reading the temperature
unsigned long readMillis = 0;

// The value of the Rref resistor. Use 430.0!
#define RREF 4300.0
#define NOMREF 1000
#define SLAVE_ADDRESS 0x04

void sendData();

void setup() {
  Serial.begin(115200);
  Serial.println("Adafruit MAX31865 PT100 Sensor Test!");

  max.begin(MAX31865_2WIRE);  // set to 2WIRE or 4WIRE as necessary
  Wire.begin(SLAVE_ADDRESS);

  Wire.onRequest(sendData);
}


void loop() {
  // Read the temperature every 10 seconds
  if (millis() - readMillis > 10000) {
    lastTemp = max.temperature(NOMREF, RREF);
  }
}

void sendData() {
    Wire.write((int)lastTemp);
  }
