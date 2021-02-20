#include <Wire.h>
#include <ESP8266WiFiMulti.h>
#include <InfluxDbClient.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include "settings.h"
#include "settings_private.h" // Copy from settings_private_example.h

// Debug-mode
//#define DEBUG

//-------------- Global variables ----------------------------------------------

// WiFi
ESP8266WiFiMulti wifiMulti;

// InfluxDB-client
InfluxDBClient influxClient(INFLUXDB_URL, INFLUXDB_DB_NAME);

// InfluxDB data-point
Point influxDataPoint(INFLUXDB_POINT_NAME);

// Time (used for sending priming request)
WiFiUDP ntpUDP;
NTPClient ntpClient(ntpUDP);

// Variable to hold the last time it was doing something
unsigned long requestDataMillis = 0;

// Next priming time in epoch time
unsigned long nextPrimeTimeEprochTime = 0;

// Unions to hold the data that received over I2C
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

//-------------- Help-methods --------------------------------------------------

void requestData();
void checkPrimeTime();

//-------------- Setup function ------------------------------------------------

void setup() {
  // Setup built in LED to blink when transmitting data
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Serial for debug if enabled
  #ifdef DEBUG
    Serial.begin(115200);
    Serial.println("Program started!");
  #endif

  // Set up the I2C-bus
  Wire.begin(D1, D2);

  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
  wl_status_t wifiStatus = wifiMulti.run();
  while(wifiStatus != WL_CONNECTED) {
    #ifdef DEBUG
      Serial.print("Wifi connection failed: ");
      Serial.println(wifiStatus);
    #endif
    delay(100);
    wifiStatus = wifiMulti.run();

    // Toggle LED
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  #ifdef DEBUG
    Serial.println("Wifi connected");
  #endif

  // Check InfluxDB connection
  while(!influxClient.validateConnection()) {
    #ifdef DEBUG
      Serial.print("InfluxDB connection failed: ");
      Serial.println(influxClient.getLastErrorMessage());
    #endif
    delay(100);

    // Toggle LED
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  #ifdef DEBUG
    Serial.println("InfluxDB connected");
  #endif

  // Setup NTP time
  ntpClient.begin();
  ntpClient.update();
  #ifdef DEBUG
    Serial.println(ntpClient.getFormattedTime());
  #endif
}

//-------------- Main loop -----------------------------------------------------

void loop() {
  // Update time
  ntpClient.update();

  // Check if priming should be requested
  checkPrimeTime();
  
  // Request data from I2C
  if (millis() - requestDataMillis >= REQUEST_DATA_TIME) {
    requestData();
  }
}

//-------------- Requsets data from the I2C slave ------------------------------

void requestData() {
  // Saves the time
  requestDataMillis = millis();
  
  // Request the data
  Wire.requestFrom(SLAVE_ADDRESS, I2C_NUM_BYTES);

  // Put the data into the unions
  int i = 0;
  while(Wire.available()) {
    if(i < 4) {
      tankI2C.bval[i % 4] = Wire.read();
    }
    else if(i < 8) {
      panelI2C.bval[i % 4] = Wire.read();
    }
    else if(i < 12) {
      pumpI2C.bval[i % 4] = Wire.read();
    }
    else if(i < 16) {
      temp0I2C.bval[i % 4] = Wire.read();
    }
    else if(i < 20) {
      temp1I2C.bval[i % 4] = Wire.read();
    }
    else if(i < 24) {
      temp2I2C.bval[i % 4] = Wire.read();
    }
    else if(i < 28) {
      temp3I2C.bval[i % 4] = Wire.read();
    }

    i++;
  }

  // Print to serial monitor if in debug mode
  #ifdef DEBUG
    Serial.print("Tank: ");
    Serial.print(tankI2C.fval);
    Serial.print(" Panel: ");
    Serial.print(panelI2C.fval);
    Serial.print(" Pump: ");
    Serial.print(pumpI2C.fval);
    Serial.print(" Temp 0: ");
    Serial.print(temp0I2C.fval);
    Serial.print(" Temp 1: ");
    Serial.print(temp1I2C.fval);
    Serial.print(" Temp 2: ");
    Serial.print(temp2I2C.fval);
    Serial.print(" Temp 3: ");
    Serial.println(temp3I2C.fval);
  #endif

  // Log to InfluxDB
  influxDataPoint.clearFields();
  influxDataPoint.addField("Tank",  tankI2C.fval);
  influxDataPoint.addField("Panel", panelI2C.fval);
  influxDataPoint.addField("Pump",  pumpI2C.fval);
  influxDataPoint.addField("Temp0", temp0I2C.fval);
  influxDataPoint.addField("Temp1", temp1I2C.fval);
  influxDataPoint.addField("Temp2", temp2I2C.fval);
  influxDataPoint.addField("Temp3", temp3I2C.fval);

  // Try to re-start wifi if logging fails
  while(!influxClient.writePoint(influxDataPoint)) {
    #ifdef DEBUG
      Serial.print("InfluxDB write failed: ");
      Serial.println(influxClient.getLastErrorMessage());
    #endif
    if(wifiMulti.run() != WL_CONNECTED) {
      #ifdef DEBUG
        Serial.println("Wifi connection failed");
      #endif
    }
    delay(100);

    // Toggle LED
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // Toggle the built-in LED
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

//-------------- Check priming time --------------------------------------------

void checkPrimeTime() {
  // Check if priming should be performed
  unsigned long epochTime = ntpClient.getEpochTime();
  if(epochTime >= nextPrimeTimeEprochTime) {
    // Request priming
    Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.write(PRIME_CMD);
    Wire.endTransmission();

    // Schedule next priming
    nextPrimeTimeEprochTime = (epochTime/86400L)*86400L + 86400L + PRIME_H*3600L + PRIME_M*60;
    #ifdef DEBUG
      Serial.print("Priming requested. Next priming in: ");
      Serial.print(nextPrimeTimeEprochTime - epochTime);
      Serial.println("s");
    #endif
  }
}
