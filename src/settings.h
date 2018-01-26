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
