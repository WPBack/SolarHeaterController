// Tuning parameters for the PID-controller
#define kP 5
#define kI 1
#define kD 0

// General settins
#define STARTTEMP 5   //Temp to start at
#define SETPOINT  10  //Wanted temp difference
#define STOPTEMP  2   //Temp to stop at
#define PWM_MIN   14  //Minimum pump speed
#define PWM_MAX   100 //Maximum pump speed

// Pin-settings
#define TANK_CS   10  //Tank chip select pin
#define PANEL_CS  11  //Panel chip select pin
#define PUMP_PIN  3   //Pump PWM pin

// MAX31865 sensor settings
#define RREF 4300.0
#define NOMREF 1000

// I2C settings
#define SLAVE_ADDRESS 0x04

// Settings for how often the stuff should run
#define READ_TEMPS_TIME 5000
#define PID_TIME 5000
