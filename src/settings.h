// Tuning parameters for the PID-controller
#define kP 5
#define kI 0.1
#define kD 0
#define PID_TIME    1000
#define PID_DB_MIN  0.5
#define PID_DB_PLUS 0.5

// General settins
#define STARTTEMP 5     //Temp to start at
#define SETPOINT  10    //Wanted temp difference
#define STOPTEMP  2     //Temp to stop at
#define PWM_MIN   14    //Minimum pump speed
#define PWM_MAX   100   //Maximum pump speed
#define PRIMETIME 5000  //The time to prime (in millisecond)

// Pin-settings
#define TANK_CS         10  //Tank chip select pin
#define PANEL_CS        9   //Panel chip select pin
#define PUMP_PIN        3   //Pump PWM pin
#define PUMP_ENABLE_PIN 2   //Pin for the pump relay

// MAX31865 sensor settings
#define RREF 4300.0
#define NOMREF 1000

// I2C settings
#define SLAVE_ADDRESS 0x04

// Settings for how often the stuff should run
#define READ_TEMPS_TIME 1000
#define STATE_TIME 1000
#define PUMP_TIME 500
