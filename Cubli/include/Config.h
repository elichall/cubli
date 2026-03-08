#ifndef CONFIG_H
#define CONFIG_H

// Debug State
#define DEBUG_MODE // Comment out after testing

// Constants
static constexpr float DEG_TO_RAD_F = (float)(DEG_TO_RAD);
enum Axis { 
    X = 0,
    Y = 1, 
    Z = 2 
};
enum OrientationState {
    FACE = 0,
    EDGE = 1,
    CORNER = 2
};

const float MOTOR_RESISTANCE = 16.4; // ohms
const float MOTOR_RATING = 90;
const float TORQUE_CONSTANT = 60 / (2 * PI * MOTOR_RATING);

const float ANGLE_LIMIT = 9 * DEG_TO_RAD;

const int SINGLE_AXIS_STATE_VECTOR_SIZE = 4;

// Multiplexor Address
const byte MUX_ADD = 0x70;

// Encoder MUX connections
const byte SENSOR_CHAN[3] = { 2 , 3 , 6 };  

// I2C GPIO pins
const byte PIN_SDA = 23;
const byte PIN_SCL = 22;

// Driver GPIO Pins
const byte PIN_EN = 33;
const byte PIN_X_IN1 = 25;
const byte PIN_X_IN2 = 26;
const byte PIN_X_IN3 = 27;

const byte PIN_Z_IN1 = 18;
const byte PIN_Z_IN2 = 17;
const byte PIN_Z_IN3 = 5;

const byte PIN_Y_IN1 = 16;
const byte PIN_Y_IN2 = 15;
const byte PIN_Y_IN3 = 2;

// Power
const float BATTERY_VOLTAGE = 11.1;
const float VOLTAGE_LIMIT = 5.0;
const float ALIGNMENT_VOLTAGE = 3.0;

// Communication
const int SERIAL_COM = 115200;
const int MOTOR_POLE_PAIRS = 7;

#endif