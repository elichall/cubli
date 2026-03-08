#include "Cubli.h"
#include "gains.h"

Cubli::Cubli() : 
    mpu(Wire),
    drivers{ {PIN_X_IN1, PIN_X_IN2, PIN_X_IN3, PIN_EN},                     // Driver: DRV8313 (IN1, IN2, IN3, EN)
             {PIN_Y_IN1, PIN_Y_IN2, PIN_Y_IN3, PIN_EN},
             {PIN_Z_IN1, PIN_Z_IN2, PIN_Z_IN3, PIN_EN} },
    motors{ {MOTOR_POLE_PAIRS}, {MOTOR_POLE_PAIRS}, {MOTOR_POLE_PAIRS} },   // Motor: 7 Pole Pairs
    sensors{ {AS5600_I2C}, {AS5600_I2C}, {AS5600_I2C} }                     // Sensor: AS5600 Magnetic Encoder
{}

Cubli::Orientation::Orientation() : 
    crossReferenceTable({
        { {FACE, 0},    {0, 0, -1} },
        { {FACE, 1},    {0, 0, 1} },
        { {EDGE, 0},    {-sqrtf(2)/2, 0, -sqrtf(2)/2} },
        { {CORNER, 0},  {-sqrtf(3)/3, -sqrtf(3)/3, -sqrtf(3)/3} }
    }),
    nbodyToVbodyTable({ 
        { {FACE, 0},    {1, 0, 0,
                         0, 1, 0,
                         0, 0, 1} },
        { {EDGE, 0},    {1,  0,  0,
                         0,  1,  0,
                         0,  0,  1} },
        { {CORNER, 0},  {1, 0, 0,
                         0, 1, 0,
                         0, 0, 1} }
    }),
    gainTable({
        { {FACE, 0},    &gainZero },
        { {FACE, 1},    &gainZero },
        { {FACE, 2},    &gainZero },
        { {FACE, 3},    &gainZero },
        { {FACE, 4},    &gainZero },
        { {FACE, 5},    &gainZero },
        { {EDGE, 0},    &gainEdgeMax },
        { {CORNER, 0},  &gainCornerMax }
    })
{}