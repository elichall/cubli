#ifndef CUBLI_H
#define CUBLI_H

#include <Arduino.h>
#include <MPU6050_light.h>
#include <SimpleFOC.h>
#include <BasicLinearAlgebra.h> 
#include <map>
#include <vector>
#include "Config.h"

#ifdef DEBUG_MODE
  #define DEBUG_BEGIN(x) Serial.begin(x)
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_BEGIN(x)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

using namespace std;

// --- V1 Software ---
/* Future Optimizations and V2 Changes
-> Fixed point Arithmetic 
    - Move from foats to ints for compuitation
    - scale all floats by factor and then do math in uint32_t, 
      then scale back to float for applying voltages etc.

-> Parallel Processing
    - ESP32 has two cores
    - keeps the state vector upkeep math on one core and the lqr control 
      loops on another and pass the state vector information between them

-> Direct Registar Access
    - idk much about it but using "Wire" has overhead in the form of 
      safety checks and abstraction 
    - using dirct registar manip for I2C is 1 cpu cycle while 
      "digitalWrite" is 50

-> Map to Matrix of Pointers
    - Change map structures to an array of pointers 
    - O(log N) to O(1)
      
      EX: 
        in structure definition 
        BLA::Matrix<3,12>* gainLUT[3][12]; // (LUT look up table)

        in constructor
        // Initialize everything to Zero first for safety
            for(int s=0; s<3; s++) 
                for(int i=0; i<12; i++) 
                    gainLUT[s][i] = &gainZero;

        gainLUT[EDGE][0] = &gainEdgeMax;
        gainLUT[EDGE][1] = &gainEdgeMin;
        gainLUT[EDGE][2] = &gainEdgeMax; // lets you share values

-> Finite State Machine (FSM) Archetecture
    - enumeratue a set of system states and loop through a switch/case 
      for what to do in each system state
    - update the system state using updateOrientation

    --- Gemini Example ---
    enum SystemState {
        IDLE,
        CALIBRATING,
        BALANCING_EDGE,
        BALANCING_CORNER,
        ERROR
    };

    SystemState currentState = IDLE;

    void loop() {
        // 1. Read Global Sensors (Always happens)
        readSensors();

        // 2. State Machine Logic
        switch (currentState) {
            case IDLE:
                if (userCommand == "start") currentState = BALANCING_EDGE;
                motor.move(0);
                break;

            case BALANCING_EDGE:
                if (abs(theta) > LIMIT) currentState = ERROR;
                torque = calculateLQR();
                motor.move(torque);
                break;

            case ERROR:
                motor.move(0);
                blinkLED();
                if (userCommand == "reset") currentState = IDLE;
                break;
        }

        // 3. Telemetry (Optional, run at lower frequency)
        sendDataToPC();
    }
*/ 

class Cubli {
    private:
        // Nested Classes/Structs
        struct Orientation {
            // Constructor
            Orientation(); 

            array<int, 2> state; // (face/edge/corner, which face/corner/edge)
            
            // Reference tabel for vector quantization to find cubli orienttation
            std::map< array<int, 2>, BLA::Matrix<1, 3> > crossReferenceTable; // key = (state, instance)

            // table of Transforms from natural body to the virtual body for each specific orientation
            std::map< array<int, 2>, BLA::Matrix<3, 3> > nbodyToVbodyTable; // key = (state, instance)

            // table of Pointers to appropriate gain forn orientation 
            std::map< array<int, 2>, BLA::Matrix<3, 12>* > gainTable; // key = (state, instance)
        };
            
        // --- Cubli System Data ---
        BLA::Matrix<12,1> stateVector = BLA::Zeros<12,1>();
        BLA::Matrix<3,1> torqueVector = BLA::Zeros<3,1>();
        BLA::Matrix<3,1> voltageVector = BLA::Zeros<3,1>();
        Orientation orientation = Orientation();

        // --- Constant ---
        const BLA::Matrix<3,3> sensorToNbody = {0, -1, 0,
                                                -1, 0, 0,
                                                0, 0, -1};   
        const BLA::Matrix<3,3> vbodyToEdge = {sqrtf(2)/2, 0, sqrtf(2)/2, 
                                              0, 1, 0,
                                              -sqrtf(2)/2, 0, sqrtf(2)/2};
        const BLA::Matrix<3,3> vbodyToCorner = {sqrtf(2)/2, 0, -sqrt(2)/2,
                                                -sqrtf(6)/6, 2*sqrtf(6)/6, -sqrtf(6)/6,
                                                sqrtf(3)/3, sqrtf(3)/3, sqrtf(3)/3};

        // --- Dynamic ---
        BLA::Matrix<3,12> gainActive = BLA::Zeros<3,12>();
        BLA::Matrix<3,3> nbodyToVbodyActive = {1, 0, 0,
                                               0, 1, 0,
                                               0, 0, 1}; 

        // --- Object Initalization ---
        MPU6050 mpu;
        BLDCDriver3PWM drivers[3];
        BLDCMotor motors[3];
        MagneticSensorI2C sensors[3];

        void updateStateVector() {// change to have state vector read into the ground frame
            mpu.update();
            
            BLA::Matrix<3,1> angle;
            BLA::Matrix<3,1> velocity;
            
            
            for (int axis = X; axis <= Z; axis++) {
                // --- Reading MPU ---
                switch(axis) {
                    case X:
                        angle(axis, 0) = mpu.getAngleX() * DEG_TO_RAD; // angle from gravity vector to x-axis in deg; filtered (we might want to filter differently)
                        velocity(axis, 0) = mpu.getGyroX() * DEG_TO_RAD; // angular velocity in deg/s
                        break;
                    case Y:
                        angle(axis, 0) = mpu.getAngleY() * DEG_TO_RAD;
                        velocity(axis, 0) = mpu.getGyroY() * DEG_TO_RAD;
                        break;  
                    case Z:
                        angle(axis, 0) = mpu.getAngleZ() * DEG_TO_RAD;
                        velocity(axis, 0) = mpu.getGyroZ() * DEG_TO_RAD;
                        break;
                }
            }
            
            // --- Transpose Angle and Velocity ---
            velocity = sensorToNbody * velocity; // natural body frame
            angle = angle + rotmToEuler(sensorToNbody);

            velocity = nbodyToVbodyActive * velocity; // virtual body frame
            angle = angle + rotmToEuler(nbodyToVbodyActive);

            switch (orientation.state[0]) { // ground frame
                case EDGE:
                    velocity = vbodyToEdge * velocity;
                    angle = angle + rotmToEuler(vbodyToEdge);
                    break;
                case CORNER:
                    velocity = vbodyToCorner * velocity;
                    angle = angle + rotmToEuler(vbodyToCorner);
                    break;
            }
            
            int i = 0;
            for (int axis = X; axis <= Z; axis++) {

                // Add angle and velcoity to state vector
                stateVector(i++, 0) = angle(axis, 0); // angle from gravity vector to x-axis in deg; filtered (we might want to filter differently)
                stateVector(i++, 0) = velocity(axis, 0); // angular velocity in deg/s
                        
                // --- Reading Encoder ---
                selectI2CChan(SENSOR_CHAN[axis]);
                stateVector(i++, 0) = sensors[axis].getAngle(); // cumulative angle (neccessary for LQR) in rad
                stateVector(i++, 0) = sensors[axis].getVelocity(); // gets velocity in rad/s
            }
        }

        void torqueToVoltage() {
            // ground frame -> virtual body frame
            switch (orientation.state[0]) {
                case EDGE:
                    torqueVector = ~vbodyToEdge * torqueVector;
                    break;
                case CORNER:
                    torqueVector = ~vbodyToCorner * torqueVector;
                    break;
            }

            // virtual body frame -> natural body frame
            torqueVector = ~nbodyToVbodyActive * torqueVector;

            // convert torque to voltage
            for (int axis = X; axis <= Z; axis++) {
                // Need variable to reference the correct angular velocity
                int wheelVelIndex = SINGLE_AXIS_STATE_VECTOR_SIZE * (axis + 1) - 1;

                // Converts torque to voltage accounting for back emf
                voltageVector(axis) = torqueVector(axis) * MOTOR_RESISTANCE / TORQUE_CONSTANT 
                                        + TORQUE_CONSTANT * stateVector(wheelVelIndex);
            }
        }

        void applyVoltage() {
            for (int axis = X; axis <= Z; axis++) {
                motors[axis].move(voltageVector(axis));
            }
        }

        void updateOrientation() {
            // Pull current acceleration values from mpu
            BLA::Matrix<3,1> accVector = { mpu.getAccX(), mpu.getAccY(), mpu.getAccZ() };
            // cast into body reference frame
            accVector = sensorToNbody * accVector;

            //normalize the vector
            float magAccVectorSquared = 0;
            for (int axis = X; axis <= Z; axis++) {
                magAccVectorSquared += powf(accVector(axis, 0),2) ;
            }
            BLA::Matrix<3,1> normalizedAccVector = accVector / sqrt(magAccVectorSquared);

            // iterate through reference table to find correct orientation using cross product
            array<int, 2> minDotProdKey;
            float minDotProdVal = 0;
            for (auto const& [key, referenceVector] : orientation.crossReferenceTable) {
                float dotProd = (referenceVector * normalizedAccVector)(0);
                if ((1 - dotProd) < (1 - minDotProdVal)) {
                    minDotProdVal = dotProd;
                    minDotProdKey = key;
                }            
            }
            orientation.state = minDotProdKey;

            // Set system data to active orientation
            gainActive = *orientation.gainTable[orientation.state];
            nbodyToVbodyActive = orientation.nbodyToVbodyTable[orientation.state];
            
            DEBUG_PRINT("State and Instance Found by getOrientation");
            DEBUG_PRINT( orientation.state[0]); DEBUG_PRINT(orientation.state[1]);
        }

        BLA::Matrix<3, 1> rotmToEuler(BLA::Matrix<3,3> R) {// (roll, pitch, yaw)
            return { atan2f( R(3, 2), R(3, 3) ),
                     atan2f( -R(3, 1), sqrtf( powf(R(1, 1), 2) + powf(R(2, 1), 2) )),
                     atan2f( R(2, 1), R(1, 1) ) };
        } 

        void selectI2CChan(const byte& channel) {
            Wire.beginTransmission(MUX_ADD);
            Wire.write(1 << channel);
            Wire.endTransmission();
        }

    public:
        // Constructor
        Cubli(); 

        void init() {// --- WAKE SYSTEM UP ---
            pinMode(PIN_EN, OUTPUT);
            digitalWrite(PIN_EN, LOW); // Force Drivers off immediately
            delay(100);
            
            DEBUG_BEGIN(SERIAL_COM); // Initiates computer to ESP32 communication
            Wire.begin(PIN_SDA, PIN_SCL); // Initiates I2C by "waking up" pins and readying them

            delay(5000); // Give time to place it on a level face

            // --- MPU6050 Initalization ---
            // initiates communication with MPU; return 0 if successful, 1 or 2 if wiring failure
            byte mpuInitationStatus = mpu.begin(); 
            if (mpuInitationStatus != 0) {
                DEBUG_PRINTLN("MPU wiring failure! Status: ");
                DEBUG_PRINTLN(mpuInitationStatus);
                while(1); // Stop Everything
            }
            DEBUG_PRINTLN("MPU is awake. Calibrating offset.");
            // Calibrates MPU
            mpu.calcOffsets();

            // --- Encoder Initalization ---
            for (int axis=X; axis<=Z; axis++) {
                selectI2CChan(SENSOR_CHAN[axis]);
                sensors[axis].init();
                motors[axis].linkSensor(&sensors[axis]);
            }

            DEBUG_PRINT("Calibrating Motors and Linking Sensors");
            for (int axis = X; axis <= Z; axis++) {
                // --- Driver Initialization ---
                drivers[axis].voltage_power_supply = BATTERY_VOLTAGE;
                drivers[axis].init();
                motors[axis].linkDriver(&drivers[axis]);
                
                // --- FOC Parameters ---
                motors[axis].controller = MotionControlType::torque;
                motors[axis].voltage_limit = VOLTAGE_LIMIT;
                motors[axis].voltage_sensor_align = ALIGNMENT_VOLTAGE;

                // --- Motor Initialization ---
                motors[axis].init();
                
                // Check to see if FOC initalizes
                selectI2CChan(SENSOR_CHAN[axis]); // need to change channels to control motor using FOC
                if (motors[axis].initFOC()) {
                    DEBUG_PRINT("Motor "); DEBUG_PRINT(axis); DEBUG_PRINT(" aligned.");
                } else {
                    DEBUG_PRINT("FOC Failed. Check Connections on "); DEBUG_PRINT(axis);
                    while(1);
                }
            }
        }
        
        // --- GEMINI WRITTEN FUNCTION FOR SYSTEM VALIDATION ---
        void systemsTest() {
            DEBUG_PRINTLN("--- STARTING SYSTEM TEST ---");
            bool all_passed = true;

            // 1. Check MPU (Keep this part, it works)
            DEBUG_PRINTLN("1. Checking MPU6050...");
            Wire.beginTransmission(0x68);
            if (Wire.endTransmission() == 0) {
                DEBUG_PRINTLN("   [PASS] MPU6050 found.");
                mpu.update();
                DEBUG_PRINT("   Current Z Angle: "); DEBUG_PRINTLN(mpu.getAngleZ());
            } else {
                DEBUG_PRINTLN("   [FAIL] MPU6050 missing!");
                all_passed = false;
            }

            // 2. Check Encoders (Keep this part, it works)
            DEBUG_PRINTLN("2. Checking Encoders...");
            for (int i = 0; i < 3; i++) {
                selectI2CChan(SENSOR_CHAN[i]);
                Wire.beginTransmission(0x36);
                if (Wire.endTransmission() == 0) {
                    DEBUG_PRINT("   [PASS] Encoder "); DEBUG_PRINT(i); 
                    DEBUG_PRINT(" Angle: "); DEBUG_PRINTLN(sensors[i].getAngle());
                } else {
                    DEBUG_PRINT("   [FAIL] Encoder "); DEBUG_PRINT(i); DEBUG_PRINTLN(" missing!");
                    all_passed = false;
                }
            }

            // 3. Check Motors (THE FIXED PART)
            DEBUG_PRINTLN("3. Checking Motors (Active Spin)...");
            
            for (int i = 0; i < 3; i++) {
                DEBUG_PRINT("   Testing Motor "); DEBUG_PRINT(i);
                
                // A. Switch Mux to this motor so loopFOC can read the sensor
                selectI2CChan(SENSOR_CHAN[i]);
                
                // B. Active Spin Loop (Replace delay with this)
                long start_time = millis();
                float max_velocity = 0;
                
                // Run for 500ms
                while (millis() - start_time < 500) {
                    // CRITICAL: Must call loopFOC constantly to spin!
                    motors[i].loopFOC(); 
                    
                    // Command 3.0 Volts (Enough to overcome friction)
                    motors[i].move(3.0); 
                    
                    // Capture peak velocity
                    float v = abs(sensors[i].getVelocity());
                    if (v > max_velocity) max_velocity = v;
                }
                
                // C. Stop
                motors[i].move(0);
                
                // D. Verify
                if (max_velocity > 0.5) { // Threshold: 0.5 rad/s
                    DEBUG_PRINTLN(" [PASS] Moved.");
                } else {
                    DEBUG_PRINTLN(" [FAIL] No movement detected!");
                    all_passed = false;
                }
                delay(100); // Short pause before next motor
            }

            DEBUG_PRINTLN("----------------------------");
            if (all_passed) {
                DEBUG_PRINTLN("SYSTEM TEST: PASSED. Starting Balance Loop...");
                delay(2000);
            } else {
                DEBUG_PRINTLN("SYSTEM TEST: FAILED. Halting.");
                while(1);
            }
        }

        void edgeJump() {// --- NOT A FEATURE OF V1 ---
        }

        void edgeBalance() {// --- EDGE BALANCE 1D LQR CONTROL LOOP ---
            // applies correct "active" gain and transpose matrix based on spacial orientation
            updateOrientation();
            
            while (1) {
                // Update state vector and check to make sure the angle isn't too great to recover from
                updateStateVector(); // in ground frame

                int yAngleIndex = Y * SINGLE_AXIS_STATE_VECTOR_SIZE;
                if (stateVector(yAngleIndex) > ANGLE_LIMIT) {
                    voltageVector = {0, 0, 0};
                    applyVoltage();
                    DEBUG_PRINT("Angle Exceeds Safety Limit.");
                    while (1); 
                }

                // Get input voltage
                torqueVector = - gainActive * stateVector; // ground frame torque 
                torqueToVoltage();        
                
                // Apply corrective voltage
                applyVoltage();
            }
        }

        void cornerJump() {// --- NOT A FEATURE OF V1 ---
            }

        void cornerBalance() {// --- CORNER BALANCE 3D LQR CONTROL LOOP ---   
            
    };
};

#endif