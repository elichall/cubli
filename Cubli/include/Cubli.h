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
            array<int, 2> stateOld;
            
            // Reference tabel for vector quantization to find cubli orienttation
            std::map< array<int, 2>, BLA::Matrix<1, 3> > crossReferenceTable; // key = (state, instance)

            // table of Transforms from natural body to the virtual body for each specific orientation
            std::map< array<int, 2>, BLA::Matrix<3, 3> > nbodyToVbodyTable; // key = (state, instance)

            // table of Pointers to appropriate gain forn orientation 
            std::map< array<int, 2>, BLA::Matrix<3, 12>* > gainTable; // key = (state, instance)
        };
            
        // --- Cubli System Data ---
        BLA::Matrix<12,1> stateVector = BLA::Zeros<12,1>(); // angle, ang. vel, mot angle, mot vel 
        BLA::Matrix<3,1> torqueVector = BLA::Zeros<3,1>();
        BLA::Matrix<3,1> voltageVector = BLA::Zeros<3,1>();
        BLA::Matrix<3,1> rawAcc = BLA::Zeros<3,1>();
        Orientation orientation = Orientation();

        // --- Constant ---
        const BLA::Matrix<3,3> sensorToNbody = { 0, -1,  0,
                                                -1,  0,  0,
                                                 0,  0, -1 };   
        const BLA::Matrix<3,3> vbodyToEdge = { sqrtf(2)/2,  0, -sqrtf(2)/2, 
                                               0,           1,  0,
                                               sqrtf(2)/2,  0,  sqrtf(2)/2 };
        const BLA::Matrix<3,3> vbodyToCorner = { sqrtf(2)/2,  0,            -sqrt(2)/2,
                                                -sqrtf(6)/6,  2*sqrtf(6)/6, -sqrtf(6)/6,
                                                 sqrtf(3)/3,  sqrtf(3)/3,    sqrtf(3)/3 };

        // --- Dynamic ---
        float dt = 0;
        unsigned long lastMPUUpdateTime = 0;
        BLA::Matrix<3,12> gainActive = BLA::Zeros<3,12>();
        BLA::Matrix<3,3> nbodyToVbodyActive = {1, 0, 0,
                                               0, 1, 0,
                                               0, 0, 1};

        // --- Object Initalization ---
        MPU6050 mpu;
        BLDCDriver3PWM drivers[3];
        BLDCMotor motors[3];
        MagneticSensorI2C sensors[3];

        void updateOrientation() {
            // Single instance of updating sensors in cycle, only call once per cycle
            mpu.update();

            // --- Update System Sample Clock ---
            // for numerical integration with complementary filter
            unsigned long currentTime = micros();
            dt = (currentTime - lastMPUUpdateTime) / 1000000.0f; // convert time to seconds
            lastMPUUpdateTime = currentTime;
            if (dt > 0.1f) dt = 0.005f; // catch for the first loop

            // Pull current acceleration values from mpu
            rawAcc = { mpu.getAccX(), mpu.getAccY(), mpu.getAccZ() };

            // cast into body reference frame
            BLA::Matrix<3,1> gravityVector = sensorToNbody * - rawAcc; // mpu reads normal force so negate accVect

            //normalize the vector
            float magGravityVectorSquared = 0;
            for (int axis = X; axis <= Z; axis++) {
                magGravityVectorSquared += powf(gravityVector(axis, 0), 2) ;
            }
            gravityVector = gravityVector / sqrtf(magGravityVectorSquared); // normalized

            // iterate through reference table to find correct orientation using dot product
            array<int, 2> minDotProdKey = {0, 0}; // intialize at a safe state
            float minDotProdVal = -2.0f; // lowest possible dot product value
            for (auto const& [key, referenceVector] : orientation.crossReferenceTable) {
                float dotProd = (referenceVector * gravityVector)(0);
                if ((1 - dotProd) < (1 - minDotProdVal)) {
                    minDotProdVal = dotProd;
                    minDotProdKey = key;
                }            
            }
            orientation.state = minDotProdKey;

            // --- Catch Null Pointer Crash ---
            // Set system data to active orientation
            if (orientation.gainTable.count(orientation.state) > 0) {
                gainActive = *orientation.gainTable[orientation.state];
                nbodyToVbodyActive = orientation.nbodyToVbodyTable[orientation.state];
            } else {
                gainActive = BLA::Zeros<3,12>();
                DEBUG_PRINTLN("[ERROR] Invalid state mapped. Applying zero gain.");
            }
        }

        void updateStateVector() {
            // Can't transform angle vector cleanly so need to transform raw true vectors 
            // then use complementary filter to get angles in correct frame
            BLA::Matrix<3,1> rawGyro = { mpu.getGyroX() * DEG_TO_RAD_F, 
                                         mpu.getGyroY() * DEG_TO_RAD_F, 
                                         mpu.getGyroZ() * DEG_TO_RAD_F };
            
            // --- Transpose Acc and Velocity ---
            // make a total active transform matrix after updateOrientation has set all the correct transforms
            BLA::Matrix<3,3> totalTransformActive;
            switch (orientation.state[0]) {
                case EDGE:
                    totalTransformActive = vbodyToEdge * nbodyToVbodyActive * sensorToNbody;
                    break;
                case CORNER:
                    totalTransformActive = vbodyToCorner * nbodyToVbodyActive * sensorToNbody;
                    break;
                case FACE:
                    totalTransformActive = nbodyToVbodyActive * sensorToNbody;
                    break;
            }
            // easy mat multiplication for vel and acc because it is a "true" vector
            BLA::Matrix<3,1> groundGyro = totalTransformActive * rawGyro;
            BLA::Matrix<3,1> groundAcc = totalTransformActive * rawAcc;

            // --- Complementary Filter ---
            BLA::Matrix<3,1> groundAngles = complementaryFilter(groundGyro, groundAcc);
            
            int i = 0;
            for (int axis = X; axis <= Z; axis++) {

                // Add angle and velcoity to state vector
                stateVector(i++, 0) = groundAngles(axis, 0); // angle from gravity vector to x-axis in deg; filtered (we might want to filter differently)
                stateVector(i++, 0) = groundGyro(axis, 0); // angular velocity in deg/s
                        
                // --- Reading Encoder and FOC Communication ---
                selectI2CChan(SENSOR_CHAN[axis]);
                motors[axis].loopFOC();
                stateVector(i++, 0) = sensors[axis].getAngle(); // cumulative angle (neccessary for LQR) in rad
                stateVector(i++, 0) = sensors[axis].getVelocity(); // gets velocity in rad/s
            }

            // Update orientation and store old state for detecting discontinuity
            orientation.stateOld = orientation.state;
        }

        BLA::Matrix<3,1> complementaryFilter(BLA::Matrix<3,1> gyro, BLA::Matrix<3,1> acc) {
            float filterWeight[3] = {0.98f,0.98f,1.0f}; 
            // if state has changed this loop then need to use a 100 percent acc based intial angle estimate
            if (orientation.state != orientation.stateOld) {filterWeight[X] = 0.0f; filterWeight[Y]=0.0f;}
            // for now perserve Z but we may need to change this later depending on how we handle the corner balance

            // Angles from accelerometer (Tait-Bryan angles)
            BLA::Matrix<3,1> accAngles = { atan2( acc(Y,0) , acc(Z,0) ),
                                           atan2( -acc(X,0) , sqrtf( powf(acc(Y,0),2) + powf(acc(Z,0),2) ) ),
                                           0.0f };

            // Angles from gyroscope (num intg)
            BLA::Matrix<3,1> gyroAngles;
            BLA::Matrix<3,1> filteredAngles;
            for (int axis = X; axis <= Z; axis++) { 
                int angleIdx = axis * SINGLE_AXIS_STATE_VECTOR_SIZE; 

                gyroAngles(axis, 0) = stateVector(angleIdx,0) + gyro(axis,0) * dt;

                filteredAngles(axis, 0) = filterWeight[axis] * gyroAngles(axis,0) + 
                                          (1 - filterWeight[axis]) * accAngles(axis,0); 
            }
    
            return filteredAngles;
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

        void selectI2CChan(const byte& channel) {
            Wire.beginTransmission(MUX_ADD);
            Wire.write(1 << channel);
            Wire.endTransmission();
        }

        /*
        // BLA::Matrix<3, 1> rotmToEuler(BLA::Matrix<3,3> R) {
        //     // returns (Roll, Pitch, Yaw)
        //     return { atan2f( R(2, 1), R(2, 2) ),
        //              atan2f( -R(2, 0), sqrtf( powf(R(0, 0), 2) + powf(R(1, 0), 2) )),
        //              atan2f( R(1, 0), R(0, 0) ) };
        // }

        // BLA::Matrix<3,3> eulerToRotm(BLA::Matrix<3,1> angles) {
        //     // input order roll pitch yaw
        //     float r = angles(X,0); float p = angles(Y,0); float y = angles(Z,0);
        //     float cr = cosf(r);  float sr = sinf(r);
        //     float cp = cosf(p); float sp = sinf(p);
        //     float cy = cosf(y);   float sy = sinf(y);

        //     return {
        //         cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr,
        //         sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr,
        //         -sp,    cp*sr,             cp*cr
        //     };
        // }
        */
        
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
            mpu.calcOffsets(true, false); // onboard accelerometer calibration breaks code (vel cali, acc cali)

            // --- Encoder Initalization ---
            for (int axis=X; axis<=Z; axis++) {
                selectI2CChan(SENSOR_CHAN[axis]);
                sensors[axis].init();
                motors[axis].linkSensor(&sensors[axis]);
            }

            DEBUG_PRINTLN("Calibrating Motors and Linking Sensors");
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
                    DEBUG_PRINT("Motor "); DEBUG_PRINT(axis); DEBUG_PRINTLN(" aligned.");
                } else {
                    DEBUG_PRINT("FOC Failed. Check Connections on "); DEBUG_PRINTLN(axis);
                    // while(1);
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

        void orientationTest() {

            updateOrientation();
            updateStateVector();

            static unsigned long lastPrintTime = 0;
            if (millis() - lastPrintTime >= 3000) {
                lastPrintTime = millis();

                DEBUG_PRINTLN("--- State and Instance Found by getOrientation ---");
                DEBUG_PRINT( orientation.state[0]);DEBUG_PRINT(" | "); 
                DEBUG_PRINTLN(orientation.state[1]);

                DEBUG_PRINTLN("--- RAW Angles ---"); 
                DEBUG_PRINT(mpu.getAngleX());DEBUG_PRINT(" | "); 
                DEBUG_PRINT(mpu.getAngleY());DEBUG_PRINT(" | "); 
                DEBUG_PRINTLN(mpu.getAngleZ());

                DEBUG_PRINTLN("--- State Vector Angles ---");
                DEBUG_PRINT(stateVector(0, 0) * RAD_TO_DEG); DEBUG_PRINT(" | ");
                DEBUG_PRINT(stateVector(4, 0) * RAD_TO_DEG); DEBUG_PRINT(" | ");
                DEBUG_PRINTLN(stateVector(8, 0) * RAD_TO_DEG);

                DEBUG_PRINTLN("--- RAW Velocities ---"); 
                DEBUG_PRINT(mpu.getGyroX());DEBUG_PRINT(" | "); 
                DEBUG_PRINT(mpu.getGyroY());DEBUG_PRINT(" | "); 
                DEBUG_PRINTLN(mpu.getGyroZ());

                DEBUG_PRINTLN("--- State Vector Velocities ---");
                DEBUG_PRINT(stateVector(1, 0) * RAD_TO_DEG); DEBUG_PRINT(" | ");
                DEBUG_PRINT(stateVector(5, 0) * RAD_TO_DEG); DEBUG_PRINT(" | ");
                DEBUG_PRINTLN(stateVector(9, 0) * RAD_TO_DEG);
                DEBUG_PRINTLN("");
            }
        }

        void edgeJump() {// --- NOT A FEATURE OF V1 ---
        }

        void edgeBalance() {// --- EDGE BALANCE 1D LQR CONTROL LOOP ---
            while (1) {
                // applies correct "active" gain and transpose matrix based on spacial orientation
                updateOrientation();
                // Update state vector and check to make sure the angle isn't too great to recover from
                updateStateVector(); // in ground frame

                int yAngleIndex = Y * SINGLE_AXIS_STATE_VECTOR_SIZE;
                if (abs(stateVector(yAngleIndex)) > ANGLE_LIMIT) {
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