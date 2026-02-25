#include <Arduino.h>
#include <SimpleFOC.h>
#include <MPU6050_light.h>
#include <Wire.h>
#include "Cubli.h"

// // --- Object Initalization ---
// Cubli cubli;

// // --- Setup Phase ---
// void setup() {
//   delay(5000);
//   // Initalize System
//   cubli.init();

//   // Run systems test to check if all components are operational, freezes loop if not
//   cubli.systemsTest();

// }

// // --- Main Loop ---
// void loop() {
  
//   delay(5000); // Wait 10s for user to place cube in the right position

//   cubli.edgeBalance();

// }
/**
 * SANITY CHECK SCRIPT
 * 1. Prints MPU6050 Data (Pitch/Roll)
 * 2. Spins all 3 motors at 5 rad/s (Open Loop)
 * * Hardware Required: ESP32, 3x Drivers, MPU6050
 * NOT Required: Encoders, Multiplexer (We skip them)
 */

// --- PIN DEFINITIONS (MATCHING YOUR CONFIG) ---
#define PIN_SDA 23
#define PIN_SCL 22
#define PIN_EN  33

// Motor X
#define PIN_X_IN1 25
#define PIN_X_IN2 26
#define PIN_X_IN3 27

// Motor Y
#define PIN_Y_IN1 18
#define PIN_Y_IN2 5
#define PIN_Y_IN3 17

// Motor Z
#define PIN_Z_IN1 16
#define PIN_Z_IN2 15
#define PIN_Z_IN3 2

// --- OBJECTS ---
MPU6050 mpu(Wire);

// Driver & Motor Instances
BLDCDriver3PWM driverX = BLDCDriver3PWM(PIN_X_IN1, PIN_X_IN2, PIN_X_IN3, PIN_EN);
BLDCDriver3PWM driverY = BLDCDriver3PWM(PIN_Y_IN1, PIN_Y_IN2, PIN_Y_IN3, PIN_EN);
BLDCDriver3PWM driverZ = BLDCDriver3PWM(PIN_Z_IN1, PIN_Z_IN2, PIN_Z_IN3, PIN_EN);

// OPEN LOOP MOTORS (No Sensors needed!)
BLDCMotor motorX = BLDCMotor(7);
BLDCMotor motorY = BLDCMotor(7);
BLDCMotor motorZ = BLDCMotor(7);

// Target Velocity
float target_velocity = 2.0; // rad/s (approx 20 RPM - visible but slow)

void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_SDA, PIN_SCL);
  delay(1000);

  Serial.println("\n--- STARTING SANITY CHECK ---");

  // 1. Initialize MPU6050
  Serial.print("Initializing MPU6050... ");
  byte status = mpu.begin();
  if (status == 0) {
    Serial.println("SUCCESS.");
    Serial.println("Calibrating (Don't move system)...");
    mpu.calcOffsets(); // Auto-calibrate
    Serial.println("Done.");
  } else {
    Serial.print("FAILED! Status: "); Serial.println(status);
    while(1); // Halt
  }

  // 2. Initialize Drivers
  Serial.println("Initializing Drivers...");
  driverX.voltage_power_supply = 11.1;
  driverY.voltage_power_supply = 11.1;
  driverZ.voltage_power_supply = 11.1;
  
  driverX.init();
  driverY.init();
  driverZ.init();

  // 3. Initialize Motors (OPEN LOOP)
  // We limit voltage to 3V to keep it safe and cool
  motorX.voltage_limit = 3.0; 
  motorY.voltage_limit = 3.0;
  motorZ.voltage_limit = 3.0;

  // Link drivers
  motorX.linkDriver(&driverX);
  motorY.linkDriver(&driverY);
  motorZ.linkDriver(&driverZ);

  // Set controller to VelocityOpenLoop
  motorX.controller = MotionControlType::velocity_openloop;
  motorY.controller = MotionControlType::velocity_openloop;
  motorZ.controller = MotionControlType::velocity_openloop;

  motorX.init();
  motorY.init();
  motorZ.init();

  Serial.println("Motors Ready. Spinning at 2 rad/s...");
}

void loop() {
  // 1. Update MPU
  mpu.update();

  // 2. Spin Motors (Open Loop calculation)
  motorX.move(target_velocity);
  motorY.move(target_velocity);
  motorZ.move(target_velocity);

  // 3. Print Status every 500ms
  static long last_print = 0;
  if (millis() - last_print > 500) {
    last_print = millis();
    
    Serial.print("PITCH: "); Serial.print(mpu.getAngleY());
    Serial.print(" | ROLL: "); Serial.print(mpu.getAngleX());
    Serial.print(" | YAW: "); Serial.println(mpu.getAngleZ());
    
    // Toggle direction every 5 seconds to prove control
    if (millis() % 10000 > 5000) target_velocity = -2.0;
    else target_velocity = 2.0;
  }
}