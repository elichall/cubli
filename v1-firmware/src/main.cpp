#include <Arduino.h>
#include <SimpleFOC.h>
#include <MPU6050_light.h>
#include <Wire.h>
#include "Cubli.h"

// --- Object Initalization ---
Cubli cubli;

// --- Setup Phase ---
void setup() {
  // Initalize System
  cubli.init();

  delay(5000); // so I can place it on its corner
}

// --- Main Loop ---
void loop() {
  cubli.update(); // updates relevant state vars
  cubli.edgeBalance(); // lqr gain and applying voltage
  cubli.stateTest(); // print state vars every three seconds
}