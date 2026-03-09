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

  delay(5000); // so I can place it on its side
}

// --- Main Loop ---
void loop() {
  cubli.edgeBalance();
  cubli.orientationTest();
}