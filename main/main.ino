#include "imu_control.h"
#include "simulate_test.h"

void setup() {
  setupIMU();

  // setupSimulation();
}

void loop() {
  updateIMU();

  // simulatePitchInput();
}
