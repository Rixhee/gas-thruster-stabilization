#ifndef THRUST_CONTROL_H
#define THRUST_CONTROL_H

#include <Arduino.h>
#include "imu_control.h"

void thrustControl(float correction) {
  if (correction == 0) {
    digitalWrite(thruster1, LOW);
    digitalWrite(thruster2, LOW);
  } else {
    unsigned long current_time = millis();
    if (digitalRead(thruster1) == LOW && digitalRead(thruster2) == LOW) {
      previous_time = current_time;
    }
    if (current_time - previous_time < abs(correction)) {
      if (error < 0) {
        digitalWrite(thruster1, HIGH);
        digitalWrite(thruster2, LOW);
      } else {
        digitalWrite(thruster2, HIGH);
        digitalWrite(thruster1, LOW);
      }
    } else {
      digitalWrite(thruster1, LOW);
      digitalWrite(thruster2, LOW);
    }
  }
}

#endif