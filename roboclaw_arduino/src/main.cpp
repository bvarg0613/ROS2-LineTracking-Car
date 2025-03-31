#include <Arduino.h>
#include <RoboClaw.h>
#include <SoftwareSerial.h>

#define address 0x80

// RX, TX (connect to RoboClaw S1 and S2)
// Make sure these are free digital pins
SoftwareSerial roboclawSerial(10, 11);  // RX=10, TX=11

RoboClaw roboclaw(&roboclawSerial, 38400);

void setup() {
  Serial.begin(9600);             // Serial from ROS (via USB)
  roboclawSerial.begin(38400);    // Serial to RoboClaw
  roboclaw.begin(38400);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    if (c == 'F') {
      roboclaw.ForwardM1(address, 64);
      roboclaw.BackwardM2(address, 64);
    } else if (c == 'R') {
      roboclaw.ForwardM1(address, 100);
      roboclaw.BackwardM2(address, 30);
    } else if (c == 'L') {
      roboclaw.ForwardM1(address, 30);
      roboclaw.BackwardM2(address, 100);
    } else if (c == 'S') {
      roboclaw.ForwardM1(address, 0);
      roboclaw.ForwardM2(address, 0);
    }
  }
}