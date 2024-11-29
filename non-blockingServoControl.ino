#include <Servo.h>

Servo servoR, servoM, servoL;

const int openPos = 150;
const int closePos = 180;

unsigned long lastActionTime = 0;
int servoState = 0; // 0 = idle, 1 = opening, 2 = closing
char activeCommand = '\0';

void setup() {
  servoR.attach(9);
  servoM.attach(10);
  servoL.attach(11);
  servoR.write(closePos);
  servoM.write(closePos);
  servoL.write(closePos);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    activeCommand = Serial.read();
    servoState = 1;
    lastActionTime = millis();
    activateServo(activeCommand);
  }

  if (servoState == 1 && millis() - lastActionTime >= 3000) {
    deactivateServo(activeCommand);
    servoState = 0;
    activeCommand = '\0';
  }
}

void activateServo(char command) {
  if (command == 'R') servoR.write(openPos);
  else if (command == 'M') servoM.write(openPos);
  else if (command == 'L') servoL.write(openPos);
}

void deactivateServo(char command) {
  if (command == 'R') servoR.write(closePos);
  else if (command == 'M') servoM.write(closePos);
  else if (command == 'L') servoL.write(closePos);
}
