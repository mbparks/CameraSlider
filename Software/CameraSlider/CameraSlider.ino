#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMStop(0x60); // Default address, no jumpers

#define stepsPerRev 200

//For Linear Motion Stepper
#define farLeftLinearPosition 1000000
#define farRightLinearPosition -1000000

//For Rotate Motion Stepper
#define farLeftRotatePosition 10000000
#define farRightRotatePosition -10000000

#define delayVal 1000

//Button and Switch Pin Assignments
#define leftLimitSwitch 10
#define rightLimitSwitch 13
#define redButton 9    // move and rotate
#define blackButton 11   // move and do NOT rotate

enum linearStates {
  unknown, movingLeft, movingRight, farLeft, farRight
};

// LinearMotor
// BLACK   + M1 OUTSIDE   D2
// GREEN   - M1 INSIDE    D3
// RED     + M2 INSIDE    D1
// BLUE    - M2 OUTSIDE   D4
Adafruit_StepperMotor *linearMotor = AFMStop.getStepper(stepsPerRev, 1);

// RotateMotor
// BLACK   + M3 OUTSIDE   D5
// GREEN   - M3 INSIDE    D7
// RED     + M4 INSIDE    D0
// BLUE    - M4 OUTSIDE   D6
Adafruit_StepperMotor *rotateMotor = AFMStop.getStepper(stepsPerRev, 2);



// you can change these to DOUBLE or INTERLEAVE or MICROSTEP!
// wrappers for the first motor!
void forwardstep1() {
  linearMotor->onestep(FORWARD, DOUBLE);
}
void backwardstep1() {
  linearMotor->onestep(BACKWARD, DOUBLE);
}
// wrappers for the second motor!
void forwardstep2() {
  rotateMotor->onestep(FORWARD, MICROSTEP);
}
void backwardstep2() {
  rotateMotor->onestep(BACKWARD, MICROSTEP);
}

AccelStepper linearStepper(forwardstep1, backwardstep1);
AccelStepper rotateStepper(forwardstep2, backwardstep2);

linearStates currentLinearState = unknown;

bool leftSwitchNotPressed = true;
bool rightSwitchNotPressed = true;
bool redButtonPressed = false;
bool blackButtonPressed = false;

static bool doRotate = true;
static bool doNotRotate = false;


void setup() {
  Serial.begin(9600);
  Serial.println("Camera Motion Project");

  AFMStop.begin(); // Start the top shield

  linearStepper.setMaxSpeed(500.0);
  linearStepper.setAcceleration(800.0);
  linearStepper.moveTo(farLeftLinearPosition);

  rotateStepper.setMaxSpeed(10.75);
  rotateStepper.setAcceleration(0.75);
  rotateStepper.moveTo(farRightRotatePosition);

  pinMode(leftLimitSwitch, INPUT_PULLUP);
  pinMode(rightLimitSwitch, INPUT_PULLUP);
  pinMode(blackButton, INPUT_PULLUP);
  pinMode(redButton, INPUT_PULLUP);


  Serial.println("Homing the linear motion platform...");
  moveToFarLeft(doNotRotate);
  Serial.println("Platform home. Waiting for button press...");
  linearStepper.setCurrentPosition(0);
}















void loop() {
  blackButtonPressed = digitalRead(blackButton);
  redButtonPressed = digitalRead(redButton);
  
  while (blackButtonPressed == false && redButtonPressed == false) {
    blackButtonPressed = digitalRead(blackButton);
    redButtonPressed = digitalRead(redButton);
  }

  if (blackButtonPressed == true) {
    switch (currentLinearState) {
      case farLeft:
        moveToFarRight(doNotRotate);
        delay(delayVal);
        break;

      case farRight:
        moveToFarLeft(doNotRotate);
        delay(delayVal);
        break;

      default:
        break;
    }
  }


  if (redButtonPressed == true) {
    switch (currentLinearState) {
      case farLeft:
        moveToFarRight(doRotate);
        delay(delayVal);
        break;

      case farRight:
        moveToFarLeft(doRotate);
        delay(delayVal);
        break;

      default:
        break;
    }
  }
} 





void displayLinearMotorStatus() {
  Serial.print("Current Linear Status: ");
  switch (currentLinearState) {
    case unknown:
      Serial.println("UNKNOWN");
      break;
    case movingLeft:
      Serial.println("MOVING TO LEFT");
      break;
    case movingRight:
      Serial.println("MOVING TO RIGHT");
      break;
    case farLeft:
      Serial.println("STOPPED FAR LEFT");
      break;
    case farRight:
      Serial.println("STOPPED FAR RIGHT");
      break;
    default:
      Serial.println("UNKNOWN");
      break;
  }
}



void moveToFarLeft(bool rotateCamera) {
  linearStepper.moveTo(farLeftLinearPosition);
  currentLinearState = movingLeft;
  displayLinearMotorStatus();
  leftSwitchNotPressed = digitalRead(leftLimitSwitch);

  while (leftSwitchNotPressed == true) {
    linearStepper.run();
    if (rotateCamera) {
      rotatePlatformRight();
    }
    leftSwitchNotPressed = digitalRead(leftLimitSwitch);
  }

  linearMotor->release();
  rotateMotor->release();
  currentLinearState = farLeft;
  linearStepper.moveTo(linearStepper.currentPosition());
  displayLinearMotorStatus();
}



void moveToFarRight(bool rotateCamera) {
  linearStepper.moveTo(farRightLinearPosition);
  currentLinearState = movingRight;
  displayLinearMotorStatus();
  rightSwitchNotPressed = digitalRead(rightLimitSwitch);

  while (rightSwitchNotPressed == true) {
    linearStepper.run();
    if (rotateCamera) {
      rotatePlatformLeft();
    }
    rightSwitchNotPressed = digitalRead(rightLimitSwitch);
  }

  linearMotor->release();
  rotateMotor->release();
  currentLinearState = farRight;
  linearStepper.moveTo(linearStepper.currentPosition());
  displayLinearMotorStatus();
}



void rotatePlatformLeft() {
  rotateStepper.move(farLeftRotatePosition);
  rotateStepper.run();
}


void rotatePlatformRight() {
  rotateStepper.move(farRightRotatePosition);
  rotateStepper.run();
}
