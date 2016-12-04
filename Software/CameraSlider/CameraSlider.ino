#include <AccelStepper.h>
#include <AFMotor.h>

//For Linear Motion Stepper
#define stepsPerRev 200
#define stepsPerLoop 1


//Button and Switch Pin Assignments
#define leftLimitSwitch 11
#define rightLimitSwitch 13
#define startButton 9
#define rotateButton 10

enum linearStates {
  unknown, movingLeft, movingRight, farLeft, farRight
};

// LinearMotor
// BLACK   + M1 OUTSIDE   D2
// GREEN   - M1 INSIDE    D3
// RED     + M2 INSIDE    D1
// BLUE    - M2 OUTSIDE   D4
AF_Stepper linearMotor(stepsPerRev, 1);

// RotateMotor
// BLACK   + M3 OUTSIDE   D5
// GREEN   - M3 INSIDE    D7
// RED     + M4 INSIDE    D0
// BLUE    - M4 OUTSIDE   D6
AF_Stepper rotateMotor(stepsPerRev, 2);



// you can change these to DOUBLE or INTERLEAVE or MICROSTEP!
// wrappers for the first motor!
void forwardstep1() {  
  linearMotor.onestep(FORWARD, DOUBLE);
}
void backwardstep1() {  
  linearMotor.onestep(BACKWARD, DOUBLE);
}
// wrappers for the second motor!
void forwardstep2() {  
  rotateMotor.onestep(FORWARD, DOUBLE);
}
void backwardstep2() {  
  rotateMotor.onestep(BACKWARD, DOUBLE);
}

AccelStepper linearStepper(forwardstep1, backwardstep1);
AccelStepper rotateStepper(forwardstep2, backwardstep2);

linearStates currentLinearState = unknown;

bool leftSwitchNotPressed = true;
bool rightSwitchNotPressed = true;
bool startButtonNotPressed = true;
bool rotateButtonNotPressed = true;

static bool doRotate = true;
static bool doNotRotate = false;


void setup() {
  Serial.begin(9600);
  Serial.println("Camera Motion Project");

  linearStepper.setMaxSpeed(100.0);
  linearStepper.setAcceleration(50.0);
  linearStepper.moveTo(1000000);
    
  rotateStepper.setMaxSpeed(100.0);
  rotateStepper.setAcceleration(50.0);
  rotateStepper.moveTo(1000000);

  pinMode(leftLimitSwitch, INPUT_PULLUP);
  pinMode(rightLimitSwitch, INPUT_PULLUP);
  pinMode(startButton, INPUT_PULLUP);
  pinMode(rotateButton, INPUT_PULLUP);

  Serial.println("Homing the linear motion platform...");
  moveToFarLeft(doNotRotate);
  Serial.println("Platform home. Waiting for button press...");
  linearStepper.setCurrentPosition(0); 
}



void loop() {

  startButtonNotPressed = digitalRead(startButton);
  rotateButtonNotPressed = digitalRead(rotateButton);
  while (startButtonNotPressed == true && rotateButtonNotPressed == true) {
    //displayLinearMotorStatus();
    startButtonNotPressed = digitalRead(startButton);
    rotateButtonNotPressed = digitalRead(rotateButton);
  }

  if (startButtonNotPressed == false) {
    switch (currentLinearState) {
      case farLeft:
        moveToFarRight(doRotate);
        delay(1000);
        break;

      case farRight:
        moveToFarLeft(doRotate);
        delay(1000);
        break;

      default:
        break;
    }
  }

  if (rotateButtonNotPressed == false) {
    Serial.println("Rotating the camera platform");
    rotatePlatformLeft();
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
  linearStepper.moveTo(100000);
  currentLinearState = movingLeft;
  displayLinearMotorStatus();
  //linearStepper.enableOutputs();
  leftSwitchNotPressed = digitalRead(leftLimitSwitch);
  
  while (leftSwitchNotPressed == true) {
    linearStepper.run();
    if (rotateCamera) {
      rotatePlatformRight();
    }
    leftSwitchNotPressed = digitalRead(leftLimitSwitch);
  }

  currentLinearState = farLeft;
  linearStepper.moveTo(linearStepper.currentPosition());
  //linearStepper.disableOutputs();
  displayLinearMotorStatus();
}



void moveToFarRight(bool rotateCamera) {
  linearStepper.moveTo(-1000000);
  currentLinearState = movingRight;
  displayLinearMotorStatus();
  //linearStepper.enableOutputs();
  rightSwitchNotPressed = digitalRead(rightLimitSwitch);
  
  while (rightSwitchNotPressed == true) {
    
    linearStepper.run();
    if (rotateCamera) {
      rotatePlatformLeft();
    }
    rightSwitchNotPressed = digitalRead(rightLimitSwitch);
  }

  currentLinearState = farRight;
  linearStepper.moveTo(linearStepper.currentPosition());
  //linearStepper.disableOutputs();
  displayLinearMotorStatus();
}



void rotatePlatformLeft() {
  //rotateStepper.enableOutputs();
  rotateStepper.move(-10000000);
  rotateStepper.run();
  //rotateStepper.disableOutputs();
}


void rotatePlatformRight() {
  //rotateStepper.enableOutputs();
  rotateStepper.move(1000000);
  rotateStepper.run();
  //rotateStepper.disableOutputs();
}

