#include <Arduino.h>
#include "A4988.h"
#include <MultiDriver.h>
#include "LiquidCrystal.h"

// using a 200-step motor (most common)
#define MOTOR_STEPS 200
// configure the pins connected
#define DIR_Y 8
#define STEP_Y 9
#define DIR_X 12
#define STEP_X 10
#define DIR_Z 13 
#define STEP_Z 11

A4988 StepperY(MOTOR_STEPS, DIR_Y, STEP_Y);
A4988 StepperX(MOTOR_STEPS, DIR_X, STEP_X);
A4988 StepperZ(MOTOR_STEPS, DIR_Z, STEP_Z);

MultiDriver controller(StepperY, StepperX, StepperZ);

const int MICROSTEP = 2;

// Setup LCD (rs, e, d4, d5, d6, d7, d8);
LiquidCrystal lcd(6, 7, 5, 4, 3, 2);
// Axis enum 
enum Axis {
  X,
  Y,
  Z, 
};

// Deadzone and center values
const int DEADZONE = 60;
const int CENTER = 512;
const int MAX_DEVIATION = 512;

// Tunable delay consts for delays between move commands
const float MIN_STEP_DELAY_MS = 2.0;   // full deflection
const float MAX_STEP_DELAY_MS = 40.0;  // lowest deflection

// Joystick Pins
const int vertPin = A0;
const int horPin = A1;
const int buttonPin = 0;
const int zPin = A2;

// Value stores
int vertValue = 511;
int horValue = 511;
int zValue = 511;

// Current position
int xSteps = 0;
int ySteps = 0;
int zSteps = 0;
float xMM = 0;
float yMM = 0;
float zMM = 0;

// Time since last step
unsigned long lastStepTimeX = 0;
unsigned long lastStepTimeY = 0;
unsigned long lastStepTimeZ = 0;

// Step amount toggled with joystick
unsigned int button = 0;
int stepAmount = 2;
unsigned int previousB = LOW;

// Limit switches
const int Y_LIMIT_MIN = 52;
const int Y_LIMIT_MAX = 53;
const int X_LIMIT_MIN = 42;
const int X_LIMIT_MAX = 43;
const int Z_LIMIT_MAX = 33;


// Total possible steps on each axis
unsigned int xTotalSteps = 0;
unsigned int yTotalSteps = 0;
unsigned int zTotalSteps = 0;

// Current possition in steps
unsigned int xCurrentSteps = 0;
unsigned int yCurrentSteps = 0;
unsigned int zCurrentSteps = 0;

// returns a signed "speed" value
// magnitude 0.0-1.0 based on a squared curve for fine control near center
float getAxisSpeed(int rawValue) {
  int deviation = rawValue - CENTER;

  if (abs(deviation) < DEADZONE) return 0.0;

  // remove deadzone so output starts at 0 right past the deadzone edge
  int sign = (deviation > 0) ? 1 : -1;
  float magnitude = (float)(abs(deviation) - DEADZONE) / (MAX_DEVIATION - DEADZONE);
  magnitude = constrain(magnitude, 0.0, 1.0);

  // squared curve: gentle near center, aggressive near full deflection
  float shaped = magnitude * magnitude;

  return sign * shaped; // range: -1.0 to 1.0
}

// 25 steps should equal 1mm
float stepsToMM(int steps) {
  return float(steps) / 25;
}

// Speed between 0.0-1.0.
// lastStepTime is the time of the last step.
// stepDirectionSign is a flipable boolean to adjust which axis moves what way on joystick
int driveAxis(float speed, unsigned long &lastStepTime, int stepDirectionSign, int stepAmount, A4988 stepper, Axis axis, int minTrigger, int maxTrigger) {
  if (speed == 0.0) return;

  // map |speed| (0-1) to a step delay (inverse relationship: faster speed = shorter delay)
  float delayMs = MAX_STEP_DELAY_MS - (fabs(speed) * (MAX_STEP_DELAY_MS - MIN_STEP_DELAY_MS));

  int steps = 0;
  if (millis() - lastStepTime >= delayMs) { 
    if ( stepDirectionSign < 0 && digitalRead(minTrigger)) {
      return 0;
    }
    if ( stepDirectionSign > 0 && digitalRead(maxTrigger)) {
      return 0;
    }
    
    steps = stepAmount*stepDirectionSign;
    lastStepTime = millis();
    int finalSteps;
    switch(axis) {
      case X:
        finalSteps = xSteps + steps;
        if (finalSteps < 0) {
          steps = -xSteps;
        } else if (finalSteps > xTotalSteps) {
          steps = xTotalSteps - xSteps;
        }
        break;
      case Y:
        finalSteps = ySteps + steps;
        if (finalSteps < 0) {
          steps = -ySteps;
        } else if (finalSteps > yTotalSteps) {
          steps = yTotalSteps - ySteps;
        }
        break;
    }
  }
  return steps;
}

int driveAxis(float speed, unsigned long &lastStepTime, int stepDirectionSign, int stepAmount, A4988 stepper, Axis axis, int maxTrigger) {
  if (speed == 0.0) return;

  // map |speed| (0-1) to a step delay (inverse relationship: faster speed = shorter delay)
  float delayMs = MAX_STEP_DELAY_MS - (fabs(speed) * (MAX_STEP_DELAY_MS - MIN_STEP_DELAY_MS));

  int steps = 0;
  if (millis() - lastStepTime >= delayMs) { 
    if ( stepDirectionSign < 0 && zSteps <= 0) {
      return 0;
    }
    if ( stepDirectionSign > 0 && digitalRead(maxTrigger)) {
      return 0;
    }
    
    steps = stepAmount*stepDirectionSign;
    lastStepTime = millis();

    int finalSteps = zSteps + steps;
    if (finalSteps < 0) {
      steps = -zSteps;
    } else if (finalSteps > zTotalSteps) {
      steps = zTotalSteps - zSteps;
    }
  }
  return steps;
}

bool xMinTrigger() {
  return LOW;
  return digitalRead(Y_LIMIT_MIN) == HIGH; // TODO: Is this not dead code because of return above?
}

void calibrateAxis(A4988 stepper, unsigned int *totalSteps, unsigned int *currentSteps, int axisMaxTrigger, int axisMinTrigger) {
  Serial.println("Starting Callibration");
  int stepsForward = 0;
  int stepsBackward = 0;
  const int stepAmount = 75; // 3 mm
  const int closeCalibrationSteps = 50; // 2 mm

  // Move forward until hitting limit switch
  Serial.println("Going forward!");
  while (digitalRead(axisMaxTrigger)) { 
    stepper.move(stepAmount);
    stepsForward += stepAmount;
    delay(5);
  }
  Serial.println("Hit edge!");
  stepper.move((stepAmount + closeCalibrationSteps) * -1); // Move back to starting spot
  stepsForward -= stepAmount + closeCalibrationSteps; // Don't include steps that hit the switch

  // Move slow to get more percise position
  Serial.println("Starting Slow move");
  while (digitalRead(axisMaxTrigger)) {
    stepper.move(5);
    stepsForward += 5;
    delay(5);
  }
  Serial.println("Hit edge!");
  stepper.move(stepsForward * -1);
  stepsForward -= 5;

  // Move backward until hittin limit switch
  Serial.println("Starting backwards");
  while (digitalRead(axisMinTrigger)) {
    stepper.move(stepAmount * -1);
    stepsBackward += stepAmount;
    delay(5);
  }
  Serial.println("Hit edge!");
  stepper.move(stepAmount + closeCalibrationSteps);
  stepsBackward -= stepAmount + closeCalibrationSteps;

  Serial.println("Starting slow move");
  while (digitalRead(axisMinTrigger)) {
    stepper.move(-5);
    stepsBackward += 5;
    delay(5);
  }
  Serial.println("Hit edge!");
  Serial.println("Returning to start point");
  stepper.move(stepsBackward); // Move back to starting spot
  stepsBackward -= 5; // Don't include steps that hit the switch
  
  *currentSteps = stepsBackward;
  
  *totalSteps = stepsForward + stepsBackward;
}

// Assumes that z axis is already possitioned at z = 0
void calibrateZ(unsigned int *totalSteps, unsigned int *currentSteps) {
  unsigned int stepsUp = 0;
  const int stepAmount = 50; // 3 mm
  const int closeCalibrationSteps = 25; // 2 mm
  const int backOffSteps = 100; // Back off 4 mm

  // Rough calibreation
  Serial.println("Starting Calibration");
  while (digitalRead(Z_LIMIT_MAX)) {
    StepperZ.move(stepAmount);
    stepsUp += stepAmount;
    // Serial.println("Steps moved: " + String(stepsUp));
    delay(5);
  }
  Serial.println("Hit top!");
  StepperZ.move(-1*(closeCalibrationSteps + stepAmount));
  // Serial.println("Moved down " + String(closeCalibrationSteps + stepAmount) + "steps");
  stepsUp -= stepAmount + closeCalibrationSteps; // Move back to do precise calibration
  // Serial.println("Steps from bottom: " + String(stepsUp));
  
  Serial.println("Starting slow move");
  while (digitalRead(Z_LIMIT_MAX)) {
    StepperZ.move(5);
    stepsUp += 5;
    // Serial.println("Steps moved " + String(stepsUp));
    delay(5);
  }
  Serial.println("Hit top!");
  StepperZ.move(-5);
  // Serial.println("Moved down 5 steps");
  stepsUp -= 5; // Don't include steps that hit the switch
  // Serial.println("Steps from bottom: " + String(stepsUp));
  // StepperZ.move(-1*backOffSteps); // Back away from top
  *totalSteps = stepsUp;
  *currentSteps = stepsUp;
}

void calibration() {
  calibrateZ(&zTotalSteps, &zSteps);
  Serial.println("Total steps on Z: " + String(zTotalSteps));
  Serial.println("Current steps on Z: "+ String(zSteps));
  calibrateAxis(StepperX, &xTotalSteps, &xSteps, X_LIMIT_MAX, X_LIMIT_MIN);
  Serial.println("Total steps on X: " + String(xTotalSteps));
  Serial.println("Current steps on X: "+ String(xSteps));
  calibrateAxis(StepperY, &yTotalSteps, &ySteps, Y_LIMIT_MAX, Y_LIMIT_MIN);
  Serial.println("Total steps on Y: " + String(yTotalSteps));
  Serial.println("Current steps on Y: "+ String(ySteps));
}

void CheckCalibration(){
  const int yAbsPoint = 1200; // 48 mm
  const int xAbsPoint = 400; // 16 mm
  const int zAbsPoint = 0;
  const int yMove = yAbsPoint - ySteps;
  const int xMove = xAbsPoint - xSteps;
  const int zMove = zAbsPoint - zSteps;
  controller.move(yMove, xMove, zMove);
}

void moveToPoint(int y, int x, int z) {
  const int stepsNeeded[] = {y*25 - ySteps, x*25 - xSteps, z*25 - zSteps};
  controller.move(stepsNeeded[0], stepsNeeded[1], stepsNeeded[2]);
  ySteps += stepsNeeded[0];
  xSteps += stepsNeeded[1];
  zSteps += stepsNeeded[2];
}

void moveToPoint(int y, int x) {
  const int stepsNeeded[] = {y*25 - ySteps, x*25 - xSteps};
  controller.move(stepsNeeded[0], stepsNeeded[1], 0);
  ySteps += stepsNeeded[0];
  xSteps += stepsNeeded[1];
}

void moveToCorner() {
  moveToPoint((yTotalSteps/25) - 10, (xTotalSteps/25) - 10, (zTotalSteps/25) - 10);
}

void drawSquare() {
  moveToPoint(42, 14);
  StepperZ.move(-zSteps);
  zSteps = 0;
  moveToPoint(62, 14);
  moveToPoint(62, 34);
  moveToPoint(42, 34);
  moveToPoint(42, 14);
}

// void checkSkippingSteps() {
//   StepperZ.move(1000);
//   delay(100);
//   StepperZ.move(-1000);
// }

void setup() {
    // Set target motor RPM to 1RPM and microstepping to 1 (full step mode)
    StepperY.begin(60, MICROSTEP);
    StepperX.begin(60, MICROSTEP);
    StepperZ.begin(60, MICROSTEP);
    pinMode(vertPin, INPUT);
    pinMode(horPin, INPUT);
    pinMode(zPin, INPUT);
    pinMode(buttonPin, INPUT);
    pinMode(Y_LIMIT_MAX, INPUT_PULLUP);
    pinMode(Y_LIMIT_MIN, INPUT_PULLUP);
    pinMode(X_LIMIT_MAX, INPUT_PULLUP);
    pinMode(X_LIMIT_MIN, INPUT_PULLUP);
    pinMode(Z_LIMIT_MAX, INPUT_PULLUP);
    Serial.begin(9600);
    lcd.begin(16, 2);
    lcd.setCursor(0, 0);
    lcd.print("Calibrating...");
    delay(2000);
    calibration();
    delay(2000);
    lcd.setCursor(0, 0);
    lcd.println("Drawing square");
    drawSquare();
    moveToCorner();
    lcd.setCursor(0, 0);
    lcd.print("Done!");
    lcd.setCursor(0, 0);
    lcd.print("x:" + String(stepsToMM(xSteps), 1));
    lcd.setCursor(8, 0);
    lcd.print("y:" + String(stepsToMM(ySteps), 1));
    lcd.setCursor(0, 1);
    lcd.print("z:" + String(stepsToMM(zSteps), 1));
}

void loop() {
    // vertValue = analogRead(vertPin);
    // horValue = analogRead(horPin);
    // button = digitalRead(buttonPin);

    // zValue = analogRead(zPin);

    // if ( button == HIGH && previousB == LOW ) {
    //   if ( stepAmount == 2 ) {
    //     stepAmount = 20;
    //   } else {
    //     stepAmount = 2;
    //   }
    // }

    // float speedY = getAxisSpeed(vertValue);
    // float speedX = getAxisSpeed(horValue);
    // float speedZ = getAxisSpeed(zValue);

    // int yMoves = driveAxis(speedY, lastStepTimeY, 1, stepAmount, StepperY, Y, Y_LIMIT_MIN, Y_LIMIT_MAX);
    // int xMoves = driveAxis(speedX, lastStepTimeX, 1, stepAmount, StepperX, X, X_LIMIT_MIN, X_LIMIT_MAX);
    // int zMoves = driveAxis(speedZ, lastStepTimeZ, 1, stepAmount, StepperZ, Z, Z_LIMIT_MAX);
    // controller.move(yMoves, xMoves, zMoves);
    
    // if (yMoves != 0) {
    //   yMM += stepsToMM(yMoves);
    //   lcd.setCursor(8, 0);
    //   lcd.print("y:" + String(yMM,1));
    // }
    
    // if (xMoves != 0) {
    //   xMM += stepsToMM(xMoves);
    //   lcd.setCursor(0, 0);
    //   lcd.print("x:" + String(xMM,1));
    // }
  
    // if (zMoves != 0) {
    //   zMM += stepsToMM(zMoves);
    //   lcd.setCursor(0, 1);
    //   lcd.print("z:" + String(zMM,1));
    // }

    // previousB = button;

}
