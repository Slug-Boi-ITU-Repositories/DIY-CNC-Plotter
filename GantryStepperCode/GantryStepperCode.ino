#include <Arduino.h>
#include "A4988.h"
#include <MultiDriver.h>
#include "LiquidCrystal.h"

// using a 200-step motor
#define MOTOR_STEPS 200
// configure the pins connected
#define DIR_Y 8
#define STEP_Y 9
#define DIR_X 12
#define STEP_X 10
#define DIR_Z 13 
#define STEP_Z 11
#define MS1 22
#define MS2 24
#define MS3 26

A4988 StepperX(MOTOR_STEPS, DIR_X, STEP_X, MS1, MS2, MS3);
A4988 StepperY(MOTOR_STEPS, DIR_Y, STEP_Y, MS1, MS2, MS3);
A4988 StepperZ(MOTOR_STEPS, DIR_Z, STEP_Z, MS1, MS2, MS3);

MultiDriver controller(StepperX, StepperY, StepperZ);

const int MICROSTEP = 8;

// Setup LCD (rs, e, d4, d5, d6, d7, d8);
LiquidCrystal lcd(6, 7, 4, 3, 5, 2);
// Axis enum 
enum Axis {
  X,
  Y,
  Z, 
};

// Deadzone and center values
const int DEADZONE = 75;
const int CENTER = 512;
const int MAX_DEVIATION = 512;

// Tunable delay consts for delays between move commands
const float MIN_STEP_DELAY_MS = 0.0;   // full deflection
const float MAX_STEP_DELAY_MS = 80.0;  // lowest deflection

// Calibration delay for stick press
const int CALIBRATION_DELAY = 1500;
unsigned long lastPressedCalibration = 0;

// Joystick Pins
const int vertPin = A0;
const int horPin = A1;
const int leftButtonPin = 46;
const int rightButtonPin = 47; 
const int zPin = A2;

// Value stores
int vertValue = 511;
int horValue = 511;
int zValue = 511;

// Current position
long xSteps = 0;
long ySteps = 0;
long zSteps = 0;
float xMM = 0;
float yMM = 0;
float zMM = 0;

// Time since last step
unsigned long lastStepTimeX = 0;
unsigned long lastStepTimeY = 0;
unsigned long lastStepTimeZ = 0;

// Step amount toggled with joystick
unsigned int L_Button = 0;
unsigned int R_Button = 0;
int stepAmount = 25 * MICROSTEP;
int stepAmountZ = 5 * MICROSTEP;
unsigned int previousRB = HIGH;
unsigned int previousLB = HIGH;

// Limit switches
const int Y_LIMIT_MIN = 52;
const int Y_LIMIT_MAX = 53;
const int X_LIMIT_MIN = 42;
const int X_LIMIT_MAX = 43;
const int Z_LIMIT_MAX = 33;


// Total possible steps on each axis
unsigned long xTotalSteps = 0;
unsigned long yTotalSteps = 0;
unsigned long zTotalSteps = 0;

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
float stepsToMM(long steps) {
  return float(steps / (25*MICROSTEP));
}

long MMToSteps(float mm) {
  return long(mm * 25 * MICROSTEP);
}

// Speed between -1.0 - 1.0.
// lastStepTime is the time of the last step.
// stepDirectionSign is a flipable boolean to adjust which axis moves what way on joystick
int driveAxis(float speed, unsigned long &lastStepTime, int stepAmount, A4988 stepper, Axis axis, int minTrigger, int maxTrigger) {
  if (speed == 0.0) return 0;

  // map |speed| (0-1) to a step delay (inverse relationship: faster speed = shorter delay)
  float delayMs = MAX_STEP_DELAY_MS - (fabs(speed) * (MAX_STEP_DELAY_MS - MIN_STEP_DELAY_MS));

  int steps = 0;
  if ( speed < 0 && !digitalRead(minTrigger)) {
    return 0;
  }
  if ( speed > 0 && !digitalRead(maxTrigger)) {
    return 0;
  }
  
  int dir = speed < 0 ? -1 : 1; 

  steps = stepAmount*speed;
  lastStepTime = millis();
  int finalSteps;
  if (xTotalSteps > 0) {
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

int driveAxis(float speed, unsigned long &lastStepTime, int stepAmount, A4988 stepper, Axis axis, int maxTrigger) {
  if (speed == 0.0) return 0;

  // map |speed| (0-1) to a step delay (inverse relationship: faster speed = shorter delay)
  float delayMs = MAX_STEP_DELAY_MS - (fabs(speed) * (MAX_STEP_DELAY_MS - MIN_STEP_DELAY_MS));

  int steps = 0; 
  if ( speed > 0 && !digitalRead(maxTrigger)) {
    return 0;
  }

  int dir = speed < 0 ? -1 : 1; 
  
  steps = stepAmount*speed;
  lastStepTime = millis();

  int finalSteps = zSteps + steps;
  if (zTotalSteps > 0) {
    if (finalSteps < 0) {
      steps = -zSteps;
    } else if (finalSteps > zTotalSteps) {
      steps = zTotalSteps - zSteps;
    }
  }
  return steps;
}

void calibrateAxis(A4988 stepper, unsigned long *totalSteps, unsigned long *currentSteps, int axisMaxTrigger, int axisMinTrigger) {
  Serial.println("Starting Callibration");
  long stepsForward = 0;
  long stepsBackward = 0;
  const int stepAmount = 50 * MICROSTEP; // 2 mm
  const int closeCalibrationSteps = 10 * MICROSTEP;

  // Move forward until hitting limit switch
  while (digitalRead(axisMaxTrigger)) { 
    stepper.move(stepAmount);
    stepsForward += stepAmount;
    delay(5);
  }
  stepper.move((stepAmount + closeCalibrationSteps) * -1); // Move back to starting spot
  stepsForward -= stepAmount + closeCalibrationSteps; // Don't include steps that hit the switch

  // Move slow to get more percise position
  while (digitalRead(axisMaxTrigger)) {
    stepper.move(1);
    stepsForward += 1;
    delay(5);
  }
  stepper.move(-1);
  delay(5);
  stepsForward -= 1;
  stepper.move(stepsForward * -1);

  // Move backward until hittin limit switch
  while (digitalRead(axisMinTrigger)) {
    stepper.move(stepAmount * -1);
    stepsBackward += stepAmount;
    delay(5);
  }
  stepper.move(stepAmount + closeCalibrationSteps);
  stepsBackward -= stepAmount + closeCalibrationSteps;

  while (digitalRead(axisMinTrigger)) {
    stepper.move(-1);
    stepsBackward += 1;
    delay(15);
  }
  stepper.move(1);
  stepsBackward -= 1; // Don't include steps that hit the switch
  
  *currentSteps = 0;
  
  *totalSteps = stepsForward + stepsBackward;
}

// Assumes that z axis is already possitioned at z = 0
void calibrateZ(unsigned long *totalSteps, unsigned long *currentSteps) {
  unsigned int stepsUp = 0;
  const int stepAmount = 50 * MICROSTEP; // 2 mm
  const int closeCalibrationSteps = 10 * MICROSTEP;

  // Rough calibreation
  while (digitalRead(Z_LIMIT_MAX)) {
    StepperZ.move(stepAmount);
    stepsUp += stepAmount;
    delay(5);
  }
  StepperZ.move(-1*(closeCalibrationSteps + stepAmount));
  stepsUp -= stepAmount + closeCalibrationSteps; // Move back to do precise calibration

  while (digitalRead(Z_LIMIT_MAX)) {
    StepperZ.move(1);
    stepsUp += 1;
    delay(15);
  }
  StepperZ.move(-5);
  stepsUp -= 1; // Don't include steps that hit the switch
  *totalSteps = stepsUp;
  *currentSteps = stepsUp - 4;
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

long checkMoveWithinRange(long numSteps, long currentSteps, long totalSteps) {
  if (numSteps + currentSteps > totalSteps) {
    return totalSteps - currentSteps;
  }
  if (numSteps + currentSteps < 0) {
    return -1 * currentSteps;
  }
  return numSteps;
}

void moveToPoint(float x, float y, float z) {
  long stepsNeeded[] = {
    checkMoveWithinRange(MMToSteps(x) - xSteps, xSteps, xTotalSteps), 
    checkMoveWithinRange(MMToSteps(y) - ySteps, ySteps, yTotalSteps),
    checkMoveWithinRange(MMToSteps(z) - zSteps, zSteps, zTotalSteps)
  };

  controller.move(stepsNeeded[0], stepsNeeded[1], stepsNeeded[2]);
  xSteps += stepsNeeded[0];
  ySteps += stepsNeeded[1];
  zSteps += stepsNeeded[2];
}

void moveToPoint(float x, float y) {
  moveToPoint(x, y, stepsToMM(zSteps));
}

void moveToCorner() {
  if ( yTotalSteps == 0 ) {
    return;
  }
  moveToPoint((stepsToMM(xTotalSteps)) - 10, (stepsToMM(yTotalSteps)) - 10, (stepsToMM(zTotalSteps)) - 10);
  penToPaper();
}

void penToPaper() {
  StepperZ.move(-zSteps);
  zSteps = 0; 
}

void raisePen() {
  StepperZ.move((zTotalSteps - zSteps) - MMToSteps(10)); // 5 mm from top
  zSteps = zTotalSteps - MMToSteps(10);
}

void raisePen(int amount) {
  StepperZ.move(MMToSteps(amount));
  zSteps += MMToSteps(amount);
}

void drawSquare(int lowerLeft_X, int lowerLeft_Y) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.println("Drawing Square");

  int size = 30; 
  int upperLeft_X = lowerLeft_X;
  int upperLeft_Y = lowerLeft_Y + size; 
  int upperRight_X = upperLeft_X + size;
  int upperRight_Y = upperLeft_Y;
  int lowerRight_X = upperRight_X;
  int lowerRight_Y = upperRight_Y - size;
  moveToPoint(lowerLeft_X, lowerLeft_Y);
  penToPaper();
  moveToPoint(upperLeft_X, upperLeft_Y);
  moveToPoint(upperRight_X, upperRight_Y);
  moveToPoint(lowerRight_X, lowerRight_Y);
  moveToPoint(lowerLeft_X, lowerLeft_Y);
}

void drawRhombus(int lowerLeft_X, int lowerLeft_Y) {
  int size = 30;
  int half = size / 2;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.println("Drawing Rhombus");

  int bottom_X = lowerLeft_X + half;
  int bottom_Y = lowerLeft_Y;
  int right_X = lowerLeft_X + size;
  int right_Y = lowerLeft_Y + half;
  int top_X = lowerLeft_X + half;
  int top_Y = lowerLeft_Y + size;
  int left_X = lowerLeft_X;
  int left_Y = lowerLeft_Y + half;

  moveToPoint(bottom_X, bottom_Y);
  penToPaper();
  moveToPoint(right_X, right_Y);
  moveToPoint(top_X, top_Y);
  moveToPoint(left_X, left_Y);
  moveToPoint(bottom_X, bottom_Y);
}

void drawParallelLines(int lowerLeft_X, int lowerLeft_Y) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.println("Drawing P Lines");

  int width = 36;
  int lineCount = 5;
  int offset = 10;

  int leftX = lowerLeft_X;
  int rightX = lowerLeft_X + width;
  int bottomY = lowerLeft_Y;

  moveToPoint(leftX, bottomY);
  for (int i = 0; i < lineCount; i++) {
    penToPaper();
    moveToPoint(rightX, bottomY + offset * i);
    raisePen(10);
    moveToPoint(leftX, bottomY + offset * (i + 1));
  }
}

void drawCircle(int center_X, int center_Y) {
  const int centerX = center_X;
  const int centerY = center_Y;
  const int radius = 10;
  const int segments = 10000; // more segments = smoother circle, but slower

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.println("Drawing Circle");

  // Move to the starting point on the circle (angle = 0), pen up
  moveToPoint(centerX + radius, centerY);

  // Lower the pen
  penToPaper();

  // Step around the circle back to the start
  for (int i = 1; i <= segments; i++) {
    float angle = (2.0 * PI * i) / segments;
    float x = centerX + radius * cos(angle);
    float y = centerY + radius * sin(angle);
    moveToPoint(x, y);
  }
}

void drawCircle(int center_X, int center_Y, int radius = 20) {
  const int centerX = center_X;
  const int centerY = center_Y;
  const int segments = 10000; // more segments = smoother circle, but slower

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.println("Drawing Circle");

  // Move to the starting point on the circle (angle = 0), pen up
  moveToPoint(centerX + radius, centerY);

  // Lower the pen
  penToPaper();

  // Step around the circle back to the start
  for (int i = 1; i <= segments; i++) {
    float angle = (2.0 * PI * i) / segments;
    float x = centerX + radius * cos(angle);
    float y = centerY + radius * sin(angle);
    moveToPoint(x, y);
  }
}

void drawRectangle(int lowerLeft_X, int lowerLeft_Y, int width, int height) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.println("Drawing rectangle");

  int upperLeft_X = lowerLeft_X;
  int upperLeft_Y = lowerLeft_Y + height;
  int upperRight_X = upperLeft_X + width;
  int upperRight_Y = upperLeft_Y;
  int lowerRight_X = upperRight_X;
  int lowerRight_Y = lowerLeft_Y;

  moveToPoint(lowerLeft_X, lowerLeft_Y);
  penToPaper();
  moveToPoint(upperLeft_X, upperLeft_Y);
  moveToPoint(upperRight_X, upperRight_Y);
  moveToPoint(lowerRight_X, lowerRight_Y);
  moveToPoint(lowerLeft_X, lowerLeft_Y);
}

void fullTestRun() {
  // repeat calibration and draw cycle 
  int tests = 3;
  for (int i = 0; i < tests; i++ ) {
    calibration();
    repeatDraw(tests, drawSquare, 10, 100);
    repeatDraw(tests, drawRhombus, 70, 130);
    repeatDraw(tests, drawParallelLines, 100, 85);
    repeatDraw(tests, drawCircle, 60, 120);
    penToPaper();
  }
}

void repeatDraw(int repeats, void (*draw)(int, int), int arg1, int arg2) {
  for ( int i = 0; i < repeats; i++ ) {
    draw(arg1, arg2);
  }
  raisePen();
}

void setup() {
    StepperY.begin(120, MICROSTEP);
    StepperX.begin(120, MICROSTEP);
    StepperZ.begin(120, MICROSTEP);
    pinMode(vertPin, INPUT);
    pinMode(horPin, INPUT);
    pinMode(zPin, INPUT);
    pinMode(leftButtonPin, INPUT_PULLUP);
    pinMode(rightButtonPin, INPUT_PULLUP);
    pinMode(Y_LIMIT_MAX, INPUT_PULLUP);
    pinMode(Y_LIMIT_MIN, INPUT_PULLUP);
    pinMode(X_LIMIT_MAX, INPUT_PULLUP);
    pinMode(X_LIMIT_MIN, INPUT_PULLUP);
    pinMode(Z_LIMIT_MAX, INPUT_PULLUP);
    Serial.begin(9600);
    lcd.begin(16, 2);
    lcd.setCursor(0, 0);
    lcd.print("Welcome!");
}

void loop() {
    vertValue = analogRead(vertPin);
    horValue = analogRead(horPin);
    zValue = analogRead(zPin);
    L_Button = digitalRead(leftButtonPin);
    R_Button = digitalRead(rightButtonPin);

    if ( R_Button == LOW && previousRB == HIGH ) {
      lcd.setCursor(0, 0);
      lcd.print("Moving to corner");
      moveToCorner();

      lcd.clear();
      yMM = stepsToMM(ySteps);
      lcd.setCursor(8, 0);
      lcd.print("y:" + String(yMM,1));
      xMM = stepsToMM(xSteps);
      lcd.setCursor(0, 0);
      lcd.print("x:" + String(xMM,1));
      zMM = stepsToMM(zSteps);
      lcd.setCursor(0, 1);
      lcd.print("z:" + String(zMM,1));
    }

    if ( L_Button == LOW && previousLB == HIGH ) {
      if ( millis() - lastPressedCalibration < CALIBRATION_DELAY ) {
        lcd.setCursor(0, 0);
        lcd.print("Calibrating...");
        calibration();

        lcd.clear();
        yMM = stepsToMM(ySteps);
        lcd.setCursor(8, 0);
        lcd.print("y:" + String(yMM,1));
        xMM = stepsToMM(xSteps);
        lcd.setCursor(0, 0);
        lcd.print("x:" + String(xMM,1));
        zMM = stepsToMM(zSteps);
        lcd.setCursor(0, 1);
        lcd.print("z:" + String(zMM,1));
      }

      lastPressedCalibration = millis();
    }

    float speedY = getAxisSpeed(vertValue);
    float speedX = getAxisSpeed(horValue); 
    float speedZ = getAxisSpeed(zValue) * -1;
    
    int yMoves = driveAxis(speedY, lastStepTimeY, stepAmount, StepperY, Y, Y_LIMIT_MIN, Y_LIMIT_MAX);
    int xMoves = driveAxis(speedX, lastStepTimeX, stepAmount, StepperX, X, X_LIMIT_MIN, X_LIMIT_MAX);
    int zMoves = driveAxis(speedZ, lastStepTimeZ, stepAmountZ, StepperZ, Z, Z_LIMIT_MAX);
    xSteps += xMoves;
    ySteps += yMoves;
    zSteps += zMoves;
    controller.move(xMoves, yMoves, zMoves);
    
    if ( xTotalSteps == 0 ) {
      lcd.setCursor(0,0);
      lcd.println("Please");
      lcd.setCursor(0,1);
      lcd.println("Calibrate");
    } else {
      if (yMoves != 0) {
        yMM = stepsToMM(ySteps);
        lcd.setCursor(8, 0);
        lcd.print("y:" + String(yMM,1));
      }
      
      if (xMoves != 0) {
        xMM = stepsToMM(xSteps);
        lcd.setCursor(0, 0);
        lcd.print("x:" + String(xMM,1));
      }
    
      if (zMoves != 0) {
        zMM = stepsToMM(zSteps);
        lcd.setCursor(0, 1);
        lcd.print("z:" + String(zMM,1));
      }
    }
    

    previousRB = R_Button;
    previousLB = L_Button;
}
