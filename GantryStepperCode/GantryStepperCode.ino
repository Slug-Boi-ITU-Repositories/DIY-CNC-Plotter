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

A4988 StepperX(MOTOR_STEPS, DIR_X, STEP_X);
A4988 StepperY(MOTOR_STEPS, DIR_Y, STEP_Y);
A4988 StepperZ(MOTOR_STEPS, DIR_Z, STEP_Z);

MultiDriver controller(StepperX, StepperY, StepperZ);

const int MICROSTEP = 8;

// Setup LCD (rs, e, d4, d5, d6, d7, d8);
LiquidCrystal lcd(6, 7, 5, 4, 3, 2);
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
const float MIN_STEP_DELAY_MS = 2.0;   // full deflection
const float MAX_STEP_DELAY_MS = 40.0;  // lowest deflection

// Calibration delay for stick press
const int CALIBRATION_DELAY = 1500;
int lastPressedCalibration = 0;

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
unsigned int L_Button = 0;
unsigned int R_Button = 0;
int stepAmount = 100;
int stepAmountZ = 50;
unsigned int previousRB = HIGH;
unsigned int previousLB = HIGH;

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
  return float(steps) / (25*MICROSTEP);
}

int MMToSteps(float mm) {
  return int(mm * 25 * MICROSTEP);
}

// Speed between 0.0-1.0.
// lastStepTime is the time of the last step.
// stepDirectionSign is a flipable boolean to adjust which axis moves what way on joystick
int driveAxis(float speed, unsigned long &lastStepTime, int stepAmount, A4988 stepper, Axis axis, int minTrigger, int maxTrigger) {
  if (speed == 0.0) return 0;

  // map |speed| (0-1) to a step delay (inverse relationship: faster speed = shorter delay)
  float delayMs = MAX_STEP_DELAY_MS - (fabs(speed) * (MAX_STEP_DELAY_MS - MIN_STEP_DELAY_MS));

  int steps = 0;
  if (millis() - lastStepTime >= delayMs) { 
    if ( speed < 0 && !digitalRead(minTrigger)) {
      Serial.println("ONE");
      return 0;
    }
    if ( speed > 0 && !digitalRead(maxTrigger)) {
      Serial.println("TWO");
      return 0;
    }
    
    int dir = speed < 0 ? -1 : 1; 

    steps = stepAmount*dir;
    lastStepTime = millis();
    int finalSteps;
    // switch(axis) {
    //   case X:
    //     finalSteps = xSteps + steps;
    //     if (finalSteps < 0) {
    //       steps = -xSteps;
    //     } else if (finalSteps > xTotalSteps) {
    //       steps = xTotalSteps - xSteps;
    //     }
    //     break;
    //   case Y:
    //     finalSteps = ySteps + steps;
    //     if (finalSteps < 0) {
    //       steps = -ySteps;
    //     } else if (finalSteps > yTotalSteps) {
    //       steps = yTotalSteps - ySteps;
    //     }
    //     break;
    // }
  }
  return steps;
}

int driveAxis(float speed, unsigned long &lastStepTime, int stepAmount, A4988 stepper, Axis axis, int maxTrigger) {
  if (speed == 0.0) return 0;

  // map |speed| (0-1) to a step delay (inverse relationship: faster speed = shorter delay)
  float delayMs = MAX_STEP_DELAY_MS - (fabs(speed) * (MAX_STEP_DELAY_MS - MIN_STEP_DELAY_MS));

  int steps = 0;
  if (millis() - lastStepTime >= delayMs) { 
    if ( speed > 0 && !digitalRead(maxTrigger)) {
      return 0;
    }

    int dir = speed < 0 ? -1 : 1; 
    
    steps = stepAmount*dir;
    lastStepTime = millis();

    int finalSteps = zSteps + steps;
    // if (finalSteps < 0) {
    //   steps = -zSteps;
    // } else if (finalSteps > zTotalSteps) {
    //   steps = zTotalSteps - zSteps;
    // }
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
  const int stepAmount = 75 * MICROSTEP; // 3 mm
  const int closeCalibrationSteps = 50 * MICROSTEP; // 2 mm

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
    stepper.move(1);
    stepsForward += 1;
    delay(5);
  }
  Serial.println("Hit edge!");
  stepper.move(stepsForward * -1);
  stepsForward -= 1;

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
    stepper.move(-1);
    stepsBackward += 1;
    delay(5);
  }
  Serial.println("Hit edge!");
  // Serial.println("Returning to start point");
  stepper.move(50); // Move back to starting spot
  // stepsBackward -= 1; // Don't include steps that hit the switch
  
  *currentSteps = 50;
  
  *totalSteps = stepsForward + stepsBackward;
}

// Assumes that z axis is already possitioned at z = 0
void calibrateZ(unsigned int *totalSteps, unsigned int *currentSteps) {
  unsigned int stepsUp = 0;
  const int stepAmount = 50 * MICROSTEP; // 3 mm
  const int closeCalibrationSteps = 25 * MICROSTEP; // 2 mm
  const int backOffSteps = 75; // Back off 4 mm

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
    StepperZ.move(1);
    stepsUp += 1;
    // Serial.println("Steps moved " + String(stepsUp));
    delay(5);
  }
  Serial.println("Hit top!");
  StepperZ.move(-5);
  // Serial.println("Moved down 5 steps");
  stepsUp -= 1; // Don't include steps that hit the switch
  // Serial.println("Steps from bottom: " + String(stepsUp));
  // StepperZ.move(-1*backOffSteps); // Back away from top
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

void CheckCalibration(){
  const int yAbsPoint = 1200; // 48 mm
  const int xAbsPoint = 400; // 16 mm
  const int zAbsPoint = 0;
  const int xMove = xAbsPoint - xSteps;
  const int yMove = yAbsPoint - ySteps;
  const int zMove = zAbsPoint - zSteps;
  controller.move(xMove, yMove, zMove);
}

void moveToPoint(float x, float y, float z) {
  const int stepsNeeded[] = {MMToSteps(x) - xSteps, MMToSteps(y) - ySteps, MMToSteps(z) - zSteps};
  controller.move(stepsNeeded[0], stepsNeeded[1], stepsNeeded[2]);
  xSteps += stepsNeeded[0];
  ySteps += stepsNeeded[1];
  zSteps += stepsNeeded[2];
}

void moveToPoint(float x, float y) {
  const int stepsNeeded[] = {MMToSteps(x) - xSteps, MMToSteps(y) - ySteps};
  controller.move(stepsNeeded[0], stepsNeeded[1], 0);
  xSteps += stepsNeeded[0];
  ySteps += stepsNeeded[1];
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

void drawSquare() {
  moveToPoint(14, 42);
  penToPaper();
  moveToPoint(14, 62);
  moveToPoint(34, 62);
  moveToPoint(34, 42);
  moveToPoint(14, 42);
}

void drawRhombus() {
  moveToPoint(34, 42);
  penToPaper();
  moveToPoint(44, 52);
  moveToPoint(34, 62);
  moveToPoint(24, 52);
  moveToPoint(34, 42);
}

void drawParallelLines() {
  lcd.setCursor(0,0);
  lcd.println("Drawing P Lines");
  moveToPoint(68, 100);
  int offset = 10;
  for (int i = 0; i < 5; i++ ) {
    penToPaper();
    moveToPoint(104, 100+offset*i);
    raisePen(10); 
    moveToPoint(68, 100+offset*(i+1));
  } 
}

void drawCircle() {
  const int centerX = 75;
  const int centerY = 150;
  const int radius = 20;
  const int segments = 10000; // more segments = smoother circle, but slower

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

void repeatDraw(int repeats, void (*draw)()) {
  for ( int i = 0; i < repeats; i++ ) {
    draw();
  }
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
    pinMode(leftButtonPin, INPUT_PULLUP);
    pinMode(rightButtonPin, INPUT_PULLUP);
    pinMode(Y_LIMIT_MAX, INPUT_PULLUP);
    pinMode(Y_LIMIT_MIN, INPUT_PULLUP);
    pinMode(X_LIMIT_MAX, INPUT_PULLUP);
    pinMode(X_LIMIT_MIN, INPUT_PULLUP);
    pinMode(Z_LIMIT_MAX, INPUT_PULLUP);
    Serial.begin(9600);
    lcd.begin(16, 2);
    // for (int i = 0; i < 3; i++ ) {
    //   delay(2000);
    //   calibration();
    //   delay(2000);
    //   repeatDraw(2, drawParallelLines);
    //   penToPaper();
    // }
    
    //drawSquare();
    //drawRhombus();
    lcd.setCursor(0, 0);
    lcd.print("Done!");
    // lcd.setCursor(0, 0);
    // lcd.print("x:" + String(stepsToMM(xSteps), 1));
    // lcd.setCursor(8, 0);
    // lcd.print("y:" + String(stepsToMM(ySteps), 1));
    // lcd.setCursor(0, 1);
    // lcd.print("z:" + String(stepsToMM(zSteps), 1));
}

void loop() {
    vertValue = analogRead(vertPin);
    horValue = analogRead(horPin);
    zValue = analogRead(zPin);
    L_Button = digitalRead(leftButtonPin);
    // Serial.println(L_Button);
    R_Button = digitalRead(rightButtonPin);
    // Serial.println(R_Button);

    // Serial.println("y speed: " + String(vertValue));
    // Serial.println("x speed: " + String(horValue));
    // Serial.println("z speed: " + String(zValue));

 

    if ( R_Button == LOW && previousRB == HIGH ) {
      lcd.setCursor(0, 0);
      lcd.print("Moving to corner");
      moveToCorner();
    }

    if ( L_Button == LOW && previousLB == HIGH ) {
      if ( millis() - lastPressedCalibration < CALIBRATION_DELAY ) {
        lcd.setCursor(0, 0);
        lcd.print("Calibrating...");
        calibration();
        drawCircle();
      }

      lastPressedCalibration = millis();
    }

    float speedY = getAxisSpeed(vertValue);
    float speedX = getAxisSpeed(horValue); 
    float speedZ = getAxisSpeed(zValue) * -1;

    // Serial.println("y speed: " + String(speedY));
    // Serial.println("x speed: " + String(speedX));
    // Serial.println("z speed: " + String(speedZ));
    

    int yMoves = driveAxis(speedY, lastStepTimeY, stepAmount, StepperY, Y, Y_LIMIT_MIN, Y_LIMIT_MAX);
    int xMoves = driveAxis(speedX, lastStepTimeX, stepAmount, StepperX, X, X_LIMIT_MIN, X_LIMIT_MAX);
    int zMoves = driveAxis(speedZ, lastStepTimeZ, stepAmountZ, StepperZ, Z, Z_LIMIT_MAX);
    controller.move(xMoves, yMoves, zMoves);

    // Serial.println("y speed: " + String(yMoves));
    // Serial.println("x speed: " + String(xMoves));
    // Serial.println("z speed: " + String(zMoves));
    
    if (yMoves != 0) {
      yMM += stepsToMM(yMoves);
      lcd.setCursor(8, 0);
      lcd.print("y:" + String(yMM,1));
    }
    
    if (xMoves != 0) {
      xMM += stepsToMM(xMoves);
      lcd.setCursor(0, 0);
      lcd.print("x:" + String(xMM,1));
    }
  
    if (zMoves != 0) {
      zMM += stepsToMM(zMoves);
      lcd.setCursor(0, 1);
      lcd.print("z:" + String(zMM,1));
    }

    previousRB = R_Button;
    previousLB = L_Button;
}
