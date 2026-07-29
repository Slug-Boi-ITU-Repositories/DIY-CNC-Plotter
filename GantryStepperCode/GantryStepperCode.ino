#include <Arduino.h>
#include "A4988.h"
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
unsigned ling lastStepTimeZ = 0;

// Step amount toggled with joystick
unsigned int button = 0;
int stepAmount = 2;
unsigned int previousB = LOW;

// Limit switches
const int LIMIT_Y_MIN = 2;

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
void driveAxis(float speed, unsigned long &lastStepTime, int stepDirectionSign, int stepAmount, A4988 stepper, Axis axis) {
  if (speed == 0.0) return;

  // map |speed| (0-1) to a step delay (inverse relationship: faster speed = shorter delay)
  float delayMs = MAX_STEP_DELAY_MS - (fabs(speed) * (MAX_STEP_DELAY_MS - MIN_STEP_DELAY_MS));

  if (millis() - lastStepTime >= delayMs) {
    int dir = (speed > 0 ? 1 : -1) * stepDirectionSign; // TODO: is dir not the same as stepDirectionSign. Also speed is always > 0 here. 
    if ( dir < 0 && xMinTrigger()) {
      return;
    }

    stepper.move(stepAmount*dir);
    if ( axis == X) {
      xMM += stepsToMM(stepAmount*dir);
      lcd.setCursor(0, 0);
      lcd.print("x:" + String(xMM,1));
    } else if ( axis == Y ) {
      yMM += stepsToMM(stepAmount*dir);
      lcd.setCursor(8, 0);
      lcd.print("y:" + String(yMM,1));
    } else {
      zMM += stepsToMM(stepAmount*dir);
      lcd.setCursor(0, 1);
      lcd.print("z:" + String(zMM,1));
    }
    
    
    lastStepTime = millis();
  }
}

bool xMinTrigger() {
  return LOW;
  return digitalRead(LIMIT_Y_MIN) == HIGH; // TODO: Is this not dead code because of return above?
}

void calibrateAxis(A4988 stepper) {
  int stepsForward = 0;
  int stepsBackward = 0;
  const int stepAmount = 5;

  // Move forward until hitting limit switch
  while (!axisMaxTrigger) { 
    stepper.move(stepAmount);
    stepsForward += stepAmount;
  }
  stepper.move(stepsForward * -1); // Move back to starting spot

  // Move backward until hittin limit switch
  while (!axisMinTrigger) {
    stepper.move(stepAmount * -1);
    stepsBackward += stepAmount;
  }
  stepper.move(stepsBackward); // Move back to starting spot
}

void calibration() {
  calibrateAxis(StepperX);
  calibrateAxis(StepperY);
}

void setup() {
    // Set target motor RPM to 1RPM and microstepping to 1 (full step mode)
    StepperY.begin(60, 1);
    StepperX.begin(60, 1);
    StepperZ.begin(60, 1);
    pinMode(vertPin, INPUT);
    pinMode(horPin, INPUT);
    pinMode(zPin, INPUT);
    pinMode(buttonPin, INPUT);
    pinMode(LIMIT_Y_MIN, INPUT_PULLUP);
    Serial.begin(9600);
    lcd.begin(16, 2);
    lcd.setCursor(0, 0);
    lcd.print("x:" + String(xMM, 1));
    lcd.setCursor(8, 0);
    lcd.print("y:" + String(yMM, 1));
    lcd.setCursor(0, 1);
    lcd.print("z:" + String(zMM, 1));
}

void loop() {
    vertValue = analogRead(vertPin);
    horValue = analogRead(horPin);
    button = digitalRead(buttonPin);

    zValue = analogRead(zPin);

    if ( button == HIGH && previousB == LOW ) {
      if ( stepAmount == 2 ) {
        stepAmount = 20;
      } else {
        stepAmount = 2;
      }
    }

    float speedY = getAxisSpeed(vertValue);
    float speedX = getAxisSpeed(horValue);
    float speedZ = getAxisSpeed(zValue);

    driveAxis(speedY, lastStepTimeY, 1, stepAmount, StepperY, Y);
    driveAxis(speedX, lastStepTimeX, 1, stepAmount, StepperX, X);
    driveAxis(speedZ, lastStepTimeZ, 1, stepAmount, StepperZ, Z);

    previousB = button; 
}
