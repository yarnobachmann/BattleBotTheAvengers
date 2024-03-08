//-----------------------------------[Include libraries]----------------------------

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

//-----------------------------------[Predeclare functions]-------------------------

void setPixelColor(int pixel, uint8_t red, uint8_t green, uint8_t blue);
void setServoAngle(int angle);
void setGripperAngle(int angle);
void openGripper();
void closeGripper();
void calibrateServo();
void setMotors(double LFWD, double LBACK, double RFWD, double RBACK);
void brakeLight();
void forwardLight();
void leftLight();
void rightLight();
void driveLeft(int speed);
void driveRight(int speed);
void driveForward(int speed);
void driveBack(int speed);
void driveStop();
void lookStraight();
void lookLeft();
void lookRight();
long measureDistance();
void calibrateToDriveStraight();
void leftPulseInterrupt();
void rightPulseInterrupt();

void calculatePulsesFor90DegreeTurn();
void turnLeft90();
void turnRight90();

//-----------------------------------[Declare pins]-------------------------------

#define PIN            13    // Digital pin connected to the Neopixel
#define NUMPIXELS      4    // Number of Neopixels in your strip

const int motorLeftBack =   12;   // Motor pin A1
const int motorLeftFwd =    11;  // Motor pin A2
const int motorRightFwd =   10;  // Motor pin B1
const int motorRightBack =  9;   // Motor pin B2

const int servo = 5; // servo
const int gripper = 6; // gripper GR

const int triggerPin = 8; // HC-SR04 trigger pin
const int echoPin = 4;    // HC-SR04 echo pin

const int motorLeftRead =   2;   // Arduino A0
const int motorRightRead =  3;   // Arduino A1

const int numberOfSensors = 8; // Number of sensors used
int sensorPins[numberOfSensors] = {A7 ,A6, A5, A4, A3, A2, A1, A0}; // Analog pins for the sensors
int sensorValues[numberOfSensors]; // Array to store the sensor values

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

//-----------------------------------[Variables]---------------------------------

volatile unsigned long leftPulse = 0;  // Stores the pulse width of the left motor
volatile unsigned long rightPulse = 0; // Stores the pulse width of the right motor

unsigned long startTime;  // Variable to store the start time for calibration
const int calibrationDuration = 5000;  // Calibration duration in milliseconds
bool calibrated = false;  // Flag to check if calibration is done

const int stopDistance = 10; // Distance threshold to stop the robot (in cm)

int interruptLeft = 0;  // Default value for the left interrupt
int interruptRight = 0;  // Default value for the right interrupt

int rightOffset = 100;  // RightOffset default value set to 100
int leftOffset = 100;  // LeftOffset default value set to 100

const int reflectionThreshold = 500; // Adjust this threshold based on your sensor's characteristics

unsigned long gripperTimer = 0;
const int gripperDelay = 1000; // Delay in milliseconds


int requiredPulsesForTurn = 0; // Variable to store the required pulses for a 90-degree turn
double pi = 3.14159; // Value of PI

//-----------------------------------[Setup function]---------------------------

void setup() {
  strip.begin();  // Initialize the Neopixel strip
  strip.show();   // Initialize all pixels to 'off'

  pinMode(motorLeftFwd,   OUTPUT);
  pinMode(motorLeftBack,  OUTPUT);
  pinMode(motorRightBack, OUTPUT);
  pinMode(motorRightFwd,  OUTPUT);
  pinMode(motorLeftRead,  INPUT);
  pinMode(motorRightRead, INPUT);
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(servo, OUTPUT);
  pinMode(gripper, OUTPUT);
  for(int i = 0;i<=7;i++) 
  {
    pinMode(sensorPins[i], INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(motorLeftRead), leftPulseInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorRightRead), rightPulseInterrupt, CHANGE);
  Serial.begin(9600);
  
  calibrateServo(); // Calibrate the servo to a known initial position

  calculatePulsesFor90DegreeTurn(); // Calculate the pulses needed for a 90-degree turn

  turnLeft90(); // Turn left 90 degrees

  turnRight90(); // Turn right 90 degrees

  turnLeft90(); // Turn left 90 degrees

  turnRight90(); // Turn right 90 degrees

}

//-----------------------------------[Loop function]----------------------------

void loop() {

  turnLeft90(); // Turn left 90 degrees
  delay(1000); // Wait for 1 second
  turnRight90(); // Turn right 90 degrees
  delay(1000); // Wait for 1 second
  
}

//-----------------------------------[testcode]-------------------------------

// Calculate how many pulses are needed to turn 90 degrees
void calculatePulsesFor90DegreeTurn() {
  double wheelCircumference = pi * 6.6; // cm
  double distancePerPulse = wheelCircumference / 40.0; 
  double distanceBetweenWheels = 11.5; // cm

   // Calculate distance to travel for a 90 degree turn (assuming the robot's turning radius is the distance between the wheels)
   double distanceForTurn = (pi / 2.0) * distanceBetweenWheels; // Replace 'distanceBetweenWheels' with the actual value

   // Calculate pulses needed for the turn
  requiredPulsesForTurn = 150;
}

// Turn 90 degrees left 
void turnLeft90() {
  calculatePulsesFor90DegreeTurn(); // Ensure pulses are calculated

  // Reset pulse counters
  noInterrupts(); // Temporarily disable interrupts
  interruptLeft = 0;
  interruptRight = 0;
  interrupts();

  // Turn left until pulses are reached
  while (interruptRight < requiredPulsesForTurn) {
    driveLeft(2); // Adjust speed if needed
  }
  driveStop();
}

// Turn 90 degrees right
void turnRight90() {
  calculatePulsesFor90DegreeTurn(); // Ensure pulses are calculated

  // Reset pulse counters
  noInterrupts(); // Temporarily disable interrupts
  interruptLeft = 0;
  interruptRight = 0;
  interrupts();

  // Turn right until pulses are reached
  while (interruptLeft < requiredPulsesForTurn) {
    driveRight(2); // Adjust speed if needed
  }
  driveStop();
}



//-----------------------------------[Neopixel]-------------------------------

// Set color for a specific Neopixel
void setPixelColor(int pixel, uint8_t red, uint8_t green, uint8_t blue) {
  strip.setPixelColor(pixel, strip.Color(red, green, blue));
  strip.show();
}

//-----------------------------------[Gripper]--------------------------------

void setGripperAngle(int angle) {
  // Convert the angle to the corresponding PWM signal
  int pwmValue = map(angle, 0, 180, 544, 2400);
  // Send the PWM signal to the servo
  digitalWrite(gripper, HIGH);
  delayMicroseconds(pwmValue);
  digitalWrite(gripper, LOW);
}

// Function to open the gripper (move the servo to the open position)
void openGripper() {
  // Adjust the angle value based on your servo's specifications
  int openAngle = 80;
  setGripperAngle(openAngle);
}

// Function to close the gripper (move the servo to the closed position)
void closeGripper() {
  // Adjust the angle value based on your servo's specifications
  int closeAngle = 30;
  setGripperAngle(closeAngle);
}

//-----------------------------------[Echo Servo]-----------------------------

void setServoAngle(int angle) {
  // Convert the angle to the corresponding PWM signal
  int pwmValue = map(angle, 0, 180, 544, 2400);
  // Send the PWM signal to the servo
  digitalWrite(servo, HIGH);
  delayMicroseconds(pwmValue);
  digitalWrite(servo, LOW);
}

// Function to calibrate the servo to find the middle position
void calibrateServo() {
  // Move to the extreme left position
  int rightAngle = 0;
  setServoAngle(rightAngle);
  delay(1000);  // Wait for 1 second to ensure the servo reaches the left position
  // Move to the extreme right position
  int leftAngle = 180;
  setServoAngle(leftAngle);
  delay(1000);  // Wait for 1 second to ensure the servo reaches the right position
  // Calculate and move to the middle position
  int middleAngle = 75;
  setServoAngle(middleAngle);
  delay(1000);  // Wait for 1 second to ensure the servo reaches the middle position
}

//-----------------------------------[Puls sensors]---------------------------

void leftPulseInterrupt() {
  noInterrupts();
  interruptLeft++;
  interrupts();
}

void rightPulseInterrupt() {
  noInterrupts();
  interruptRight++;
  interrupts();
}

//-----------------------------------[Wheels motor]---------------------------

void calibrateToDriveStraight() {
  driveForward(2);
  delay(5000);
  driveStop();
  // Stops the interrupts to prevent interference during calibration
  noInterrupts();
  // Determine the maximum and minimum pulses from left and right interrupts
  double maxPulses = max(interruptLeft, interruptRight);
  double minPulses = min(interruptLeft, interruptRight);
  // Calculate the offset pulses as a percentage based on the minimum and maximum pulses
  double offsetPulses = round((minPulses / maxPulses) * 100);
  // Set the offsets for left and right motors based on the maximum pulses
  rightOffset = (interruptRight == maxPulses) ? offsetPulses : 100;
  leftOffset = (interruptLeft == maxPulses) ? offsetPulses : 100;
  // Print calibration results to Serial Monitor
  Serial.print("Left offset: ");
  Serial.print(leftOffset);
  Serial.print(" | Right offset: ");
  Serial.println(rightOffset);
  Serial.println(offsetPulses);
  Serial.println(maxPulses);
  Serial.println(minPulses);
  // Re-enable interrupts 
  interrupts();
}

// Sets motor power to input
void setMotors(double LFWD, double LBACK, double RFWD, double RBACK) {
  analogWrite(motorLeftFwd,   round(LFWD * leftOffset));
  analogWrite(motorLeftBack,  round(LBACK * leftOffset));
  analogWrite(motorRightFwd,  round(RFWD * rightOffset));
  analogWrite(motorRightBack, round(RBACK * rightOffset));
}

//-----------------------------------[Drive functions]------------------------

// Rotate left at 0-2 speed
void driveLeft(int speed) {
  leftLight();
  setMotors(0, speed, speed, 0);
}

// Rotate right at 0-2 speed
void driveRight(int speed) {
  rightLight();
  setMotors(speed, 0 , 0, speed);
}

// Drive forwards at 0-2 speed
void driveForward(int speed) {
  setMotors(speed, 0 , speed, 0);
  forwardLight(); // Turn on forward lights
}

// Drive backwards at 0-2 speed
void driveBack(int speed) {
  setMotors(0, speed, 0, speed);
}

// Stop driving
void driveStop() {
  setMotors(0, 0, 0, 0);
  brakeLight(); // Turn on brake lights
}

//-----------------------------------[Blinking lights]------------------------

void brakeLight() {
  // Set color for each Neopixel individually
  setPixelColor(0, 0, 255, 0);  // Red for the first Neopixel
  setPixelColor(1, 0, 255, 0);  // Red for the second Neopixel
  setPixelColor(2, 0, 0, 0);    // Turn off other Neopixels
  setPixelColor(3, 0, 0, 0);    // Turn off other Neopixels
}

void forwardLight() {
  // Set color for each Neopixel individually
  setPixelColor(0, 0, 0, 0);  // Turn off other Neopixels
  setPixelColor(1, 0, 0, 0);  // Turn off other Neopixels
  setPixelColor(2, 255, 0, 0);    // green for the 3th Neopixel
  setPixelColor(3, 255, 0, 0);    // green for the 4th Neopixel
}

void leftLight() {
  // Set color for each Neopixel individually
  setPixelColor(0, 0, 0, 0);  // Turn off other Neopixels
  setPixelColor(1, 0, 0, 0);  // Turn off other Neopixels
  setPixelColor(2, 255, 0, 0);    // Turn off other Neopixels
  setPixelColor(3, 80, 255, 0);    // Orange for the 4th Neopixel
}

void rightLight() {
  // Set color for each Neopixel individually
  setPixelColor(0, 0, 0, 0);  // Turn off other Neopixels
  setPixelColor(1, 0, 0, 0);  // Turn off other Neopixels
  setPixelColor(2, 80, 255, 0);    // Orange for the 3th Neopixel
  setPixelColor(3, 255, 0, 0);    // Turn off other Neopixels
}

//-----------------------------------[Echo servo directions]------------------

void lookStraight() {
  // Adjust the angle value based on your servo's specifications
  int angle = 75;
  setServoAngle(angle);
}

void lookLeft() {
  // Adjust the angle value based on your servo's specifications
  int angle = 180;
  setServoAngle(angle);
  // Add a delay to give some time for the servo to move (you may need to adjust the delay duration)
  delay(800);
  // Measure distance
  long distance = measureDistance();
  // Check for obstacles
  if (distance <= stopDistance) {
    lookStraight();
  } else {
    driveLeft(2);
    lookStraight();
  }
}

// Function to close the gripper (move the servo to the closed position)
void lookRight() {
  // Adjust the angle value based on your servo's specifications
  int angle = 0;
  setServoAngle(angle);
  delay(800);
  // Measure distance
  long distance = measureDistance();
  // Check for obstacles
  if (distance <= stopDistance) {
    lookLeft();
  } else {
    driveRight(2);
    lookStraight();
  }
}

//-----------------------------------[Echo sensor]------------------------

long measureDistance()
{
  // Echo detection to measure the distance
  long duration, distance;
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  return distance;
}