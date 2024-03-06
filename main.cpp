#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN            13    // Digital pin connected to the Neopixel
#define NUMPIXELS      4    // Number of Neopixels in your strip

// Pin Configuration
const int motorLeftBack =   12;   // Motor pin A1
const int motorLeftFwd =    11;  // Motor pin A2
const int motorRightFwd =   10;  // Motor pin B1
const int motorRightBack =  9;   // Motor pin B2

const int servo = 5; // servo
const int gripper = 6; // gripper GR

const int triggerPin = 2; // HC-SR04 trigger pin
const int echoPin = 4;    // HC-SR04 echo pin

const int motorLeftRead =   8;   // Arduino A0
const int motorRightRead =  7;   // Arduino A1

const int numberOfSensors = 8; // Number of sensors used
int sensorPins[numberOfSensors] = {A0, A1, A2, A3, A4, A5, A6, A7}; // Analog pins for the sensors
int sensorValues[numberOfSensors]; // Array to store the sensor values


const int stopDistance = 10; // Distance threshold to stop the robot (in cm)

int pulseAvgLeft;
int pulseAvgRight;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Set color for a specific Neopixel
void setPixelColor(int pixel, uint8_t red, uint8_t green, uint8_t blue) {
  strip.setPixelColor(pixel, strip.Color(red, green, blue));
  strip.show();
}

void setServoAngle(int angle) {
  // Convert the angle to the corresponding PWM signal
  int pwmValue = map(angle, 0, 180, 544, 2400);

  // Send the PWM signal to the servo
  digitalWrite(servo, HIGH);
  delayMicroseconds(pwmValue);
  digitalWrite(servo, LOW);
}

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
  int openAngle = 90;
  setGripperAngle(openAngle);
}

// Function to close the gripper (move the servo to the closed position)
void closeGripper() {
  // Adjust the angle value based on your servo's specifications
  int closeAngle = 0;
  setGripperAngle(closeAngle);
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

// Sets motor power to input
void setMotors(int LFWD, int LBACK, int RFWD, int RBACK) {
  analogWrite(motorLeftFwd,   LFWD);
  analogWrite(motorLeftBack,  LBACK);
  analogWrite(motorRightFwd,  RFWD);
  analogWrite(motorRightBack, RBACK);
}

void brakeLight() {
  // Set color for each Neopixel individually
  setPixelColor(0, 0, 255, 0);  // Red for the first Neopixel
  setPixelColor(1, 0, 255, 0);  // Red for the second Neopixel
  setPixelColor(2, 0, 0, 0);    // Turn off other Neopixels
  setPixelColor(3, 0, 0, 0);    // Turn off other Neopixels
}

void lightForward() {
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
  setPixelColor(2, 0, 0, 0);    // Turn off other Neopixels
  setPixelColor(3, 80, 255, 0);    // Orange for the 4th Neopixel
}

void rightLight() {
  // Set color for each Neopixel individually
  setPixelColor(0, 0, 0, 0);  // Turn off other Neopixels
  setPixelColor(1, 0, 0, 0);  // Turn off other Neopixels
  setPixelColor(2, 80, 255, 0);    // Orange for the 3th Neopixel
  setPixelColor(3, 0, 0, 0);    // Turn off other Neopixels
}

// Rotate left at 0-255 speed
void driveLeft(int speed) {
  leftLight();
  setMotors(0, speed, speed, 0);
}

// Rotate right at 0-255 speed
void driveRight(int speed) {
  rightLight();
  setMotors(speed, 0 , 0, speed);
}

// Drive forwards at 0-255 speed
void driveForward(int speed) {
  setMotors(speed, 0 , speed, 0);
  lightForward(); // Turn on forward lights
}

// Drive backwards at 0-255 speed
void driveBack(int speed) {
  setMotors(0, speed, 0, speed);
}

// Stop driving
void driveStop(){
  setMotors(0, 0, 0, 0);
  brakeLight(); // Turn on brake lights
}

// Function to close the gripper (move the servo to the closed position)
void lookstraight() {
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
  long duration, distance;
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  // Check for obstacles
  if (distance <= stopDistance) {
    lookstraight();
  } else {
    driveLeft(180);
    lookstraight();
  }
}

// Function to close the gripper (move the servo to the closed position)
void lookRight() {
  // Adjust the angle value based on your servo's specifications
  int angle = 0;
  setServoAngle(angle);

  delay(800);

   // Measure distance
  long duration, distance;
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  // Check for obstacles
  if (distance <= stopDistance) {
    lookLeft();
  } else {
    driveRight(180);
    lookstraight();
  }
}

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
  Serial.begin(9600);
   // Calibrate the servo to a known initial position
  calibrateServo();
}

void loop() {
  // Measure distance
  long duration, distance;
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  // Check for obstacles
  if (distance <= stopDistance) {
    driveStop();
    Serial.println("Obstacle detected");

    // Look left
    lookRight();
    delay(1000);

    // Measure distance again after looking left
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;

    // If there is still an obstacle on the left, look right and drive right
    if (distance <= stopDistance) {
      lookLeft();
      delay(1000);
  } else {
    // Move forward if no obstacle
    driveForward(255);
    Serial.println("No obstacle detected");
  }
  } else {
    // Move forward if no obstacle
      driveForward(255);
      Serial.println("No obstacle detected");
  }
}