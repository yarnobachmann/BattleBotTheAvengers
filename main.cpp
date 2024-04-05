/*╭──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╮
             Include Libraries
  ╰──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╯*/

#include <Arduino.h>

#include <Adafruit_NeoPixel.h>

/*╭──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╮
              Prepare functions
  ╰──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╯*/

// Motor control functions
void setPixelColor(int pixel, uint8_t red, uint8_t green, uint8_t blue);
void setMotors(int LFWD, int LBACK, int RFWD, int RBACK);
void driveLeft(int LFWD, int LBACK, int RFWD, int RBACK);
void driveRight(int LFWD, int LBACK, int RFWD, int RBACK);
void driveForward(int LFWD, int LBACK, int RFWD, int RBACK);
void driveBack(int LFWD, int LBACK, int RFWD, int RBACK);
void driveStop();

// Sensor and input/output functions
void startLightOne();
void startLightTwo();
void startLightThree();
void brakeLight();
void forwardLight();
void leftLight();
void rightLight();
void distanceReader();
void distanceSensor();
void distanceReaderRight();
void distanceSensorRight();
void gripper(int pulse);
void initializeGyro();
void readLineSensor();
void playNote(float frequency, int duration);
void defaultSpeakerValues();
void channel();
boolean isOnLightColor();
void startup();
void detectFinish();
void dropCone();
boolean isStuck();
void incrementPulseLeft();
void incrementPulseRight();
boolean isAnythingBlack();
void playWiiTheme();

/*╭──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╮
                Declare pins
  ╰──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╯*/

#define PIN 13 // Pin connected to Neopixel strip
#define NUMPIXELS 4 // Number of Neopixels in the strip

#define BUZZER A5 // Pin connected to buzzer

#define MOTOR_LEFT_BACKWARD 12 // Pin for left motor backward control
#define MOTOR_LEFT_FORWARD 11 // Pin for left motor forward control
#define MOTOR_RIGHT_FORWARD 10 // Pin for right motor forward control
#define MOTOR_RIGHT_BACKWARD 9 // Pin for right motor backward control
#define MOTOR_RIGHT_READ 2 // Pin for right motor encoder
#define MOTOR_LEFT_READ 3 // Pin for left motor encoder

#define GRIPPER_PIN 6 // Pin for controlling gripper servo
#define GRIPPER_OPEN 1600 // Pulse width to open the gripper
#define GRIPPER_CLOSED 930 // Pulse width to close the gripper

#define SERVO_MIN_PULSE 250 // Minimum pulse width for servo control
#define SERVO_MID_PULSE 1400 // Middle pulse width for servo control
#define SERVO_MAX_PULSE 2400 // Maximum pulse width for servo control

#define TRIGGER_PIN 8 // Pin for ultrasonic sensor trigger
#define ECHO_PIN 4 // Pin for ultrasonic sensor echo
#define TRIGGER_PIN_RIGHT 5 // Pin for right ultrasonic sensor trigger
#define ECHO_PIN_RIGHT 7 // Pin for right ultrasonic sensor echo
long duration, distance; // Variables for ultrasonic sensor readings
long durationRight, distanceRight; // Variables for right ultrasonic sensor 

// Line sensor pins and threshold
#define NUMBER_OF_SENSORS 6 // Number of line sensors
#define SENSOR_INTERVAL 20 // Interval between line sensor readings
#define BLACK_THRESHOLD 900 // Threshold for detecting black line
int sensorPins[NUMBER_OF_SENSORS] = {A1, A6, A7, A3, A2, A0};
int sensorValues[NUMBER_OF_SENSORS]; // Values read from line sensors

// Melody arrays
const float notes[] = {739.99,659.25,783.99,739.99,783.99,783.99,739.99,783.99,659.25,587.33,783.99,739.99,783.99,783.99,739.99,783.99,783.99,739.99,783.99,659.25,
  587.33,698.46,739.99,783.99,659.25,622.25,739.99,783.99,783.99,739.99,783.99,659.25,587.33,739.99,739.99,783.99,739.99,783.99,783.99,739.99,783.99,659.25,587.33,
  783.99,739.99,783.99,783.99,739.99,783.99,783.99,739.99,783.99,659.25,587.33,783.99,739.99,783.99,783.99,739.99,783.99,783.99,739.99,783.99,659.25,587.33,783.99,
  739.99,783.99,783.99,739.99,783.99,783.99,739.99,783.99,659.25,587.33,783.99,739.99,783.99,783.99,739.99,783.99,783.99,739.99,783.99,659.25,587.33,783.99,739.99,
  783.99,783.99,739.99,783.99,783.99,739.99,783.99,659.25,587.33,783.99,739.99,783.99,783.99,739.99,783.99,659.25,587.33,783.99,739.99,783.99,783.99,739.99,783.99,
  783.99,739.99,783.99,659.25,587.33,783.99,739.99,783.99,783.99,739.99,783.99,783.99,739.99,783.99,659.25,587.33,783.99,739.99,783.99,783.99,739.99,783.99,783.99,
  739.99,783.99,659.25,587.33,783.99,739.99,783.99,783.99,739.99,783.99,783.99,739.99,783.99,659.25,587.33,783.99,739.99,783.99,783.99,739.99,783.99,783.99,739.99,
  783.99,659.25,587.33,783.99,739.99,783.99,783.99,739.99,783.99,783.99,739.99
};

// Timing and control variables
#define STOP_DISTANCE 10 // Distance threshold for stopping
#define MIN_RIGHT_DISTANCE 8 // Minimum distance for right sensor
#define MAX_RIGHT_DISTANCE 10 // Maximum distance for right sensor
bool HAS_START_ENDED = false; // Flag indicating startup completion
bool HAS_END_STARTED = false; // Flag indicating startup completion
#define START_TRIGGER false // Flag indicating startup trigger
bool CONTINUE_RIGHT_SENSOR = true; // Flag for continuing right sensor readings
#define LIGHT_VALUE 700 // Threshold for detecting light color
#define TIME_TO_START_DETECTING_FINISH 10000 // Time to start detecting finish line
long PULSES_LEFT = 0; // Counter for left wheel pulses
long PULSES_RIGHT = 0; // Counter for right wheel pulses
long stuckTimer = 0; // Timer for detecting robot stuck state
static unsigned long timeToStartDetectingFinish; // Time to start detecting the finish line
bool IS_GRIPPER_OPEN = false; // Flag indicating gripper state

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_RGB + NEO_KHZ800);

/*╭──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╮
               Setup functions
  ╰──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╯*/

// Setup function to initialize the robot's hardware and peripherals
void setup() {
  strip.begin(); // Initialize NeoPixel strip
  strip.show(); // Turn off all NeoPixels initially
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT); // Set left motor forward pin as output
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT); // Set left motor backward pin as output
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT); // Set right motor backward pin as output
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT); // Set right motor forward pin as output
  attachInterrupt(digitalPinToInterrupt(MOTOR_RIGHT_READ), incrementPulseRight, CHANGE); // Attach interrupt for right motor encoder pulse counting
  attachInterrupt(digitalPinToInterrupt(MOTOR_LEFT_READ), incrementPulseLeft, CHANGE); // Attach interrupt for left motor encoder pulse counting
  pinMode(TRIGGER_PIN, OUTPUT); // Set trigger pin for front distance sensor as output
  pinMode(ECHO_PIN, INPUT); // Set echo pin for front distance sensor as input
  pinMode(TRIGGER_PIN_RIGHT, OUTPUT); // Set trigger pin for right distance sensor as output
  pinMode(ECHO_PIN_RIGHT, INPUT); // Set echo pin for right distance sensor as input
  pinMode(GRIPPER_PIN, OUTPUT); // Set gripper control pin as output
  digitalWrite(GRIPPER_PIN, LOW); // Initialize gripper pin to low (closed position)
  pinMode(BUZZER, OUTPUT); // Set pin for buzzer as output

  // Initialize line sensor pins as inputs
  for (int i = 0; i < NUMBER_OF_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  Serial.begin(9600); // Initialize serial communication for debugging
}

/*╭──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╮
               Loop function
  ╰──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╯*/

// Main loop of the program
void loop() {
  // Check if startup sequence has ended
  if (!HAS_START_ENDED) {
    startup(); // Execute startup sequence
  }

  // Read distance sensor for obstacle detection in front
  distanceSensor(); // Check distance sensor in front

  // Continue reading distance sensor for obstacle detection on the right
  if (CONTINUE_RIGHT_SENSOR) {
    distanceSensorRight(); // Check distance sensor on the right side
  }

  // Check if the robot is stuck
  if (isStuck()) {
    driveBack(0, 255, 0, 255); // Move backward to get unstuck
    delay(500); // Delay to ensure movement
    stuckTimer = millis() + 500; // Set timer for stuck condition
  }

  // Start detecting finish line after a certain time
  if (TIME_TO_START_DETECTING_FINISH < millis()) {
    detectFinish(); // Execute finish line detection
  }

  // Set timer to trigger gripper action periodically
  static unsigned long timer;
  if (millis() > timer) {
    // Toggle gripper state to open/close
    if (IS_GRIPPER_OPEN) {
      gripper(GRIPPER_OPEN); // Open gripper if closed
    } else {
      gripper(GRIPPER_CLOSED); // Close gripper if open
    }
    timer = millis() + 500; // Set next gripper action time
  }
}

/*╭──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╮
                Start / End
  ╰──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╯*/

// Function to initialize the robot and start the competition
void startup() {
  int linesPassed = 0;
  boolean currentColor = false;
  boolean hasGotPion = false;

  // Open gripper for initial 100 milliseconds
  for (int i = 0; i < 100; i++) {
    delay(10);
    gripper(GRIPPER_OPEN);
    startLightOne(); // Activate light function
  }

  // Move forward until obstacle is detected and stop at a safe distance
  for (int i = 0; i < 3; i++) {
    do {
      digitalWrite(TRIGGER_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIGGER_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIGGER_PIN, LOW);

      long duration = pulseIn(ECHO_PIN, HIGH);
      distance = duration * 0.034 / 2;
    } while (distance > 24); // Repeat until distance is less than 24 cm
  }

  // Check if there is an echo before proceeding with startup sequence
  if (distance > 0) {
    // Proceed with startup sequence
    // Drive forward until a certain number of lines are detected
    startLightTwo(); // Activate light function
    delay(850); // Delay for stability
    startLightThree(); // Activate light function
    delay(850); // Delay for stabiliy
    tone(BUZZER, 600, 300); // Emit a warning tone
    delay(300);
    driveForward(220, 0, 230, 0); // Move forward at specified speed
    while (linesPassed <= 6) {
      boolean detectedColor = isOnLightColor(); // Check if line is detected
      if (currentColor != detectedColor) { // If color changes
        currentColor = detectedColor; // Update current color
        linesPassed++; // Increment lines passed
      }
    }

    // Move forward until the robot grasps the pylon
    while (true) {
      if (!isOnLightColor()) { // If not on light color
        gripper(GRIPPER_CLOSED); // Close gripper to grasp pylon
        hasGotPion = true; // Update status to indicate pylon is grasped
      }
      if (hasGotPion) { // If pylon is grasped
        if (isOnLightColor()) { // If back on light color
          driveLeft(0, 255, 255, 0); // Turn left to avoid obstacle
          delay(400); // Delay for turn to complete
          long timer = millis() + 2500; // Set timer for forward movement
          while (timer > millis()) {
            readLineSensor(); // Read line sensors while moving forward
          }
          driveForward(180, 0, 180, 0); // Move forward at reduced speed
          HAS_START_ENDED = true; // Update status to indicate startup is complete
          break; // Exit loop
        }
      }
    }

    timeToStartDetectingFinish = millis() + 15000; // Set time to start detecting finish line
  } else {
    // Echo not detected, take appropriate action (e.g., error handling)
    // For simplicity, let's just stop here and indicate an error
    while (true) {
      // Indicate error state, for example, by blinking lights or emitting a sound
      tone(BUZZER, 1000, 500);
    }
  }
}

// Function to drop the cone after completing the competition
void dropCone() {
  gripper(GRIPPER_OPEN); // Open gripper to release cone
  unsigned long timer = millis() + 1000; // Set timer for cone release
  while (timer > millis()) { // Repeat until timer expires
    gripper(GRIPPER_OPEN); // Open gripper continuously to release cone
    driveBack(0, 255, 0, 255); // Move backward while releasing cone
  }
  driveStop(); // Stop robot movement
  playWiiTheme(); // Play victory melody
  delay(2000); // Delay for melody to complete
  while (true) { // Loop indefinitely
  }
}

// Function to detect the finish line and trigger cone dropping
void detectFinish() {
  if (isAnythingBlack()) { // If any sensor detects black
    if (isOnLightColor()) { // If on light color
      while (true) { // Loop indefinitely
        bool allBlack = true;
        for (int i = 0; i < NUMBER_OF_SENSORS; i++) { // Iterate through all sensors
          if (sensorValues[i] < BLACK_THRESHOLD) { // If any sensor value is less than black threshold
            allBlack = false; // Update flag to indicate not all sensors detect black
            break; // Exit loop
          }
        }
        if (allBlack) { // If all sensors detect black
          dropCone(); // Trigger cone dropping
          break; // Exit loop
        } else {
          // Check if the robot is stuck
          HAS_END_STARTED = true;
          if (isStuck()) {
            driveBack(0, 255, 0, 255); // Move backward to get unstuck
            delay(500); // Delay to ensure movement
            stuckTimer = millis() + 500; // Set timer for stuck condition
          }
          readLineSensor(); // Read line sensors while moving forward
        }
      }
    }
  }
}

/*╭──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╮
                  NeoPixel
  ╰──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╯*/

// Function to set color for a specific Neopixel
void setPixelColor(int pixel, uint8_t red, uint8_t green, uint8_t blue) {
  strip.setPixelColor(pixel, strip.Color(red, green, blue)); // Set color of specified Neopixel
  strip.show(); // Update Neopixel display
}

/*╭──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╮
                  Gripper
  ╰──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╯*/

// Function to control the gripper servo based on pulse width
void gripper(int pulse) {
  static int pulse1;
  if (pulse == GRIPPER_OPEN) { // If open command received
    IS_GRIPPER_OPEN = true; // Set gripper status to open
  } else if (pulse == GRIPPER_CLOSED) { // If close command received
    IS_GRIPPER_OPEN = false; // Set gripper status to closed
  }

  if (pulse > 0) {
    pulse1 = pulse; // Set pulse width
  }
  digitalWrite(GRIPPER_PIN, HIGH); // Activate gripper servo
  delayMicroseconds(pulse1); // Apply pulse width
  digitalWrite(GRIPPER_PIN, LOW); // Deactivate gripper servo
}

/*╭──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╮
                    Motor
  ╰──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╯*/

void setMotors(int LFWD, int LBACK, int RFWD, int RBACK) {
  // Control motors with specified power levels with constrain
  analogWrite(MOTOR_LEFT_FORWARD, constrain(LFWD, 0, 255));
  analogWrite(MOTOR_LEFT_BACKWARD, constrain(LBACK, 0, 255));
  analogWrite(MOTOR_RIGHT_FORWARD, constrain(RFWD, 0, 255));
  analogWrite(MOTOR_RIGHT_BACKWARD, constrain(RBACK, 0, 255));
}

/*╭──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╮
              Drive functions
  ╰──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╯*/

void driveLeft(int LFWD, int LBACK, int RFWD, int RBACK) {
  // Control left wheel movement and turn on left light
  leftLight();
  setMotors(LFWD, LBACK, RFWD, RBACK);
}

void driveRight(int LFWD, int LBACK, int RFWD, int RBACK) {
  // Control right wheel movement and turn on right light
  rightLight();
  setMotors(LFWD, LBACK, RFWD, RBACK);
}

void driveForward(int LFWD, int LBACK, int RFWD, int RBACK) {
  // Control forward movement and turn on forward light
  forwardLight();
  setMotors(LFWD, LBACK, RFWD, RBACK);
}

void driveBack(int LFWD, int LBACK, int RFWD, int RBACK) {
  // Control backward movement and turn on backward light
  brakeLight();
  setMotors(LFWD, LBACK, RFWD, RBACK);
}

void driveStop() {
  // Stop robot movement and turn on backward light
  brakeLight();
  setMotors(0, 0, 0, 0);
}

/*╭──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╮
              Light functions
  ╰──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╯*/

void brakeLight() {
  // Set color for each Neopixel individually
  setPixelColor(0, 255, 0, 0); // Red for the first Neopixel
  setPixelColor(1, 255, 0, 0); // Red for the second Neopixel
  setPixelColor(2, 0, 0, 0); // Turn off other Neopixels
  setPixelColor(3, 0, 0, 0); // Turn off other Neopixels
}

void forwardLight() {
  // Set color for each Neopixel individually
  setPixelColor(0, 0, 0, 0); // Turn off other Neopixels
  setPixelColor(1, 0, 0, 0); // Turn off other Neopixels
  setPixelColor(2, 0, 255, 0); // green for the 3th Neopixel
  setPixelColor(3, 0, 255, 0); // green for the 4th Neopixel
}

void leftLight() {
  // Set color for each Neopixel individually
  setPixelColor(0, 0, 0, 0); // Turn off other Neopixels
  setPixelColor(1, 0, 0, 0); // Turn off other Neopixels
  setPixelColor(2, 0, 255, 0); // green for the 3th Neopixel
  setPixelColor(3, 255, 80, 0); // Orange for the 4th Neopixel
}

void rightLight() {
  // Set color for each Neopixel individually
  setPixelColor(0, 0, 0, 0); // Turn off other Neopixels
  setPixelColor(1, 0, 0, 0); // Turn off other Neopixels
  setPixelColor(2, 255, 80, 0); // Orange for the 3th Neopixel
  setPixelColor(3, 0, 255, 0); // green for the 4th Neopixel
}

void startLightOne() {
  // Set color for each Neopixel individually
  setPixelColor(0, 255, 0, 0); // Red for the first Neopixel
  setPixelColor(1, 255, 0, 0); // Red for the second Neopixel
  setPixelColor(2, 255, 0, 0); // Red off other Neopixels
  setPixelColor(3, 255, 0, 0); // Red off other Neopixels
}

void startLightTwo() {
  tone(BUZZER, 5000, 100); // Emit a warning tone
  // Set color for each Neopixel individually
  setPixelColor(0, 255, 80, 0); // Orange for the first Neopixel
  setPixelColor(1, 255, 80, 0); // Orange for the second Neopixel
  setPixelColor(2, 255, 80, 0); // Orange for the third Neopixel
  setPixelColor(3, 255, 80, 0); // Orange for the fourth Neopixel
}

void startLightThree() {
  tone(BUZZER, 5000, 100); // Emit a warning tone
  // Set color for each Neopixel individually
  setPixelColor(0, 0, 255, 0); // Green for the first Neopixel
  setPixelColor(1, 0, 255, 0); // Green for the second Neopixel
  setPixelColor(2, 0, 255, 0); // Green for the third Neopixel
  setPixelColor(3, 0, 255, 0); // Green for the fourth Neopixel
}

/*╭──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╮
                 Echo sensor
  ╰──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╯*/

// Function to read distance from ultrasonic sensor
void distanceReader() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH); // Measure the duration of the pulse
  distance = (duration / 2) * 0.0343; // Convert duration to distance in centimeters
}

// Function to control behavior based on front distance
void distanceSensor() {
  static unsigned long timer;
  int interval = 500; // Read distance every 500 milliseconds
  if (millis() > timer) {
    distanceReader(); // Read distance from ultrasonic sensor
    if (distance <= STOP_DISTANCE) { // If distance is less than stop threshold
      CONTINUE_RIGHT_SENSOR = false; // Stop reading from right sensor
      tone(BUZZER, 1000, 100); // Emit a warning tone
      driveBack(0, 255, 0, 255); // Move backward to get unstuck
      delay(100); // Delay to ensure movement
      driveLeft(0, 210, 230, 0); // Turn left to avoid obstacle
      delay(500); // Delay to allow turn to complete
      CONTINUE_RIGHT_SENSOR = true; // Resume reading from right sensor
    }
    timer = millis() + interval; // Set next reading time
  }
}

// Function to read distance from right ultrasonic sensor
void distanceReaderRight() {
  digitalWrite(TRIGGER_PIN_RIGHT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN_RIGHT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_RIGHT, LOW);

  durationRight = pulseIn(ECHO_PIN_RIGHT, HIGH); // Measure the duration of the pulse
  distanceRight = (durationRight / 2) * 0.0343; // Convert duration to distance in centimeters
}

// Function to control behavior based on right distance
void distanceSensorRight() {
  distanceReaderRight(); // Read distance from right ultrasonic sensor

  static unsigned long timer;
  int interval = 50; // Read distance every 50 milliseconds
  if (millis() > timer) {

    int difference = distanceRight - MIN_RIGHT_DISTANCE;

    if (distanceRight >= MIN_RIGHT_DISTANCE && difference <= 10) { // If distance is within safe range
      driveRight(230, 0, 130, 0); // Adjust direction to the right
    } else if (difference >= 8) // If distance is too close to the right
    {
      driveForward(230, 0, 230, 0); // Move forward
      delay(80); // Delay to allow forward movement
      driveRight(180, 0, 0, 230); // Turn right
      delay(40); // Delay to allow turn to complete
    } else if (distanceRight <= MAX_RIGHT_DISTANCE && difference <= 10) // If distance is too far from the right
    {
      driveLeft(130, 0, 230, 0); // Adjust direction to the left
    } else // If distance is within acceptable range
    {
      driveForward(230, 0, 230, 0); // Move forward
    }
    timer = millis() + interval; // Set next reading time
  }
}

/*╭──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╮
                Line sensor
  ╰──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╯*/

// Function to read inputs from line sensors and control robot movement accordingly
void readLineSensor() {
  static unsigned long timer;
  static unsigned long allBlackTimerStart = 0;
  static bool gripperClosed = false;

  if (millis() > timer) {
    for (int i = 0; i < NUMBER_OF_SENSORS; i++) {
      sensorValues[i] = analogRead(sensorPins[i]); // Read analog values from line sensors
    }

    bool allBlack = true;
    for (int i = 0; i < NUMBER_OF_SENSORS; i++) {
      if (sensorValues[i] < BLACK_THRESHOLD) {
        allBlack = false;
        break;
      }
    }

    if (allBlack) { // If all sensors detect black
      allBlackTimerStart += SENSOR_INTERVAL; // Increment timer
    }

    if (allBlackTimerStart >= 1000) { // If all sensors detect black for 1 second
      gripper(GRIPPER_CLOSED); // Close gripper
      gripperClosed = true;
    }

    if (sensorValues[2] >= BLACK_THRESHOLD || sensorValues[3] >= BLACK_THRESHOLD || sensorValues[4] >= BLACK_THRESHOLD || sensorValues[1] >= BLACK_THRESHOLD) {
      driveForward(200, 0, 200, 0); // Drive forward if any of the middle sensors detect black
    } else if (sensorValues[5] >= BLACK_THRESHOLD) {
      driveRight(255, 0, 0, 0); // Turn right if rightmost sensor detects black
    } else if (sensorValues[0] >= BLACK_THRESHOLD) {
      driveLeft(0, 0, 255, 0); // Turn left if leftmost sensor detects black
    }

    timer = millis() + SENSOR_INTERVAL; // Set next reading time
  }
}

// Function to check if the robot is on a light-colored surface
boolean isOnLightColor() {
  int averageColor = 0;
  for (int sensorPin: sensorValues) {
    averageColor += analogRead(sensorPin); // Read analog values from line sensors
  }

  int lightColor = (averageColor / 6) <= BLACK_THRESHOLD; // Calculate average color and compare to black threshold

  return lightColor; // Return true if on light color, false otherwise
}

// Function to check if any sensor detects black
boolean isAnythingBlack() {
  for (int pin: sensorValues) {
    if (analogRead(pin) > 800) { // Check if sensor value exceeds threshold for black
      return true; // Return true if any sensor detects black
    }
  }
  return false; // Return false if no sensor detects black
}

/*╭──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╮
                Pulse sensor
  ╰──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╯*/

void incrementPulseLeft()
// Increment pulse count for left wheel
{
  PULSES_LEFT++;
}

void incrementPulseRight()
// Increment pulse count for right wheel
{
  PULSES_RIGHT++;
}

// Function to check if the robot is stuck
boolean isStuck() {
  static long previousPulsesL = PULSES_LEFT;
  static long previousPulsesR = PULSES_RIGHT;
  static long timer = millis();
  static int amountOfFailedPulsesL;
  static int amountOfFailedPulsesR;

  if (timer <= millis()) {
    if ((previousPulsesL + 3) < PULSES_LEFT) { // If left wheel pulses increase
      amountOfFailedPulsesL = 0; // Reset failed pulse count
      previousPulsesL = PULSES_LEFT; // Update previous pulse count
    } else {
      amountOfFailedPulsesL++; // Increment failed pulse count
    }
    if ((previousPulsesR + 3) < PULSES_RIGHT) { // If right wheel pulses increase
      amountOfFailedPulsesR = 0; // Reset failed pulse count
      previousPulsesR = PULSES_RIGHT; // Update previous pulse count
    } else {
      amountOfFailedPulsesR++; // Increment failed pulse count
    }
    if (amountOfFailedPulsesL >= 5) { // If left wheel has been stuck for too long
      return true; // Return true to indicate the robot is stuck
    }
    if (amountOfFailedPulsesR >= 5) { // If right wheel has been stuck for too long
      return true; // Return true to indicate the robot is stuck
    }
    timer = millis() + 100; // Set next check time
  }
  return false; // Return false to indicate the robot is not stuck
}

/*╭──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╮
                   Music
  ╰──────────── ⋆˖⁺‧₊☽◯☾₊‧⁺˖⋆ ────────────╯*/

// Function to play the Wii theme melody
void playWiiTheme() {
  for (int i = 0; i < sizeof(notes) / sizeof(notes[0]); i++) {
    if (notes[i] == 0) {
      delay(500); // Delay if rest note
    } else {
      tone(BUZZER, notes[i], 500); // Play note
      delay(500); // Delay note duration
    }
    noTone(BUZZER); // Stop playing note
  }
}