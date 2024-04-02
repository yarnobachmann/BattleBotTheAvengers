//-----------------------------------[Include libraries]----------------------------

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

//-----------------------------------[Predeclare functions]-------------------------

void setPixelColor(int pixel, uint8_t red, uint8_t green, uint8_t blue);
void setMotors(int LFWD, int LBACK, int RFWD, int RBACK);
void brakeLight();
void forwardLight();
void leftLight();
void rightLight();
void driveLeft(int speed);
void driveRight(int speed);
void driveForward(int speed);
void driveBack(int speed);
void driveStop();
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
boolean isAnythingBlack();
void playWiiTheme();

//-----------------------------------[Declare pins]-------------------------------

#define PIN 13 // Digital pin connected to the Neopixel
#define NUMPIXELS 4 // Number of Neopixels in your strip

#define BUZZER A5 // Buzzer pin

#define MOTOR_LEFT_BACKWARD 12 // Motor pin A1
#define MOTOR_LEFT_FORWARD 11 // Motor pin A2
#define MOTOR_RIGHT_FORWARD 10 // Motor pin B1
#define MOTOR_RIGHT_BACKWARD 9 // Motor pin B2
#define MOTOR_RIGHT_READ 2 // Motor pin B1
#define MOTOR_LEFT_READ 3// Motor pin B2

#define GRIPPER_PIN 6 // gripper GR
#define GRIPPER_OPEN 1600 // pulse length servo open
#define GRIPPER_CLOSED 930 // pulse length servo closed
#define GRIPPER_INTERVAL 20 // time between pulse
#define GRIPPER_TOGGLE 1000 // toggle gripper every second

#define SERVO_MIN_PULSE 250 // pulse length for -90 degrees
#define SERVO_MID_PULSE 1400 // pulse length for 0 degrees
#define SERVO_MAX_PULSE 2400 // pulse length for 90 degrees

#define TRIGGER_PIN 8 // HC-SR04 trigger pin
#define ECHO_PIN 4 // HC-SR04 echo pin
#define TRIGGER_PIN_RIGHT 5 // HC-SR04 trigger pin
#define ECHO_PIN_RIGHT 7 // HC-SR04 echo pin

const int numberOfSensors = 6; // Number of sensors used
int sensorPins[numberOfSensors] = {
  A1,A6,A7,A3,A2,A0
}; // Analog pins for the sensors      
int sensorValues[numberOfSensors]; // Array to store the sensor values
#define SENSOR_INTERVAL 20 // time between pulse

const int BLACK = 900;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_RGB + NEO_KHZ800);

//-----------------------------------[Variables]---------------------------------

const float notes[] = {659.25, 622.25, 659.25, 0, 659.25, 0, 659.25, 0, 523.25, 0, 0, 493.88, 0, 0, 523.25, 0, 0, 392, 0, 0, 466.16, 0, 0, 440, 0, 0, 0, 0, 0, 466.16, 0, 0, 392, 0, 0, 311.13, 0, 0, 369.99, 0, 0, 311.13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 311.13, 0, 0, 369.99, 0, 0, 392, 0, 0, 466.16, 0, 0, 392, 0, 0, 311.13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const int durations[] = {250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250};

unsigned long startTime; // Variable to store the start time for calibration
const int calibrationDuration = 5000; // Calibration duration in milliseconds
bool calibrated = false; // Flag to check if calibration is done

const int stopDistance = 10; // Distance threshold to stop the robot (in cm)
const int minRightDistance = 8; // Distance threshold to stop the robot (in cm)
const int maxRightDistance = 10; // Distance threshold to stop the robot (in cm)

int rightOffset = 100; // RightOffset default value set to 100
int leftOffset = 100; // LeftOffset default value set to 100

bool hasStartEnded = false;
bool startTrigger = false;
bool continueRightSensor = true;

int LIGHT_VALUE = 700; // Adjust this threshold based on your sensor's characteristics

long duration, distance; // Variables to store the duration and distance for the HC-SR04 sensor'
long durationRight, distanceRight; // Variables to store the duration and distance for the HC-SR04 sensor'

static unsigned long timeToStartDetectingFinish;

long pulsesLeft                 = 0;
long pulsesRight                = 0;
long stuckTimer                 = 0;
bool isGripperOpen = false;


//-----------------------------------[Setup function]---------------------------

void setup() {
  strip.begin(); // Initialize the Neopixel strip
  strip.show(); // Initialize all pixels to 'off'
  // Motor pins
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(MOTOR_LEFT_READ), incrementPulseLeft, CHANGE);
  // Echo sensor pins
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIGGER_PIN_RIGHT, OUTPUT);
  pinMode(ECHO_PIN_RIGHT, INPUT);
  // Servo pins
  pinMode(GRIPPER_PIN, OUTPUT);
  digitalWrite(GRIPPER_PIN, LOW);
  pinMode(BUZZER, OUTPUT);
  // Reflection sensor pins
  for (int i = 0; i <= numberOfSensors; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  Serial.begin(9600);
}

//-----------------------------------[Loop function]----------------------------

void loop() {
  if (!hasStartEnded)
  {
    startup(); 
  }

  distanceSensor();  

  if (continueRightSensor)
  {
    distanceSensorRight();
  }

  if (isStuck())
  {
    driveBack(255);
    delay(800);
    stuckTimer = millis() + 500;
  }

  if (timeToStartDetectingFinish < millis())
  {
    detectFinish();
  }

  if (!startTrigger)
  {
    startTrigger = true;
  } 
  
  static unsigned long timer;

  if (millis() > timer) {

    if (isGripperOpen)
    {
      gripper(GRIPPER_OPEN);
    }
    else
    {
      gripper(GRIPPER_CLOSED);
    }
    
    timer = millis() + 500;
  }
}

//-----------------------------------[Start&End]---------------------------------

void startup()
{
  int linesPassed = 0;
  boolean currentColor = false;
  boolean hasGotPion = false;

  for (int i = 0; i < 100; i++)
    {
        delay(10);
        gripper(GRIPPER_OPEN);
    }

    for (int i = 0; i < 3; i++)
  {
    do
    {
      digitalWrite(TRIGGER_PIN, LOW); // Reset pin
      delayMicroseconds(2);
      digitalWrite(TRIGGER_PIN, HIGH); // High pulses for 10 ms
      delayMicroseconds(10);
      digitalWrite(TRIGGER_PIN, LOW);
    
      long duration = pulseIn(ECHO_PIN, HIGH);
      distance = duration * 0.034 / 2; // Bereken de afstand in centimeters
    }  
      while(distance > 24);
  }
  delay(500);
  driveForward(220);
  //Count the lines it passes whilst driving forward
  //When it reaches 7 switches, go to the next phase
  while(linesPassed <= 6)
  {
    boolean detectedColor = isOnLightColor();
    if (currentColor != detectedColor)
    {
      Serial.println("Detected line");
      currentColor = detectedColor;
      linesPassed++;
    }
  }
  Serial.println("Waiting for the black square");
  //Loop untill it's done with the startup sequence
  while(true)
  {
    //Close the gripper once a black line is detected
    if (!isOnLightColor())
    {
      gripper(GRIPPER_CLOSED);
      hasGotPion = true;
    }
    //If it has the Pion, drive forward untill it reached the end of the black square
    if (hasGotPion)
    {
      if (isOnLightColor())
      {
        //Stuff to follow the line for a small bit
        driveLeft(255);
        delay(300);
        long timer = millis() + 2000;
        while(timer > millis())
        {
          readLineSensor();
        }
        driveForward(180);
        hasStartEnded = true;
        break;
      }
    }
  }
  timeToStartDetectingFinish = millis() + 10000;
}

void detectFinish()
{
  if(isAnythingBlack())
  {
    if (isOnLightColor())
    {
     while(true)
      {
        bool allBlack = true;
        for (int i = 0; i < numberOfSensors; i++) {
          if (sensorValues[i] < BLACK) {
            allBlack = false;
            break;
          }
        }
        if (allBlack)
        {
          dropCone();
          break;
        }
        else
        {
          readLineSensor();
        }  
      }  
    }
  }
}

void dropCone()
{
  gripper(GRIPPER_OPEN);
  unsigned long timer = millis() + 1000; 
  while(timer > millis())
  {
    gripper(GRIPPER_OPEN);
    driveBack(255);
  }
  driveStop();
  // Call the function to play the Wii theme
  playWiiTheme();
  // Add a delay before replaying the theme
  delay(2000);
  while(true)
  {
  }
}

//-----------------------------------[Neopixel]-------------------------------------

// Set color for a specific Neopixel
void setPixelColor(int pixel, uint8_t red, uint8_t green, uint8_t blue) {
  strip.setPixelColor(pixel, strip.Color(red, green, blue));
  strip.show();
}

//-----------------------------------[Gripper servo]--------------------------------

void gripper(int pulse) {
  static int pulse1;
  if (pulse == GRIPPER_OPEN)
  {
    isGripperOpen = true;
  }
  else if (pulse == GRIPPER_CLOSED)
  {
    isGripperOpen = false;
  }
  
  if (pulse > 0) {
    pulse1 = pulse;
  }
  
    digitalWrite(GRIPPER_PIN, HIGH);
    delayMicroseconds(pulse1);
    digitalWrite(GRIPPER_PIN, LOW);
  
}

//-----------------------------------[Wheels motor]---------------------------

// Sets motor power to input
void setMotors(int LFWD, int LBACK, int RFWD, int RBACK) {
  analogWrite(MOTOR_LEFT_FORWARD, constrain(LFWD, 0, 255));
  analogWrite(MOTOR_LEFT_BACKWARD, constrain(LBACK, 0, 255));
  analogWrite(MOTOR_RIGHT_FORWARD, constrain(RFWD, 0, 255));
  analogWrite(MOTOR_RIGHT_BACKWARD, constrain(RBACK, 0, 255));
}

//-----------------------------------[Drive functions]------------------------

void driveLeft(int speed) {
  leftLight();
  setMotors(0, speed, speed, 0);
}

void driveRight(int speed) {
  setMotors(speed, 0, 0, speed);
  rightLight();
}

void driveForward(int speed) {
  setMotors(speed, 0, speed, 0);
  forwardLight();
}

void driveBack(int speed) {
  setMotors(0, speed, 0, speed);
  brakeLight();
}

void driveStop() {
  setMotors(0, 0, 0, 0);
  brakeLight();
}

//-----------------------------------[Blinking lights]------------------------

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

//-----------------------------------[Echo sensor]------------------------

void distanceReader() {
  digitalWrite(TRIGGER_PIN, LOW); // Reset pin
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH); // High pulses for 10 ms
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH); // Reads pins
  distance = (duration / 2) * 0.0343; // 343 m/s per second as speed of sound
}

void distanceSensor() {
  int leftSpeed = 230;
  int rightSpeed = 230;

  static unsigned long timer;
  int interval = 500;
  if (millis() > timer) {
    distanceReader();
    if (distance <= stopDistance) {
      continueRightSensor = false;
      tone(BUZZER, 1000, 100); 
      setMotors(0, leftSpeed - 20, rightSpeed, 0);
      delay(500);
      continueRightSensor = true;
    } 
    timer = millis() + interval;
  }
}

void distanceReaderRight() {
  digitalWrite(TRIGGER_PIN_RIGHT, LOW); // Reset pin
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN_RIGHT, HIGH); // High pulses for 10 ms
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_RIGHT, LOW);

  durationRight = pulseIn(ECHO_PIN_RIGHT, HIGH); // Reads pins
  distanceRight = (durationRight / 2) * 0.0343; // 343 m/s per second as speed of sound
}

void distanceSensorRight() {
  int leftSpeed = 230;
  int rightSpeed = 230;

  // Limit the motor speeds to the valid range (0-255)
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  distanceReaderRight();
  
  static unsigned long timer;
  int interval = 50;
  if (millis() > timer) {

  int difference = distanceRight - minRightDistance;

  if (distanceRight >= minRightDistance && difference <= 10) {
    setMotors(leftSpeed, 0, rightSpeed - 100, 0);
  } 
  else if (difference >= 8)
  {
    setMotors(leftSpeed, 0, rightSpeed, 0);
    delay(80);
    setMotors(leftSpeed -50, 0, 0, rightSpeed);
    delay(40);
  }
  else if (distanceRight <= maxRightDistance && difference <= 10) 
  {
    setMotors(leftSpeed - 100, 0, rightSpeed, 0);
  } 
  else 
  {
    // Drive the robot with the adjusted motor speeds
    setMotors(leftSpeed, 0, rightSpeed, 0);
  }
  timer = millis() + interval;
  }
}

//-----------------------------------[Line sensor]------------------------

void readLineSensor() {
  static unsigned long timer;
  static unsigned long allBlackTimerStart = 0;
  static bool gripperClosed = false;

  if (millis() > timer) {
    // Read sensor values
    for (int i = 0; i < numberOfSensors; i++) {
      sensorValues[i] = analogRead(sensorPins[i]);
    }

    // Check if all sensors are black
    bool allBlack = true;
    for (int i = 0; i < numberOfSensors; i++) {
      if (sensorValues[i] < BLACK) {
        allBlack = false;
        break;
      }
    }

    if (allBlack) {
      // Increment the timer if all sensors are black
      allBlackTimerStart += SENSOR_INTERVAL;
    }

    if (allBlackTimerStart >= 1000) {
      // If all sensors have been black for more than 5 seconds and gripper is not already closed, close the gripper
      gripper(GRIPPER_CLOSED);
      gripperClosed = true;
    }

    if (sensorValues[2] >= BLACK || sensorValues[3] >= BLACK || sensorValues[4] >= BLACK || sensorValues[1] >= BLACK) {
      driveForward(200); //We start with slowest and then modify the speed to a decent speed
      Serial.print("forward");
      Serial.print(" ");
    } else if (sensorValues[5] >= BLACK) {
      driveRight(255);
      Serial.print("right222");
      Serial.print(" ");
    } else if (sensorValues[0] >= BLACK) {
      driveLeft(255);
      Serial.print("left222");
      Serial.print(" ");
    }
    timer = millis() + SENSOR_INTERVAL;
  }
}

boolean isOnLightColor()
{
  int averageColor = 0;
  for (int sensorPin : sensorValues)
  {
    averageColor += analogRead(sensorPin);
  }
  
  int lightColor = (averageColor / 6) <= BLACK;

  return lightColor;
}

boolean isAnythingBlack()
{
  for (int pin : sensorValues)
  {
    if(analogRead(pin) > 800)
    {
      return true;  
    }
  }  
  return false;
}

//-----------------------------------[Pulse sensor]------------------------

void incrementPulseLeft()
{
  pulsesLeft++;
}

boolean isStuck()
{
  static long previousPulses = pulsesLeft;
  static long timer = millis();
  static int amountOfFailedPulses;

  if(timer <= millis())
  {
    if ((previousPulses + 3) < pulsesLeft)
    {
      amountOfFailedPulses = 0;
      previousPulses = pulsesLeft;
    }
    else
    {
      amountOfFailedPulses++;
    }
    if (amountOfFailedPulses >= 20)
    {
      return true;
    } 
    timer = millis() + 200;
  }
  return false;
}

void playWiiTheme() {
  // Loop through each note and play it with its duration
  for (int i = 0; i < sizeof(notes) / sizeof(notes[0]); i++) {
    if (notes[i] == 0) {
      // Pause if the note is 0
      delay(durations[i]);
    } else {
      // Play the note with its duration
      tone(BUZZER, notes[i], durations[i]);
      delay(durations[i]);
    }
    // Silence the buzzer after each note to prevent overlapping sounds
    noTone(BUZZER);
  }
}