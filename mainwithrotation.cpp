//-----------------------------------[Include libraries]----------------------------

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_LSM6DS3TRC.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

//-----------------------------------[Predeclare functions]-------------------------

void setPixelColor(int pixel, uint8_t red, uint8_t green, uint8_t blue);
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
void distanceReader();
void distanceSensor();
void calibrateToDriveStraight();
void leftPulseInterrupt();
void rightPulseInterrupt();
void gripperToggle();
void gripper(int pulse); 
void servo(int pulse); 
void updateRotation();
void turnToAngle(double wantedRotation);
bool isRightTurnFaster(double wantedRotation);
double wrapAngle(int rotationToWrap);
double radiansToDegrees(double radians);


//-----------------------------------[Declare pins]-------------------------------

#define PIN 13  // Digital pin connected to the Neopixel
#define NUMPIXELS 4 // Number of Neopixels in your strip

#define MOTOR_LEFT_BACKWARD 12  // Motor pin A1
#define MOTOR_LEFT_FORWARD  11  // Motor pin A2
#define MOTOR_RIGHT_FORWARD 10  // Motor pin B1
#define MOTOR_RIGHT_BACKWARD 9  // Motor pin B2
#define MOTOR_LEFT_READ 2   // Arduino A0
#define MOTOR_RIGHT_READ 3  // Arduino A1

#define GRIPPER_PIN 6 // gripper GR
#define GRIPPER_OPEN 1600 // pulse length servo open
#define GRIPPER_CLOSED  930 // pulse length servo closed
#define GRIPPER_INTERVAL  20  // time between pulse
#define GRIPPER_TOGGLE  1000  // toggle gripper every second

#define SERVO_INTERVAL  20  // time between pulse
#define SERVO_MIN_PULSE  250  // pulse length for -90 degrees
#define SERVO_MID_PULSE  1400 // pulse length for 0 degrees
#define SERVO_MAX_PULSE  2400 // pulse length for 90 degrees
#define SERVO_PIN 5 // servo
#define TRIGGER_PIN 8 // HC-SR04 trigger pin
#define ECHO_PIN 4  // HC-SR04 echo pin

const int numberOfSensors = 8; // Number of sensors used
int sensorPins[numberOfSensors] = {A7 ,A6, A5, A4, A3, A2, A1, A0}; // Analog pins for the sensors
int sensorValues[numberOfSensors]; // Array to store the sensor values

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Adafruit_LSM6DS3TRC lsm6ds3trc;

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

double wantedRotation = 0; // Variable to store the rotation value

bool isDrivingForward = true; // Variable to store the direction of the robot

long duration, distance; // Variables to store the duration and distance for the HC-SR04 sensor

//-----------------------------------[Gyro]-----------------------------------

#define PI 3.1415926535897932384626433832795
double rotationInDegrees;
sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

//-----------------------------------[Setup function]---------------------------

void setup() {
  strip.begin();  // Initialize the Neopixel strip
  strip.show();   // Initialize all pixels to 'off'

  pinMode(MOTOR_LEFT_FORWARD,   OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD,  OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD,  OUTPUT);
  pinMode(MOTOR_LEFT_READ,  INPUT);
  pinMode(MOTOR_RIGHT_READ, INPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);
  pinMode(GRIPPER_PIN, OUTPUT);
  digitalWrite(GRIPPER_PIN, LOW);
  for(int i = 0;i<=7;i++) 
  {
    pinMode(sensorPins[i], INPUT);
  }
  Serial.begin(38400);

  while (!Serial)
    delay(1000); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit LSM6DS3TR-C test!");

  if (!lsm6ds3trc.begin_I2C()) {
    // if (!lsm6ds3trc.begin_SPI(LSM_CS)) {
    // if (!lsm6ds3trc.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DS3TR-C chip");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("LSM6DS3TR-C Found!");

  lsm6ds3trc.setGyroRange(LSM6DS_GYRO_RANGE_125_DPS);

  // lsm6ds3trc.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (lsm6ds3trc.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

    // lsm6ds3trc.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  Serial.print("Gyro range set to: ");
  switch (lsm6ds3trc.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DS33
  }

  // lsm6ds3trc.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (lsm6ds3trc.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

   // lsm6ds3trc.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (lsm6ds3trc.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  lsm6ds3trc.configInt1(false, false, true); // accelerometer DRDY on INT1
  lsm6ds3trc.configInt2(false, true, false); // gyro DRDY on INT2


  gripper(GRIPPER_OPEN);
  updateRotation();

  // turnToAngle("left");

  // Serial.println(wantedRotation);

  // delay(5000);

}

//-----------------------------------[Loop function]----------------------------

void loop() {
  // Get a new normalized sensor event
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6ds3trc.getEvent(&accel, &gyro, &temp);

  // Serial.print(temp.temperature);
  // Serial.print(" Temps,");

  // Serial.print(accel.acceleration.x);
  // Serial.print(" Accel.y,"); Serial.print(accel.acceleration.y);
  // Serial.print(" Accel.z,"); Serial.print(accel.acceleration.z);
  // Serial.print(",");

  // Serial.print(gyro.gyro.x);
  // Serial.print(" Gyro.y,"); Serial.print(gyro.gyro.y);
  // Serial.print(" Gyro.z,"); Serial.print(gyro.gyro.z);
  // Serial.println();
  // delayMicroseconds(10000);


  gripper(GRIPPER_CLOSED); // Close the gripper

  updateRotation(); // Update the rotation value
  Serial.println(rotationInDegrees); // Print the rotation value to Serial Monitor

  distanceSensor(); // Call the distanceSensor function to check for obstacles
  
  // driveForward(2.55);
  // Serial.print("wantedRotation:");
  // Serial.println(wantedRotation);

  // Serial.print("rotationInDegrees:");
  // Serial.println(rotationInDegrees);
  
}

//-----------------------------------[Neopixel]-------------------------------

// Set color for a specific Neopixel
void setPixelColor(int pixel, uint8_t red, uint8_t green, uint8_t blue) {
  strip.setPixelColor(pixel, strip.Color(red, green, blue));
  strip.show();
}

//-----------------------------------[Gripper]--------------------------------

void gripperToggle() {
  static unsigned long timer;
  static bool state;
  if (millis() > timer) {
    if (state == true) {
      gripper(GRIPPER_OPEN);
      state = false;
    } else {
      gripper(GRIPPER_CLOSED);
      state = true;
    }
    timer = millis() + GRIPPER_TOGGLE;
  }
}

void gripper(int pulse) {
  static unsigned long timer;
  static int pulse1;
  if (pulse > 0) {
    pulse1 = pulse;
  }
  if (millis() > timer) {
    digitalWrite(GRIPPER_PIN, HIGH);
    delayMicroseconds(pulse1);
    digitalWrite(GRIPPER_PIN, LOW);
    timer = millis() + GRIPPER_INTERVAL;
  }
}

//-----------------------------------[Echo Servo]-----------------------------

void servo(int pulse) {
  static unsigned long timer;
  static int pulse1;
  if (pulse > 0) {
    pulse1 = pulse;
  }
  if (millis() > timer) {
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(pulse1);
    digitalWrite(SERVO_PIN, LOW);
    timer = millis() + SERVO_INTERVAL;
  }
}


// Sets motor power to input
void setMotors(double LFWD, double LBACK, double RFWD, double RBACK) {
  analogWrite(MOTOR_LEFT_FORWARD,   round(LFWD * leftOffset));
  analogWrite(MOTOR_LEFT_BACKWARD,  round(LBACK * leftOffset));
  analogWrite(MOTOR_RIGHT_FORWARD,  round(RFWD * rightOffset));
  analogWrite(MOTOR_RIGHT_BACKWARD, round(RBACK * rightOffset));
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
  setMotors(speed, 0, 0, speed);
}
// Rotate right at 0-2 speed
void driveForward(double speed) {
  if (abs(rotationInDegrees - wantedRotation) > 3)
  {
    if (isRightTurnFaster(wantedRotation)) {
    setMotors(speed, 0, 1, 0);
    } else {
    setMotors(1, 0, speed, 0);
    } 
  } else {
    setMotors(speed, 0, speed, 0);
  }
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
  servo(SERVO_MID_PULSE);
}

void lookLeft() {
  servo(SERVO_MAX_PULSE);
  // Add a delay to give some time for the servo to move (you may need to adjust the delay duration)
  delay(800);
  // Measure distance
  distanceReader();
  // Check for obstacles
  if (distance >= stopDistance) {
    Serial.println("wants to turn left");
    driveStop();
    delay(3000);
  } 
}

// Function to close the gripper (move the servo to the closed position)
void lookRight() {
  servo(SERVO_MIN_PULSE);
  delay(800);
  // Measure distance
  distanceReader();
  // Check for obstacles
  if (distance <= stopDistance) {
    lookLeft();
  } else {
    driveRight(2);
    lookStraight();
  }
}

//-----------------------------------[Echo sensor]------------------------


void distanceReader()
{
    digitalWrite(TRIGGER_PIN, LOW); // Reset pin
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH); // High pulses for 10 ms
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);


    duration = pulseIn(ECHO_PIN, HIGH); // Reads pins

    distance = (duration / 2) * 0.0343; // 343 m/s per second as speed of sound
}

void distanceSensor(){
static unsigned long timer;
int interval = 200;
  if (millis() > timer) 
  {

    distanceReader();

    if (distance <= stopDistance)
      {
        
      }
    else
      {
        driveForward(2.55);
        lookLeft();
      }
      timer = millis() + interval;
    }
}

//-----------------------------------[Gyro]-----------------------------------

void updateRotation()
{
  lsm6ds3trc.getEvent(&accel, &gyro, &temp);
  static unsigned long timer;
  int interval = 100;
  if(millis() > timer)
  {
    double rotated = (radiansToDegrees(gyro.gyro.z) / 10);
    Serial.println(rotated);
    if (abs(rotated) >= 0.2) {
        rotationInDegrees += rotated;
        rotationInDegrees = wrapAngle(rotationInDegrees); // Call the wrapAngle function to ensure the rotation is within 0-359 degrees
    }

    timer = millis() + interval;  
  } 
}

double radiansToDegrees(double radians)
{
  return radians * (180 / PI);
}


void turnToAngle(String direction)
{

    if (direction == "left")
    {
      wantedRotation = +90;
      Serial.println("left");
    }
    else if (direction == "right")
    {
      wantedRotation = -90;
    }

    int speed;

    if (abs(rotationInDegrees - wantedRotation) > 3)
    {
      if (isRightTurnFaster(wantedRotation)) {
      setMotors(speed, 0, 0, 0);
      } else {
      setMotors(0, 0, speed, 0);
      } 
    }
    
}

bool isRightTurnFaster(double wantedRotation)
{
    double rightDiffrence = wrapAngle(rotationInDegrees - wantedRotation);
    double leftDiffrence = wrapAngle(wantedRotation - rotationInDegrees);
    if (rightDiffrence > leftDiffrence)
    {
        return false;
    }
    else
    {
        return true;
    }
}

double wrapAngle(int rotationToWrap) {
    if (rotationToWrap >= 360 || rotationToWrap < 0) {
        rotationToWrap = rotationToWrap % 360; // Ensure angle is within range
        if (rotationToWrap < 0) {
            rotationToWrap += 360; // Adjust negative angles
        }
    }
    return rotationToWrap;
}


double setWantedRotation(double newWantedRotation)
{
  wantedRotation = newWantedRotation;
}