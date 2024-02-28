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

const int gripper = 6; // gripper GR

float leftOffsetPercentage = 1;      
float rightOffsetPercentage = 3; 

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Set color for a specific Neopixel
void setPixelColor(int pixel, uint8_t red, uint8_t green, uint8_t blue) {
  strip.setPixelColor(pixel, strip.Color(red, green, blue));
  strip.show();
}

void testLight() {
  // Set color for each Neopixel individually
  setPixelColor(0, 0, 255, 0);  // Red for the first Neopixel
  setPixelColor(1, 0, 255, 0);  // Red for the second Neopixel
  setPixelColor(2, 0, 0, 0);    // Turn off other Neopixels
  setPixelColor(3, 0, 0, 0);    // Turn off other Neopixels
}

// Sets motor power to input
void setMotors(int LFWD, int LBACK, int RFWD, int RBACK){
  analogWrite(motorLeftFwd,   LFWD * rightOffsetPercentage);
  analogWrite(motorLeftBack,  LBACK * rightOffsetPercentage);
  analogWrite(motorRightFwd,  RFWD * leftOffsetPercentage);
  analogWrite(motorRightBack, RBACK * leftOffsetPercentage);
}

// Drive forwards at 0-255 speed
void driveForward(int speed){
  setMotors(speed, 0 , speed, 0);
  lightForward(); // Turn on forward lights
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


void setup() {
  strip.begin();  // Initialize the Neopixel strip
  strip.show();   // Initialize all pixels to 'off'
    
  pinMode(motorLeftFwd,   OUTPUT);
  pinMode(motorLeftBack,  OUTPUT);
  pinMode(motorRightBack, OUTPUT);
  pinMode(motorRightFwd,  OUTPUT);
  pinMode(gripper, OUTPUT);
  Serial.begin(9600);
}

void loop() {
    openGripper();
    delay(1000);
    closeGripper();
}