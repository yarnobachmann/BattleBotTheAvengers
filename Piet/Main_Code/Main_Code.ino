#define   GRIPPER_PIN     12      // servo pin  
#define   GRIPPER_OPEN    1800   // pulse length servo open
#define   GRIPPER_CLOSED  1100    // pulse length servo closed
#define   SERVO_INTERVAL  20    // time between pulse
#define   GRIPPER_TOGGLE  1000  // toggle gripper every second
#define   TRIG_PIN 8
#define   ECHO_PIN 9
#define   MOTOR_LEFT_BACKWARD   11    // Motor Pin
#define   MOTOR_LEFT_FORWARD    10    // Motor Pin
#define   MOTOR_RIGHT_BACKWARD  6     // Motor Pin
#define   MOTOR_RIGHT_FORWARD   5     // Motor Pin 
#define   MOTOR_AFWIJKING       4     // Afwijking van de motor (moet nog in het Engels) 

#include <Adafruit_NeoPixel.h>

// NeoPixel setup
#define NEOPIXEL_PIN     13  // Define the pin connected to NeoPixels
#define NUMPIXELS        4  // Number of NeoPixels

Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

const int LIGHT_SENSOR[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
int LIGHT_VALUE  =   850;  // Light value at the beginning

long currentMillis;
bool currentLightState = false;
unsigned long previousMillis = 0;
const long interval = 100; // Interval in milliseconden
bool end = false;

unsigned long endingTimer = 500;
unsigned long distanceMillis = 0;
float duration = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(GRIPPER_PIN, OUTPUT);
  digitalWrite(GRIPPER_PIN, LOW);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pixels.begin();

  Serial.begin(9600);

  for (int i = 0; i < 8; i++)
  {
    pinMode(LIGHT_SENSOR[i], INPUT);
  }
  stopLights();
  start();
}
  

void loop() {
  forwardLights();
  followTheLine();
//  // put your main code here, to run repeatedly:
// while (!allBlack())
//    {
//      forwardLights();
//      followTheLine();
//    }
//    if (allBlack()){
//    theEnd(); 
//    
//    }
//
//  while (end)
//  {
//    
//  }
}
    
void followTheLine()
{
  if (analogRead(LIGHT_SENSOR[7]) > LIGHT_VALUE || analogRead(LIGHT_SENSOR[6]) > LIGHT_VALUE)
  {
    specialTurnLeft();
  }
//  
//  else if (analogRead(LIGHT_SENSOR[7]) > LIGHT_VALUE && analogRead(LIGHT_SENSOR[0]) < LIGHT_VALUE)
//  {
//    turnLeft(245); 
//  }
  else if (analogRead(LIGHT_SENSOR[2]) > LIGHT_VALUE || analogRead(LIGHT_SENSOR[3]) > LIGHT_VALUE)
  {
    driveForward(220);
  }
  else if (analogRead(LIGHT_SENSOR[5]) > LIGHT_VALUE || analogRead(LIGHT_SENSOR[4]) > LIGHT_VALUE)
  {
    turnLeft(210);
  }
  else if (analogRead(LIGHT_SENSOR[0]) > LIGHT_VALUE || analogRead(LIGHT_SENSOR[1]) > LIGHT_VALUE)
  {
     turnRight(210);
  }
  else if (analogRead(LIGHT_SENSOR[0]) < LIGHT_VALUE && analogRead(LIGHT_SENSOR[3]) < LIGHT_VALUE && analogRead(LIGHT_SENSOR[5]) < LIGHT_VALUE && analogRead(LIGHT_SENSOR[7]) < LIGHT_VALUE)
  {
    turnRight(245);
  }
  else 
  {
    driveForward(220);
  }
}

bool allBlack()
{
  int blackDetected = 0;
  for (int i = 0; i < 8; i++)
  {
    if (analogRead(LIGHT_SENSOR[i]) > LIGHT_VALUE)
    {
      blackDetected++;
    }
  }

  if (blackDetected >= 6)
  {
    if (endingTimer == 0)
    {
      endingTimer = millis();
    }
    else
    {
      if (millis() - endingTimer >= 300)
      {
        Serial.println("true");
        return true;
      }
    }
  }
  else
  {
    endingTimer = 0;
  }

  Serial.println("false");
  return false;
 }

void theEnd()
{ 
  motorStop();
  driveBackwards(245);
  delay(300);
  motorStop();
  for (int i = 0; i < 8; i++)
  {
    servo(GRIPPER_OPEN);
    delay(100);
  }
  delay(10);
  driveBackwards(245);
  delay(3000);
  motorStop();
}

void gripperToggle() 
{
  static unsigned long timer;
  static bool state;
  if (millis() > timer) {
    if (state == true) {
      servo(GRIPPER_OPEN);
      state = false;
    } else {
      servo(GRIPPER_CLOSED);
      state = true;
    }
    timer = millis() + GRIPPER_TOGGLE;
  }
}

void servo(int pulse) 
{
  static unsigned long timer;
  static int pulse1;
  if (pulse > 0) 
  {
    pulse1 = pulse;
  }
  if (millis() > timer) 
  {
    digitalWrite(GRIPPER_PIN, HIGH);
    delayMicroseconds(pulse1);
    digitalWrite(GRIPPER_PIN, LOW);
    timer = millis() + SERVO_INTERVAL;
  }
}

void driveForward(int speed)
{
  analogWrite(MOTOR_LEFT_BACKWARD, 0);
  analogWrite(MOTOR_LEFT_FORWARD, speed - MOTOR_AFWIJKING);
  analogWrite(MOTOR_RIGHT_BACKWARD, 0);
  analogWrite(MOTOR_RIGHT_FORWARD, speed);
}

void turnLeft(int speed)
{
//  analogWrite(MOTOR_LEFT_BACKWARD, 255);
//  analogWrite(MOTOR_LEFT_FORWARD, 0);
//  analogWrite(MOTOR_RIGHT_BACKWARD, 0);
//  analogWrite(MOTOR_RIGHT_FORWARD, 255);
//
//  delay(10);
//  
  analogWrite(MOTOR_LEFT_BACKWARD, speed - MOTOR_AFWIJKING);
  analogWrite(MOTOR_LEFT_FORWARD, 0);
  analogWrite(MOTOR_RIGHT_BACKWARD, 0);
  analogWrite(MOTOR_RIGHT_FORWARD, speed);
}

void specialTurnLeft()
{
  driveForward(210);
  delay(300);
  if (analogRead(LIGHT_SENSOR[5]) > LIGHT_VALUE && analogRead(LIGHT_SENSOR[2]) > LIGHT_VALUE )
  {
    theEnd();
    while(true){
      
    }
  }
  turnLeft(215);
  delay(300);
  while (true)
  {
    delay(1);
    if (analogRead(LIGHT_SENSOR[2]) > LIGHT_VALUE || analogRead(LIGHT_SENSOR[3]) > LIGHT_VALUE)
    {
      break;
    }
  }
  motorStop();
}

void turnRight(int speed)
{
//  analogWrite(MOTOR_LEFT_BACKWARD, 0);
//  analogWrite(MOTOR_LEFT_FORWARD, 255);
//  analogWrite(MOTOR_RIGHT_BACKWARD, 255);
//  analogWrite(MOTOR_RIGHT_FORWARD, 0);
//
//  delay(10);

  analogWrite(MOTOR_LEFT_BACKWARD, 0);
  analogWrite(MOTOR_LEFT_FORWARD, speed - MOTOR_AFWIJKING);
  analogWrite(MOTOR_RIGHT_BACKWARD, speed);
  analogWrite(MOTOR_RIGHT_FORWARD, 0);
}


void driveBackwards(int speed)
{
  analogWrite(MOTOR_LEFT_BACKWARD, speed - MOTOR_AFWIJKING);
  analogWrite(MOTOR_LEFT_FORWARD, 0);
  analogWrite(MOTOR_RIGHT_BACKWARD, speed);
  analogWrite(MOTOR_RIGHT_FORWARD, 0); 
}

void motorStop()
{
  analogWrite(MOTOR_LEFT_BACKWARD, 0);
  analogWrite(MOTOR_LEFT_FORWARD, 0);
  analogWrite(MOTOR_RIGHT_BACKWARD, 0);
  analogWrite(MOTOR_RIGHT_FORWARD, 0);
}


void start()
{
  Serial.print("Current LIGHT_VALUE: ");
  Serial.println(LIGHT_VALUE);
  
  int blackLineSum = 0;
  int blackLineCount = 0;
  
  for (int i = 0; i < 3; i++)
  {
    
      while(getDistance() > 24);
  }
    waitLights();
    startLights();
    driveForward(255);
  
    while(blackLineCount < 4)
    {
      while (true)
      {
        if (analogRead(LIGHT_SENSOR[3]) > LIGHT_VALUE)
        {
          break;
        }
      }
      while (true)
      {
        if (analogRead(LIGHT_SENSOR[3]) < LIGHT_VALUE)
        {
          break;
        }
      }
        blackLineSum += getAverageLightValue();    
        blackLineCount++; 
    }
      motorStop();

     LIGHT_VALUE = blackLineSum / blackLineCount;

     Serial.print("New LIGHT_VALUE: ");
     Serial.println(LIGHT_VALUE);
     for (int i = 0; i < 100; i++)
     {
        delay(10);
        servo(GRIPPER_CLOSED);
     }

      turnLeft(200);
      delay(500);
      while(true)
      {
        if(analogRead(LIGHT_SENSOR[4]) > LIGHT_VALUE)
        {
          break;
        }
       }
  motorStop();
}


int getAverageLightValue()
{
  int sum = 0;
  for (int i = 0; i < 8; i++)
  {
    sum += analogRead(LIGHT_SENSOR[i]);
  }
  return sum / 8;
}

// Pixel 0 is links achter
// Pixel 1 is rechts achter
// Pixel 2 is rechts voor
// Pixel 3 is links voor
// Kleurvolgerde is GRB


void lightsOff(){
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.setPixelColor(1, pixels.Color(0, 0, 0));
  pixels.setPixelColor(2, pixels.Color(0, 0, 0));
  pixels.setPixelColor(3, pixels.Color(0, 0, 0));
  pixels.show();
  delay(200);
}


void waitLights()
{
  pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  pixels.setPixelColor(1, pixels.Color(0, 0, 255));
  pixels.setPixelColor(2, pixels.Color(0, 0, 255));
  pixels.setPixelColor(3, pixels.Color(0, 255, 0));
  pixels.show();
  delay(200);

  pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  pixels.setPixelColor(1, pixels.Color(0, 0, 255));
  pixels.setPixelColor(2, pixels.Color(0, 255, 0));
  pixels.setPixelColor(3, pixels.Color(0, 0, 255));
  pixels.show();
  delay(200);

  pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  pixels.setPixelColor(1, pixels.Color(0, 255, 0));
  pixels.setPixelColor(2, pixels.Color(0, 0, 255));
  pixels.setPixelColor(3, pixels.Color(0, 0, 255));
  pixels.show();
  delay(200);

  pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  pixels.setPixelColor(1, pixels.Color(0, 0, 255));
  pixels.setPixelColor(2, pixels.Color(0, 0, 255));
  pixels.setPixelColor(3, pixels.Color(0, 0, 255));
  pixels.show();
  delay(200);

  pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  pixels.setPixelColor(1, pixels.Color(0, 0, 255));
  pixels.setPixelColor(2, pixels.Color(0, 0, 255));
  pixels.setPixelColor(3, pixels.Color(0, 255, 0));
  pixels.show();
  delay(200);

  pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  pixels.setPixelColor(1, pixels.Color(0, 0, 255));
  pixels.setPixelColor(2, pixels.Color(0, 255, 0));
  pixels.setPixelColor(3, pixels.Color(0, 0, 255));
  pixels.show();
  delay(200);

  pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  pixels.setPixelColor(1, pixels.Color(0, 255, 0));
  pixels.setPixelColor(2, pixels.Color(0, 0, 255));
  pixels.setPixelColor(3, pixels.Color(0, 0, 255));
  pixels.show();
  delay(200);

  pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  pixels.setPixelColor(1, pixels.Color(0, 0, 255));
  pixels.setPixelColor(2, pixels.Color(0, 0, 255));
  pixels.setPixelColor(3, pixels.Color(0, 0, 255));
  pixels.show();
  delay(200);

  pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  pixels.setPixelColor(1, pixels.Color(0, 0, 255));
  pixels.setPixelColor(2, pixels.Color(0, 0, 255));
  pixels.setPixelColor(3, pixels.Color(0, 255, 0));
  pixels.show();
  delay(200);

  pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  pixels.setPixelColor(1, pixels.Color(0, 0, 255));
  pixels.setPixelColor(2, pixels.Color(0, 255, 0));
  pixels.setPixelColor(3, pixels.Color(0, 0, 255));
  pixels.show();
  delay(200);

  pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  pixels.setPixelColor(1, pixels.Color(0, 255, 0));
  pixels.setPixelColor(2, pixels.Color(0, 0, 255));
  pixels.setPixelColor(3, pixels.Color(0, 0, 255));
  pixels.show();
  delay(200);

  pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  pixels.setPixelColor(1, pixels.Color(0, 0, 255));
  pixels.setPixelColor(2, pixels.Color(0, 0, 255));
  pixels.setPixelColor(3, pixels.Color(0, 0, 255));
  pixels.show();
  delay(200);
}

void forwardLights()
{
  if(millis() - currentMillis >= 200)
  {
    currentMillis = millis();
    if(currentLightState)
    {
      pixels.setPixelColor(0, pixels.Color(0, 255, 0));
      pixels.setPixelColor(1, pixels.Color(0, 0, 255));
      pixels.setPixelColor(2, pixels.Color(255, 255, 255));
      pixels.setPixelColor(3, pixels.Color(255, 255, 255));
      pixels.show();
      currentLightState = !currentLightState;
    } 
    else
    {
      pixels.setPixelColor(0, pixels.Color(0, 0, 255));
      pixels.setPixelColor(1, pixels.Color(0, 255, 0));
      pixels.setPixelColor(2, pixels.Color(255, 255, 255));
      pixels.setPixelColor(3, pixels.Color(255, 255, 255));
      pixels.show();
      currentLightState = !currentLightState;
    }
  }
}

void startLights()
{
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.setPixelColor(1, pixels.Color(0, 0, 0));
  pixels.setPixelColor(2, pixels.Color(255, 255, 255));
  pixels.setPixelColor(3, pixels.Color(255, 255, 255));
  pixels.show();
}

void stopLights()
{
  pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  pixels.setPixelColor(1, pixels.Color(0, 255, 0));
  pixels.setPixelColor(2, pixels.Color(0, 255, 0));
  pixels.setPixelColor(3, pixels.Color(0, 255, 0));
  pixels.show();
}

float getDistance()
{
  if (millis() >= distanceMillis) 
  {
    distanceMillis = millis() + 200;
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    duration = pulseIn(ECHO_PIN, HIGH);
    //Serial.println(0.017 * duration);
    return 0.017 * duration;
  }
}
