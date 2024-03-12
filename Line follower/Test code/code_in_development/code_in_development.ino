#include <Adafruit_NeoPixel.h>

const int NEO_PIXEL_PIN = 7; // neopixel pin
const int NEO_PIXEL_COUNT = 4; // amount of neopixel lights
Adafruit_NeoPixel pixels(NEO_PIXEL_COUNT, NEO_PIXEL_PIN, NEO_GRB + NEO_KHZ800);

const int MOTOR_A_1 = 11; // Left wheel backwards
const int MOTOR_A_2 = 10; // Left wheel forwards
const int MOTOR_B_1 = 6; // Right wheel backwards
const int MOTOR_B_2 = 5; // Right wheel forwards

const int gripperPin = 9;
const int GRIPPER_OPEN = 1600;
const int GRIPPER_CLOSED = 950;
const int SERVO_INTERVAL = 20;
const int GRIPPER_TOGGLE = 1000;

const int trigPin = 4;
const int echoPin = 8;

int lineValues[] = {0, 0, 0, 0, 0, 0};

int LINE = 900;

const int LINE_SENSOR[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Brown, 0th sensor
// Orange, 1st sensor
// Yellow, 2nd sensor
// Green, 3rd sensor
// Blue, 4th sensor
// Purple, 5th sensor
// White, 6th sensor
// Grey, 7th sensor

unsigned long startTime = 0; // Variable to store the start time
unsigned long currentTime = 0; // Variable to store the current time

float duration;
float distance;

int lineCheck1 = 0;
int lineCheck2 = 0;
int lineCheck3 = 0;
int lineCheck4 = 0; 
int lineCheck5 = 0;
int lineCheck6 = 0;
int lineCheck7 = 0;
int lineCheck8 = 0;

bool START = false;

unsigned long previousMillis = 0;
const long interval = 250;



void setup() {
  Serial.begin(9600);

  pinMode(gripperPin, OUTPUT);
  digitalWrite(gripperPin, LOW);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  pinMode(MOTOR_A_1, OUTPUT);
  pinMode(MOTOR_A_2, OUTPUT);
  pinMode(MOTOR_B_1, OUTPUT);
  pinMode(MOTOR_B_2, OUTPUT);

  for (int i = 0; i < 8; i++) {
    pinMode(LINE_SENSOR[i], INPUT);
  }
}

void color0() {
  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(75, 255, 0));
  pixels.setPixelColor(1, pixels.Color(0, 255, 0));
  pixels.setPixelColor(2, pixels.Color(75, 255, 0));
  pixels.setPixelColor(3, pixels.Color(0, 255, 0));
  pixels.show();
}

void colors() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // Save the last time the LED strip was updated
    previousMillis = currentMillis;

    static bool alternate = false;

    if (alternate) {
      pixels.begin();
      pixels.setPixelColor(0, pixels.Color(0, 255, 0));
      pixels.setPixelColor(1, pixels.Color(75, 255, 0));
      pixels.setPixelColor(2, pixels.Color(0, 255, 0));
      pixels.setPixelColor(3, pixels.Color(75, 255, 0));
      pixels.show();
      alternate = false;
    } else {
      pixels.begin();
      pixels.setPixelColor(0, pixels.Color(75, 255, 0));
      pixels.setPixelColor(1, pixels.Color(0, 255, 0));
      pixels.setPixelColor(2, pixels.Color(75, 255, 0));
      pixels.setPixelColor(3, pixels.Color(0, 255, 0));
      pixels.show();
      alternate = true;
    }
  }
}

void start() {
  for(int i = 0; i > 0; i++){
    
  }
}

void lineFollower() {
  lineCheck1 = analogRead(LINE_SENSOR[0]);
  lineCheck2 = analogRead(LINE_SENSOR[1]);
  lineCheck3 = analogRead(LINE_SENSOR[2]);
  lineCheck4 = analogRead(LINE_SENSOR[3]);
  lineCheck5 = analogRead(LINE_SENSOR[4]);
  lineCheck6 = analogRead(LINE_SENSOR[5]);
  lineCheck7 = analogRead(LINE_SENSOR[6]);
  lineCheck8 = analogRead(LINE_SENSOR[7]);
  
  if((lineCheck1 > LINE) && (lineCheck2 > LINE) && (lineCheck3 > LINE ) && (lineCheck4 > LINE ) && 
  (lineCheck5 > LINE) && (lineCheck6 > LINE ) && (lineCheck7 > LINE) && (lineCheck8 > LINE)){
    if (startTime == 0) {
      startTime = millis(); // Start the timer
    } else {
      currentTime = millis(); // Update the current time
      if (currentTime - startTime >= 500) {
        // Turn off motors
        analogWrite(MOTOR_A_1, 0);
        analogWrite(MOTOR_A_2, 0);
        analogWrite(MOTOR_B_1, 0);
        analogWrite(MOTOR_B_2, 0);
        color0();
      } else {
        // Run motors at slower speed
        analogWrite(MOTOR_A_1, 0);
        analogWrite(MOTOR_A_2, 100);
        analogWrite(MOTOR_B_1, 0);
        analogWrite(MOTOR_B_2, 100);
      } 
    } 
  }
  else if((lineCheck1 > LINE) && (lineCheck2 > LINE)  && (lineCheck3 > LINE )  && (lineCheck4 > LINE )  && (lineCheck5 > LINE) ){
    analogWrite(MOTOR_A_2, 255);
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_B_2, 255);
    startTime = 0;
    colors();
    delay(100);
  }
  else if((lineCheck4 > LINE )  && (lineCheck5 > LINE)  && (lineCheck6 > LINE)  && (lineCheck7 > LINE)  && (lineCheck8 > LINE) ){
    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, 255);
    analogWrite(MOTOR_A_2, 255);
    startTime = 0;
    colors();
    delay(100);
  }
  else if((lineCheck1 > LINE)){
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_2, 220);
    analogWrite(MOTOR_A_1, 255);
    analogWrite(MOTOR_B_1, 0);
    startTime = 0;
    colors();
  }
  else if((lineCheck2 > LINE)){
    analogWrite(MOTOR_A_2, 80);
    analogWrite(MOTOR_B_2, 255);
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_B_1, 0);
    startTime = 0;
    colors();
  }
  else if((lineCheck3 > LINE )){
    analogWrite(MOTOR_A_2, 160);
    analogWrite(MOTOR_B_2, 255);
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_B_1, 0);
    startTime = 0;
    colors();
  }
  else if(((lineCheck4 > LINE ) ) || ((lineCheck5 > LINE) )){
    analogWrite(MOTOR_A_2, 255);
    analogWrite(MOTOR_B_2, 255);
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_B_1, 0);
    startTime = 0;
    colors();
  }
  else if((lineCheck6 > LINE) ){
    analogWrite(MOTOR_B_2, 160);
    analogWrite(MOTOR_A_2, 255);
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_B_1, 0);
    startTime = 0;
    colors();
  }
  else if((lineCheck7 > LINE) ){
    analogWrite(MOTOR_B_2, 80);
    analogWrite(MOTOR_A_2, 255);
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_B_1, 0);
    startTime = 0;
    colors();
  }
  else if((lineCheck8 > LINE) ){
    analogWrite(MOTOR_B_2, 0);
    analogWrite(MOTOR_A_2, 220);
    analogWrite(MOTOR_B_1, 255);
    analogWrite(MOTOR_A_1, 0);
    startTime = 0;
    colors();
  }
  else{
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, 0);
    color0(); 
  }
} 

void servo(int pulse) {
  static unsigned long timer;
  static int pulse1;
  if (pulse > 0) {
    pulse1 = pulse;
  }
  if (millis() > timer) {
    digitalWrite(gripperPin, HIGH);
    delayMicroseconds(pulse1);
    digitalWrite(gripperPin, LOW);
    timer = millis() + SERVO_INTERVAL;
  }
}

void gripperToggle() {
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

float getDistance()
{
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  //Serial.println(0.017 * duration);
  return 0.017 * duration;
}

int average(int numbers[], int size)
{
  double sum = 0;
  for (int x = 0; x < size; x++)
  {
    sum += numbers[x];
  }
  return (int)sum / (double)size;
}


void loop() {
  color0();
  Serial.println(analogRead(LINE_SENSOR[0]));
  
  if (START) {
    lineFollower();
  }
  else {
    getDistance();

    for (int i = 0; i < 3; i++) {
      do {
        delay(100);
      } while (getDistance() > 24);
    }
    analogWrite(MOTOR_B_2, 255);
    analogWrite(MOTOR_A_2, 255);

    for (int i = 0; i < 6; i += 2) {
      while (analogRead(LINE_SENSOR[0]) > LINE && analogRead(LINE_SENSOR[6]) > LINE) {
        delay(1);
      }

      lineValues[i] = analogRead(LINE_SENSOR[3]);
      delay(100);
      
      analogWrite(MOTOR_B_2, 150);
      analogWrite(MOTOR_A_2, 150);
      while (analogRead(LINE_SENSOR[0]) < LINE && analogRead(LINE_SENSOR[6]) < LINE) {
        delay(1);
      }

      lineValues[i + 1] = analogRead(LINE_SENSOR[3]);
      delay(100);
      
      analogWrite(MOTOR_B_2, 150);
      analogWrite(MOTOR_A_2, 150);
    }
    analogWrite(MOTOR_B_2, 200);
    analogWrite(MOTOR_A_2, 200);
    delay(200);

    LINE = average(lineValues, 6) + 250;
    
    while (analogRead(LINE_SENSOR[4]) > LINE && analogRead(LINE_SENSOR[5]) > LINE) {
      delay(1);
    }
    while (analogRead(LINE_SENSOR[4]) < LINE) {
      delay(1);
    }
    servo(GRIPPER_CLOSED);
    analogWrite(MOTOR_B_2, 150);
    analogWrite(MOTOR_A_2, 0);
    delay(1000);

    do {
      analogWrite(MOTOR_B_2, 150);
      analogWrite(MOTOR_A_2, 0);
      if (analogRead(LINE_SENSOR[6]) > LINE) {
        break;
      }
    }
    while (true);
    START = true;
  }
}
