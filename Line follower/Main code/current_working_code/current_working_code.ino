#include <Adafruit_NeoPixel.h>

const int NEO_PIXEL_PIN = 7; // neopixel pin
const int NEO_PIXEL_COUNT = 4; // amount of neopixel lights
Adafruit_NeoPixel pixels(NEO_PIXEL_COUNT, NEO_PIXEL_PIN, NEO_GRB + NEO_KHZ800);

const int MOTOR_A_1 = 11; // Left wheel backwards
const int MOTOR_A_2 = 10; // Left wheel forwards
const int MOTOR_B_1 = 6; // Right wheel backwards
const int MOTOR_B_2 = 5; // Right wheel forwards

const int gripperPin = 9; // Pin for the gripper

const int trigPin = 4; // Pin for the ultrasonic sensor
const int echoPin = 8; // Pin for the ultrasonic sensor

const int LINE_SENSOR[8] = {A0, A1, A2, A3, A4, A5, A6, A7}; // Array for the line sensor pins
// Brown, 0th sensor
// Orange, 1st sensor
// Yellow, 2nd sensor
// Green, 3rd sensor
// Blue, 4th sensor
// Purple, 5th sensor
// White, 6th sensor
// Grey, 7th sensor

const int LOWVALUE = 550; // Value for white
const int LOWVALUEUNIQUE = 550; 
const int HIGHVALUE = 1005; // Value for black

const int GRIPPER_OPEN = 1800; // Pulse length to open gripper
const int GRIPPER_CLOSED = 950; // Pulse length to close gripper
bool gripperOpen = true; // Set boolean for gripper open to true

// Variables for the last used values of the motors
int lastValueA1 = 0;
int lastValueA2 = 0;
int lastValueB1 = 0;
int lastValueB2 = 0;

int lineValues[] = {0, 0, 0, 0, 0, 0};

int LINE = 900; // Set the value of the line to 900

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

// Function to set the neo pixels at the start
void color0() {
  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(75, 255, 0));
  pixels.setPixelColor(1, pixels.Color(0, 255, 0));
  pixels.setPixelColor(2, pixels.Color(75, 255, 0));
  pixels.setPixelColor(3, pixels.Color(0, 255, 0));
  pixels.show();
}

// Function to switch the neo light colors around
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

void lineFollower() {
  lineCheck1 = analogRead(LINE_SENSOR[0]);
  lineCheck2 = analogRead(LINE_SENSOR[1]);
  lineCheck3 = analogRead(LINE_SENSOR[2]);
  lineCheck4 = analogRead(LINE_SENSOR[3]);
  lineCheck5 = analogRead(LINE_SENSOR[4]);
  lineCheck6 = analogRead(LINE_SENSOR[5]);
  lineCheck7 = analogRead(LINE_SENSOR[6]);
  lineCheck8 = analogRead(LINE_SENSOR[7]);

  
  if((lineCheck1 > LOWVALUE) && (lineCheck1 < HIGHVALUE) && (lineCheck2 > LOWVALUE) && (lineCheck2 < HIGHVALUE) && (lineCheck3 > LOWVALUE) && (lineCheck3 < HIGHVALUE) && (lineCheck4 > LOWVALUE) && (lineCheck4 < HIGHVALUE) &&
  (lineCheck5 > LOWVALUE) && (lineCheck5 < HIGHVALUE) && (lineCheck6 > LOWVALUE) && (lineCheck6 < HIGHVALUE) && (lineCheck7 > LOWVALUE) && (lineCheck7 < HIGHVALUE) && (lineCheck8 > LOWVALUE) && (lineCheck8 < HIGHVALUE)){
    if (startTime == 0) {
      startTime = millis(); // Start the timer
    } else {
      currentTime = millis(); // Update the current time
      if (currentTime - startTime >= 150) {
        analogWrite(MOTOR_A_1, 255);
        analogWrite(MOTOR_A_2, 0);
        analogWrite(MOTOR_B_1, 255);
        analogWrite(MOTOR_B_2, 0);
        delay(300);
        servo(GRIPPER_OPEN);
        delay(1500);
        for (int i = 0; i < 10;) {
          analogWrite(MOTOR_A_1, 0);
          analogWrite(MOTOR_A_2, 0);
          analogWrite(MOTOR_B_1, 0);
          analogWrite(MOTOR_B_2, 0);
        }
        
      } else {
        // Run motors at slower speed
        analogWrite(MOTOR_A_1, 0);
        analogWrite(MOTOR_A_2, 100);
        analogWrite(MOTOR_B_1, 0);
        analogWrite(MOTOR_B_2, 100);
      } 
    } 
  }
  else if ((lineCheck1 > LOWVALUE) && (lineCheck1 < HIGHVALUE) && (lineCheck2 > LOWVALUE) && (lineCheck2 < HIGHVALUE) && (lineCheck3 > LOWVALUE) && (lineCheck3 < HIGHVALUE) && (lineCheck4 > LOWVALUE) && (lineCheck4 < HIGHVALUE) && (lineCheck5 > LOWVALUE) && (lineCheck5 < HIGHVALUE)){
    analogWrite(MOTOR_A_2, 255);
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_B_2, 255);
    startTime = 0;
    delay(100);
    lastValueA1 = 220;
    lastValueA2 = 0;
    lastValueB1 = 0;
    lastValueB2 = 255;
  }
  else if ((lineCheck4 > LOWVALUE) && (lineCheck4 < HIGHVALUE) && (lineCheck5 > LOWVALUE) && (lineCheck5 < HIGHVALUE) && (lineCheck6 > LOWVALUE) && (lineCheck6 < HIGHVALUE) && (lineCheck7 > LOWVALUE) && (lineCheck7 < HIGHVALUE) && (lineCheck8 > LOWVALUE) && (lineCheck8 < HIGHVALUE)){
    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, 255);
    analogWrite(MOTOR_A_2, 255);
    startTime = 0;
    delay(100);
    lastValueA1 = 0;
    lastValueA2 = 255;
    lastValueB1 = 220;
    lastValueB2 = 0;
  }
  else if ((lineCheck1 > LOWVALUE) && (lineCheck1 < HIGHVALUE)){
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_2, 220);
    analogWrite(MOTOR_A_1, 255);
    analogWrite(MOTOR_B_1, 0);
    startTime = 0;
    lastValueA1 = 255;
    lastValueA2 = 0;
    lastValueB1 = 0;
    lastValueB2 = 220;
  }
  else if ((lineCheck2 > LOWVALUE) && (lineCheck2 < HIGHVALUE)){
    analogWrite(MOTOR_A_2, 80);
    analogWrite(MOTOR_B_2, 255);
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_B_1, 0);
    startTime = 0;
    colors();
    lastValueA1 = 220;
    lastValueA2 = 0;
    lastValueB1 = 0;
    lastValueB2 = 255;
  }
  else if ((lineCheck3 > LOWVALUE) && (lineCheck3 < HIGHVALUE)){
    analogWrite(MOTOR_A_2, 160);
    analogWrite(MOTOR_B_2, 255);
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_B_1, 0);
    startTime = 0;
    lastValueA1 = 220;
    lastValueA2 = 0;
    lastValueB1 = 0;
    lastValueB2 = 255;
  }
  else if ((lineCheck4 > LOWVALUE) && (lineCheck4 < HIGHVALUE)){
    analogWrite(MOTOR_A_2, 255);
    analogWrite(MOTOR_B_2, 255);
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_B_1, 0);
    startTime = 0;
    lastValueA1 = 0;
    lastValueA2 = 255;
    lastValueB1 = 0;
    lastValueB2 = 220;
  }
  else if ((lineCheck5 > LOWVALUE) && (lineCheck5 < HIGHVALUE)){
    analogWrite(MOTOR_A_2, 255);
    analogWrite(MOTOR_B_2, 255);
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_B_1, 0);
    startTime = 0;
    lastValueA1 = 220;
    lastValueA2 = 0;
    lastValueB1 = 0;
    lastValueB2 = 255;
  }
  else if((lineCheck6 > LOWVALUEUNIQUE) && (lineCheck6 < HIGHVALUE)){
    analogWrite(MOTOR_B_2, 160);
    analogWrite(MOTOR_A_2, 255);
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_B_1, 0);
    startTime = 0;
    lastValueA1 = 0;
    lastValueA2 = 255;
    lastValueB1 = 220;
    lastValueB2 = 0;
  }
  else if((lineCheck7 > LOWVALUE) && (lineCheck7 < HIGHVALUE)){
    analogWrite(MOTOR_B_2, 80);
    analogWrite(MOTOR_A_2, 255);
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_B_1, 0);
    startTime = 0;
    lastValueA1 = 0;
    lastValueA2 = 255;
    lastValueB1 = 220;
    lastValueB2 = 0;
  }
  else if((lineCheck8 > LOWVALUE) && (lineCheck8 < HIGHVALUE)){
    analogWrite(MOTOR_B_2, 0);
    analogWrite(MOTOR_A_2, 220);
    analogWrite(MOTOR_B_1, 255);
    analogWrite(MOTOR_A_1, 0);
    startTime = 0;
    lastValueA1 = 0;
    lastValueA2 = 255;
    lastValueB1 = 220;
    lastValueB2 = 0;
  }
  else{
    analogWrite(MOTOR_A_1, lastValueA1);
    analogWrite(MOTOR_A_2, lastValueA2);
    analogWrite(MOTOR_B_1, lastValueB1);
    analogWrite(MOTOR_B_2, lastValueB2);
  } 
}

void servo(int pulse) {
  // Control the servo position using PWM
  digitalWrite(gripperPin, HIGH);
  delayMicroseconds(pulse);
  digitalWrite(gripperPin, LOW);
}

void gripperToggle(bool open) {
  if (open) {
    // Open the gripper
    servo(GRIPPER_OPEN);
  } else {
    // Close the gripper
    servo(GRIPPER_CLOSED);
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
  
  if (START) {
    lineFollower();
    Serial.println(analogRead(LINE_SENSOR[0]));
  }
  else {
    getDistance();

    for (int i = 0; i < 3; i++) {
      do {
        delay(100);
      } while (getDistance() < 15);
    }
    analogWrite(MOTOR_B_2, 255);
    analogWrite(MOTOR_A_2, 230);

    for (int i = 0; i < 6; i += 2) {
      while (analogRead(LINE_SENSOR[0]) > LINE && analogRead(LINE_SENSOR[6]) > LINE) {
        delay(1);
      }

      lineValues[i] = analogRead(LINE_SENSOR[3]);
      delay(200);
      
      analogWrite(MOTOR_B_2, 150);
      analogWrite(MOTOR_A_2, 130);
      while (analogRead(LINE_SENSOR[0]) < LINE && analogRead(LINE_SENSOR[6]) < LINE) {
        delay(1);
      }

      lineValues[i + 1] = analogRead(LINE_SENSOR[3]);
      delay(200);
      
      analogWrite(MOTOR_B_2, 150);
      analogWrite(MOTOR_A_2, 130);
    }
    analogWrite(MOTOR_B_2, 200);
    analogWrite(MOTOR_A_2, 180);
    delay(200);

    for (int i = 0; i < 8; i++) {
      servo(GRIPPER_CLOSED);
     }
    

    LINE = average(lineValues, 6) + 250;
    
    while (analogRead(LINE_SENSOR[4]) > LINE && analogRead(LINE_SENSOR[5]) > LINE) {
      delay(1);
    }
    while (analogRead(LINE_SENSOR[4]) < LINE) {
      delay(1);
    }
    if((lineCheck1 < LOWVALUE) && (lineCheck1 > HIGHVALUE) && (lineCheck2 < LOWVALUE) && (lineCheck2 > HIGHVALUE) && (lineCheck3 < LOWVALUE) && (lineCheck3 > HIGHVALUE) && (lineCheck4 < LOWVALUE) && (lineCheck4 > HIGHVALUE) &&
      (lineCheck5 < LOWVALUE) && (lineCheck5 > HIGHVALUE) && (lineCheck6 < LOWVALUE) && (lineCheck6 > HIGHVALUE) && (lineCheck7 < LOWVALUE) && (lineCheck7 > HIGHVALUE) && (lineCheck8 < LOWVALUE) && (lineCheck8 > HIGHVALUE))
    analogWrite(MOTOR_B_2, 0);
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_2, 220);
    analogWrite(MOTOR_A_1, 255);
    delay(500);

    do {
      analogWrite(MOTOR_B_2, 220);
      analogWrite(MOTOR_A_1, 255);
      if (analogRead(LINE_SENSOR[6]) > LINE) {
        break;
      }
    }
    while (true);
    START = true;
  }
}
