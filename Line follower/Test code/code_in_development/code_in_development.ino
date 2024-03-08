#include <Adafruit_NeoPixel.h>

const int NEO_PIXEL_PIN = 7; // neopixel pin
const int NEO_PIXEL_COUNT = 4; // amount of neopixel lights
Adafruit_NeoPixel pixels(NEO_PIXEL_COUNT, NEO_PIXEL_PIN, NEO_GRB + NEO_KHZ800);

const int MOTOR_A_1 = 11; // Left wheel backwards
const int MOTOR_A_2 = 10; // Left wheel forwards
const int MOTOR_B_1 = 6; // Right wheel backwards
const int MOTOR_B_2 = 5; // Right wheel forwards

const int gripperPin = 9;
const int openPosition = 35;
const int closedPosition = 110;

const int trigPin = 4;
const int echoPin = 8;

const int LOWVALUE = 700;
const int LOWVALUEUNIQUE = 600;
const int HIGHVALUE = 1000;

const float DISTANCE_THRESHOLD = 15.0;

const int LINESENSOR_1 = A0;
const int LINESENSOR_2 = A1;
const int LINESENSOR_3 = A2;
const int LINESENSOR_4 = A3;
const int LINESENSOR_5 = A4;
const int LINESENSOR_6 = A5;
const int LINESENSOR_7 = A6;
const int LINESENSOR_8 = A7;

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

unsigned long previousMillis = 0;
const long interval = 250;



void setup() {
  Serial.begin(9600);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  pinMode(MOTOR_A_1, OUTPUT);
  pinMode(MOTOR_A_2, OUTPUT);
  pinMode(MOTOR_B_1, OUTPUT);
  pinMode(MOTOR_B_2, OUTPUT);



  pinMode(LINESENSOR_1, INPUT);
  pinMode(LINESENSOR_2, INPUT);
  pinMode(LINESENSOR_3, INPUT);
  pinMode(LINESENSOR_4, INPUT);
  pinMode(LINESENSOR_5, INPUT);
  pinMode(LINESENSOR_6, INPUT);
  pinMode(LINESENSOR_7, INPUT);
  pinMode(LINESENSOR_8, INPUT);


  
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

void loop() {
  lineCheck1 = analogRead(LINESENSOR_1);
  lineCheck2 = analogRead(LINESENSOR_2);
  lineCheck3 = analogRead(LINESENSOR_3);
  lineCheck4 = analogRead(LINESENSOR_4);
  lineCheck5 = analogRead(LINESENSOR_5);
  lineCheck6 = analogRead(LINESENSOR_6);
  lineCheck7 = analogRead(LINESENSOR_7);
  lineCheck8 = analogRead(LINESENSOR_8);

  digitalWrite(trigPin, LOW);
  delay(250);

  digitalWrite(trigPin, HIGH);
  delay(250);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration * 0.0343) / 2;

  Serial.print("Afstand in cm: ");
  Serial.println(distance);
  
  if((lineCheck1 > LOWVALUE) && (lineCheck1 < HIGHVALUE) && (lineCheck2 > LOWVALUE) && (lineCheck2 < HIGHVALUE) && (lineCheck3 > LOWVALUE) && (lineCheck3 < HIGHVALUE) && (lineCheck4 > LOWVALUE) && (lineCheck4 < HIGHVALUE) &&
  (lineCheck5 > LOWVALUE) && (lineCheck5< HIGHVALUE) && (lineCheck6 > LOWVALUE) && (lineCheck6 < HIGHVALUE) && (lineCheck7 > LOWVALUE) && (lineCheck7 < HIGHVALUE) && (lineCheck8 > LOWVALUE) && (lineCheck8 < HIGHVALUE)){
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
  else if((lineCheck1 > LOWVALUE) && (lineCheck1 < HIGHVALUE) && (lineCheck2 > LOWVALUE) && (lineCheck2 < HIGHVALUE) && (lineCheck3 > LOWVALUE) && (lineCheck3 < HIGHVALUE) && (lineCheck4 > LOWVALUE) && (lineCheck4 < HIGHVALUE) && (lineCheck5 > LOWVALUE) && (lineCheck5 < HIGHVALUE)){
    analogWrite(MOTOR_A_2, 255);
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_B_2, 255);
    startTime = 0;
    colors();
    delay(100);
  }
  else if((lineCheck4 > LOWVALUE) && (lineCheck4 < HIGHVALUE) && (lineCheck5 > LOWVALUE) && (lineCheck5 < HIGHVALUE) && (lineCheck6 > LOWVALUE) && (lineCheck6 < HIGHVALUE) && (lineCheck7 > LOWVALUE) && (lineCheck7 < HIGHVALUE) && (lineCheck8 > LOWVALUE) && (lineCheck8 < HIGHVALUE)){
    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, 255);
    analogWrite(MOTOR_A_2, 255);
    startTime = 0;
    colors();
    delay(100);
  }
  else if((lineCheck1 > LOWVALUE) && (lineCheck1 < HIGHVALUE)){
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_2, 220);
    analogWrite(MOTOR_A_1, 255);
    analogWrite(MOTOR_B_1, 0);
    startTime = 0;
    colors();
  }
  else if((lineCheck2 > LOWVALUE) && (lineCheck2 < HIGHVALUE)){
    analogWrite(MOTOR_A_2, 80);
    analogWrite(MOTOR_B_2, 255);
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_B_1, 0);
    startTime = 0;
    colors();
  }
  else if((lineCheck3 > LOWVALUE) && (lineCheck3 < HIGHVALUE)){
    analogWrite(MOTOR_A_2, 160);
    analogWrite(MOTOR_B_2, 255);
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_B_1, 0);
    startTime = 0;
    colors();
  }
  else if(((lineCheck4 > LOWVALUE) && (lineCheck4 < HIGHVALUE)) || ((lineCheck5 > LOWVALUE) && (lineCheck5 < HIGHVALUE))){
    analogWrite(MOTOR_A_2, 255);
    analogWrite(MOTOR_B_2, 255);
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_B_1, 0);
    startTime = 0;
    colors();
  }
  else if((lineCheck6 > LOWVALUEUNIQUE) && (lineCheck6 < HIGHVALUE)){
    analogWrite(MOTOR_B_2, 160);
    analogWrite(MOTOR_A_2, 255);
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_B_1, 0);
    startTime = 0;
    colors();
  }
  else if((lineCheck7 > LOWVALUE) && (lineCheck7 < HIGHVALUE)){
    analogWrite(MOTOR_B_2, 80);
    analogWrite(MOTOR_A_2, 255);
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_B_1, 0);
    startTime = 0;
    colors();
  }
  else if((lineCheck8 > LOWVALUE) && (lineCheck8 < HIGHVALUE)){
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
