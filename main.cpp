//-----------------------------------[Include libraries]----------------------------

#include <Arduino.h>

#include <Adafruit_NeoPixel.h>

#ifdef __AVR__#include <avr/power.h>

#endif

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

//-----------------------------------[Declare pins]-------------------------------

#define PIN 13 // Digital pin connected to the Neopixel
#define NUMPIXELS 4 // Number of Neopixels in your strip

#define BUZZER A5 // Buzzer pin

#define MOTOR_LEFT_BACKWARD 12 // Motor pin A1
#define MOTOR_LEFT_FORWARD 11 // Motor pin A2
#define MOTOR_RIGHT_FORWARD 10 // Motor pin B1
#define MOTOR_RIGHT_BACKWARD 9 // Motor pin B2
#define MOTOR_RIGHT_READ 2 // Motor pin B1
#define MOTOR_RIGHT_READ 3// Motor pin B2

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

#define NOTE_B0 31
#define NOTE_C1 33
#define NOTE_CS1 35
#define NOTE_D1 37
#define NOTE_DS1 39
#define NOTE_E1 41
#define NOTE_F1 44
#define NOTE_FS1 46
#define NOTE_G1 49
#define NOTE_GS1 52
#define NOTE_A1 55
#define NOTE_AS1 58
#define NOTE_B1 62
#define NOTE_C2 65
#define NOTE_CS2 69
#define NOTE_D2 73
#define NOTE_DS2 78
#define NOTE_E2 82
#define NOTE_F2 87
#define NOTE_FS2 93
#define NOTE_G2 98
#define NOTE_GS2 104
#define NOTE_A2 110
#define NOTE_AS2 117
#define NOTE_B2 123
#define NOTE_C3 131
#define NOTE_CS3 139
#define NOTE_D3 147
#define NOTE_DS3 156
#define NOTE_E3 165
#define NOTE_F3 175
#define NOTE_FS3 185
#define NOTE_G3 196
#define NOTE_GS3 208
#define NOTE_A3 220
#define NOTE_AS3 233
#define NOTE_B3 247
#define NOTE_C4 262
#define NOTE_CS4 277
#define NOTE_D4 294
#define NOTE_DS4 311
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_FS4 370
#define NOTE_G4 392
#define NOTE_GS4 415
#define NOTE_A4 440
#define NOTE_AS4 466
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_CS5 554
#define NOTE_D5 587
#define NOTE_DS5 622
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_FS5 740
#define NOTE_G5 784
#define NOTE_GS5 831
#define NOTE_A5 880
#define NOTE_AS5 932
#define NOTE_B5 988
#define NOTE_C6 1047
#define NOTE_CS6 1109
#define NOTE_D6 1175
#define NOTE_DS6 1245
#define NOTE_E6 1319
#define NOTE_F6 1397
#define NOTE_FS6 1480
#define NOTE_G6 1568
#define NOTE_GS6 1661
#define NOTE_A6 1760
#define NOTE_AS6 1865
#define NOTE_B6 1976
#define NOTE_C7 2093
#define NOTE_CS7 2217
#define NOTE_D7 2349
#define NOTE_DS7 2489
#define NOTE_E7 2637
#define NOTE_F7 2794
#define NOTE_FS7 2960
#define NOTE_G7 3136
#define NOTE_GS7 3322
#define NOTE_A7 3520
#define NOTE_AS7 3729
#define NOTE_B7 3951
#define NOTE_C8 4186
#define NOTE_CS8 4435
#define NOTE_D8 4699
#define NOTE_DS8 4978
#define REST 0

const int numberOfSensors = 6; // Number of sensors used
int sensorPins[numberOfSensors] = {
  A1,A6,A7,A3,A2,A0
}; // Analog pins for the sensors      
int sensorValues[numberOfSensors]; // Array to store the sensor values
#define SENSOR_INTERVAL 20 // time between pulse

const int BLACK = 900;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_RGB + NEO_KHZ800);

//-----------------------------------[Variables]---------------------------------

unsigned long startTime; // Variable to store the start time for calibration
const int calibrationDuration = 5000; // Calibration duration in milliseconds
bool calibrated = false; // Flag to check if calibration is done

const int stopDistance = 11; // Distance threshold to stop the robot (in cm)
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


//-----------------------------------[Setup function]---------------------------

void setup() {
  strip.begin(); // Initialize the Neopixel strip
  strip.show(); // Initialize all pixels to 'off'
  // Motor pins
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  // Echo sensor pins
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIGGER_PIN_RIGHT, OUTPUT);
  pinMode(ECHO_PIN_RIGHT, INPUT);
  // Servo pins
  pinMode(GRIPPER_PIN, OUTPUT);
  digitalWrite(GRIPPER_PIN, LOW);
  // Reflection sensor pins
  for (int i = 0; i <= numberOfSensors; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  Serial.begin(9600);
  // channel();
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

  if (timeToStartDetectingFinish < millis())
  {
    detectFinish();
  }

  if (!startTrigger)
  {

    startTrigger = true;
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
  if(sensorValues[5] >= BLACK || sensorValues[0] >= BLACK)
  {
    while(isOnLightColor())
    {
      readLineSensor();
    }  
    dropCone();
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
  int interval = 200;
  if (millis() > timer) {
    distanceReader();
    if (distance <= stopDistance) {
      continueRightSensor = false;
      tone(BUZZER, 1000, 100); 
      setMotors(0, leftSpeed, rightSpeed, 0);
    } 
    else
    {
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
  static unsigned long timer;

  int leftSpeed = 230;
  int rightSpeed = 230;

  // Limit the motor speeds to the valid range (0-255)
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  distanceReaderRight();
  
  int difference = distanceRight - minRightDistance;
  Serial.print("Verschil: ");
  Serial.println(difference);
  if (distanceRight >= minRightDistance && difference <= 10) {
    setMotors(leftSpeed, 0, rightSpeed - 100, 0);
  } 
  else if (difference >= 8)
  {
    setMotors(leftSpeed, 0, rightSpeed, 0);
    delay(100);
    setMotors(leftSpeed, 0, 0, rightSpeed);
    delay(50);
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
    timer = millis() + 100;
  }
  return false;
}

//-----------------------------------[Speaker]--------------------------------------

void defaultSpeakerValues(int tempo, int melody[], int melodyLength) {
  // Calculate the number of notes based on the length of the melody array
  int notes = melodyLength / 2;

  // Calculate the duration of a whole note in milliseconds
  int wholenote = (60000 * 4) / tempo;

  int divider = 0, noteDuration = 0;

  for (int thisNote = 0; thisNote < notes * 2; thisNote += 2) {
    // Calculate the duration of each note
    divider = melody[thisNote + 1];
    if (divider > 0) {
      // Regular note, just proceed
      noteDuration = wholenote / divider;
    } else if (divider < 0) {
      // Dotted notes are represented with negative durations
      noteDuration = wholenote / abs(divider);
      noteDuration *= 1.5; // Increase the duration in half for dotted notes
    }

    // Play the note for 90% of the duration, leaving 10% as a pause
    tone(BUZZER, melody[thisNote], noteDuration * 0.9);

    // Wait for the specified duration before playing the next note
    delay(noteDuration);

    // Stop the waveform generation before the next note
    noTone(BUZZER);
  }
}

void channel() {
  int tempo = 114;

  // notes of the moledy followed by the duration.
  // a 4 means a quarter note, 8 an eighteenth , 16 sixteenth, so on
  // !!negative numbers are used to represent dotted notes,
  // so -4 means a dotted quarter note, that is, a quarter plus an eighteenth!!
  int melody[] = {

  // Pink Panther theme
  // Score available at https://musescore.com/benedictsong/the-pink-panther
  // Theme by Masato Nakamura, arranged by Teddy Mason

  NOTE_FS4,8, REST,8, NOTE_A4,8, NOTE_CS5,8, REST,8,NOTE_A4,8, REST,8, NOTE_FS4,8, //1
  NOTE_D4,8, NOTE_D4,8, NOTE_D4,8, REST,8, REST,4, REST,8, NOTE_CS4,8,
  NOTE_D4,8, NOTE_FS4,8, NOTE_A4,8, NOTE_CS5,8, REST,8, NOTE_A4,8, REST,8, NOTE_F4,8,
  NOTE_E5,-4, NOTE_DS5,8, NOTE_D5,8, REST,8, REST,4,
  
  NOTE_GS4,8, REST,8, NOTE_CS5,8, NOTE_FS4,8, REST,8,NOTE_CS5,8, REST,8, NOTE_GS4,8, //5
  REST,8, NOTE_CS5,8, NOTE_G4,8, NOTE_FS4,8, REST,8, NOTE_E4,8, REST,8,
  NOTE_E4,8, NOTE_E4,8, NOTE_E4,8, REST,8, REST,4, NOTE_E4,8, NOTE_E4,8,
  NOTE_E4,8, REST,8, REST,4, NOTE_DS4,8, NOTE_D4,8, 

  NOTE_CS4,8, REST,8, NOTE_A4,8, NOTE_CS5,8, REST,8,NOTE_A4,8, REST,8, NOTE_FS4,8, //9
  NOTE_D4,8, NOTE_D4,8, NOTE_D4,8, REST,8, NOTE_E5,8, NOTE_E5,8, NOTE_E5,8, REST,8,
  REST,8, NOTE_FS4,8, NOTE_A4,8, NOTE_CS5,8, REST,8, NOTE_A4,8, REST,8, NOTE_F4,8,
  NOTE_E5,2, NOTE_D5,8, REST,8, REST,4,

  NOTE_B4,8, NOTE_G4,8, NOTE_D4,8, NOTE_CS4,4, NOTE_B4,8, NOTE_G4,8, NOTE_CS4,8, //13
  NOTE_A4,8, NOTE_FS4,8, NOTE_C4,8, NOTE_B3,4, NOTE_F4,8, NOTE_D4,8, NOTE_B3,8,
  NOTE_E4,8, NOTE_E4,8, NOTE_E4,8, REST,4, REST,4, NOTE_AS4,4,
  NOTE_CS5,8, NOTE_D5,8, NOTE_FS5,8, NOTE_A5,8, REST,8, REST,4, 
};

  int melodyLength = sizeof(melody) / sizeof(melody[0]);

  defaultSpeakerValues(tempo, melody, melodyLength);

}