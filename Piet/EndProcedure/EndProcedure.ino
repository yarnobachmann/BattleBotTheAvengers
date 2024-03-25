#define   GRIPPER_PIN     12      // servo pin  
#define   GRIPPER_OPEN    1800   // pulse length servo open
#define   GRIPPER_CLOSED  1100    // pulse length servo closed
#define   SERVO_INTERVAL  20    // time between pulse
#define   GRIPPER_TOGGLE  1000  // toggle gripper every second

#define   MOTOR_LEFT_BACKWARD   11    // Motor Pin
#define   MOTOR_LEFT_FORWARD    10    // Motor Pin
#define   MOTOR_RIGHT_BACKWARD  6     // Motor Pin
#define   MOTOR_RIGHT_FORWARD   5     // Motor Pin 
#define   MOTOR_AFWIJKING       6     // Afwijking van de motor (moet nog in het Engels) 
#define   MOTOR_AFWIJKING_LAAG  10

const int LIGHT_SENSOR[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
#define   LIGHT_VALUE     850   // Light value at the beginning

unsigned long endingTimer = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);

  for (int i = 0; i < 8; i++)
  {
    pinMode(LIGHT_SENSOR[i], INPUT);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (shouldEnd())
  {
    theEnd();
  }
  else
  {
    followTheLine();
  }
}

bool shouldEnd()
{
  int blackDetected = 0;
  for (int i = 0; i < 8; i++)
  {
    if (analogRead(LIGHT_SENSOR[i]) > LIGHT_VALUE)
    {
      blackDetected++;
    }
  }

  if (blackDetected >= 5)
  {
    if (endingTimer == 0)
    {
      endingTimer = millis();
    }
    else
    {
      if (millis() - endingTimer >= 500)
      {
        return true;
      }
    }
  }
  else
  {
    endingTimer = 0;
  }

  return false;
 }

void followTheLine()
{
  if (analogRead(LIGHT_SENSOR[7]) > LIGHT_VALUE || analogRead(LIGHT_SENSOR[6]) > LIGHT_VALUE || analogRead(LIGHT_SENSOR[7]) > LIGHT_VALUE && analogRead(LIGHT_SENSOR[0]) < LIGHT_VALUE)
  {
    turnLeft(245); 
  }
  else if (analogRead(LIGHT_SENSOR[3]) > LIGHT_VALUE || analogRead(LIGHT_SENSOR[4]) > LIGHT_VALUE)
  {
    driveForward(210);
  }
  else if (analogRead(LIGHT_SENSOR[5]) > LIGHT_VALUE || analogRead(LIGHT_SENSOR[6]) > LIGHT_VALUE)
  {
    turnLeft(200);
  }
  else if (analogRead(LIGHT_SENSOR[0]) > LIGHT_VALUE || analogRead(LIGHT_SENSOR[1]) > LIGHT_VALUE || analogRead(LIGHT_SENSOR[0]) > LIGHT_VALUE && analogRead(LIGHT_SENSOR[7]) < LIGHT_VALUE)
  {
    turnRight(200);
  }
  else if (analogRead(LIGHT_SENSOR[8]) < LIGHT_VALUE)
  {
    turnRight(245);
  }
}

void theEnd()
{
  motorStop();
  for (int i = 0; i < 8; i++)
  {
    servo(GRIPPER_OPEN);
  }
  delay(1000);
  driveBackwards(210);
  delay(1000);
  motorStop();
  delay(1000);
}

void driveForward(int speed)
{
  analogWrite(MOTOR_LEFT_BACKWARD, 0);
  analogWrite(MOTOR_LEFT_FORWARD, speed - MOTOR_AFWIJKING_LAAG);
  analogWrite(MOTOR_RIGHT_BACKWARD, 0);
  analogWrite(MOTOR_RIGHT_FORWARD, speed);
}

void turnLeft(int speed)
{
  analogWrite(MOTOR_LEFT_BACKWARD, 255);
  analogWrite(MOTOR_LEFT_FORWARD, 0);
  analogWrite(MOTOR_RIGHT_BACKWARD, 0);
  analogWrite(MOTOR_RIGHT_FORWARD, 255);

  delayMicroseconds(10);
  
  analogWrite(MOTOR_LEFT_BACKWARD, speed - MOTOR_AFWIJKING);
  analogWrite(MOTOR_LEFT_FORWARD, 0);
  analogWrite(MOTOR_RIGHT_BACKWARD, 0);
  analogWrite(MOTOR_RIGHT_FORWARD, speed);
}

void turnRight(int speed)
{
  analogWrite(MOTOR_LEFT_BACKWARD, 0);
  analogWrite(MOTOR_LEFT_FORWARD, 255);
  analogWrite(MOTOR_RIGHT_BACKWARD, 255);
  analogWrite(MOTOR_RIGHT_FORWARD, 0);

  delayMicroseconds(10);

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
