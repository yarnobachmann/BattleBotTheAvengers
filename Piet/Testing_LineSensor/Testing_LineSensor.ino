#define   MOTOR_LEFT_BACKWARD   11    // Motor Pin
#define   MOTOR_LEFT_FORWARD    10    // Motor Pin
#define   MOTOR_RIGHT_BACKWARD  6     // Motor Pin
#define   MOTOR_RIGHT_FORWARD   5     // Motor Pin 
#define   MOTOR_AFWIJKING       6     // Afwijking van de motor (moet nog in het Engels) 
 
const int LIGHT_SENSOR[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
#define   LIGHT_VALUE     850   // Light value at the beginning

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
  for (int i = 0; i < 8; i++)
  {
    followTheLine();
  }
}

void followTheLine()
{
  if (analogRead(LIGHT_SENSOR[7]) > LIGHT_VALUE || analogRead(LIGHT_SENSOR[3]) > LIGHT_VALUE && analogRead(LIGHT_SENSOR[4]) > LIGHT_VALUE)
  {
    turnLeft(245); 
  }
  else if (analogRead(LIGHT_SENSOR[3]) > LIGHT_VALUE || analogRead(LIGHT_SENSOR[4]) > LIGHT_VALUE)
  {
    driveForward(255);
  }
  else if (analogRead(LIGHT_SENSOR[5]) > LIGHT_VALUE || analogRead(LIGHT_SENSOR[6]) > LIGHT_VALUE)
  {
    turnLeft(160);
  }
  else if (analogRead(LIGHT_SENSOR[1]) > LIGHT_VALUE || analogRead(LIGHT_SENSOR[2]) > LIGHT_VALUE)
  {
    turnRight(160);
  }
  else if (analogRead(LIGHT_SENSOR[8]) < LIGHT_VALUE)
  {
    turnRight(245);
  }
  else if (analogRead(LIGHT_SENSOR[3]) > LIGHT_VALUE && analogRead(LIGHT_SENSOR[4]) > LIGHT_VALUE || analogRead(LIGHT_SENSOR [0]) > LIGHT_VALUE)
  {
    turnRight(245);
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
  analogWrite(MOTOR_LEFT_BACKWARD, 255);
  analogWrite(MOTOR_LEFT_FORWARD, 0);
  analogWrite(MOTOR_RIGHT_BACKWARD, 0);
  analogWrite(MOTOR_RIGHT_FORWARD, 255);

  delay(10);
  
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

  delay(10);

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
