 #define   GRIPPER_PIN     9    // servo pin
#define   GRIPPER_OPEN    1800   // pulse length servo open
#define   GRIPPER_CLOSED  950    // pulse length servo closed
#define   SERVO_INTERVAL  20    // time between pulse
#define   GRIPPER_TOGGLE  1000  // toggle gripper every second

#define   MOTOR_LEFT_BACKWARD   11    // Motor Pin
#define   MOTOR_LEFT_FORWARD    10    // Motor Pin
#define   MOTOR_RIGHT_BACKWARD  6     // Motor Pin
#define   MOTOR_RIGHT_FORWARD   5     // Motor Pin 
 
const int LIGHT_SENSOR[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
#define   LIGHT_VALUE     850   // Light value at the beginning
bool BEGINNING = false;



void setup() {
  // put your setup code here, to run once:
  pinMode(GRIPPER_PIN, OUTPUT);
  digitalWrite(GRIPPER_PIN, LOW);
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
  servo(GRIPPER_OPEN);
  delay(1000);
  servo(GRIPPER_CLOSED);
  delay(1000);

  servo(GRIPPER_OPEN);
  delay(2000);
  servo(0);
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

void servo(int pulse) {
  static unsigned long timer;
  static int pulse1;
  if (pulse > 0) {
    pulse1 = pulse;
  }
  if (millis() > timer) {
    digitalWrite(GRIPPER_PIN, HIGH);
    delayMicroseconds(pulse1);
    digitalWrite(GRIPPER_PIN, LOW);
    timer = millis() + SERVO_INTERVAL;
  }
}
