#define   GRIPPER_PIN     6      // servo pin  
#define   GRIPPER_OPEN    1600   // pulse length servo open
#define   GRIPPER_CLOSED  930    // pulse length servo closed
#define   SERVO_INTERVAL  20    // time between pulse
#define   GRIPPER_TOGGLE  1000  // toggle gripper every second

void setup() {
  // put your setup code here, to run once:
  pinMode(GRIPPER_PIN, OUTPUT);
  digitalWrite(GRIPPER_PIN, LOW);
  servo(GRIPPER_OPEN);
}

void loop() {
  // put your main code here, to run repeatedly:
  servo(GRIPPER_OPEN);
  delay(1000);
  servo(GRIPPER_CLOSED);
  delay(1000);

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