const int openPos = 120;   // Open position in degrees
const int servo = 12;


void setup() {
  pinMode(servo, OUTPUT);  // Set servo pin as an output
  writeServo(0);
  delay(500);
}

void loop() {
  // Your main loop code here
  for (int pos = 0; pos <= openPos; pos++) {
    writeServo(pos);
    delay(40);
  }

  for (int pos = openPos; pos >= 0; pos--) {
    writeServo(pos);
    delay(50);
  }
}

void writeServo(int angle) {
  int servoAngle = map(angle, 0, openPos, 0, 180);
  analogWrite(servo, servoAngle);
}
