
const int numSensors = 8;
const int sensorPins[numSensors] = {A0, A1, A2, A3, A4, A5, A6, A7};
unsigned int sensorValues[numSensors];


// Define motor pins and speed
const int motorA1 = 11; // Links Achteruit
const int motorA2 = 10; // Links Vooruit
const int motorB1 = 6; // Rechts Achteruit
const int motorB2 = 5; //Rechts Vooruit

unsigned long previousMillis = 0;
const long draaiTijd = 1000;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);

  // Initialize motor pins as output
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

}

void loop() {

  for (int i = 0; i < numSensors; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }
      Serial.print("Sensor 0: ");
      Serial.println(sensorValues[0]);
      Serial.print("Sensor 1: ");
      Serial.println(sensorValues[1]);
      Serial.print("Sensor 2: ");
      Serial.println(sensorValues[2]);
      Serial.print("Sensor 3: ");
      Serial.println(sensorValues[3]);
      Serial.print("Sensor 4: ");
      Serial.println(sensorValues[4]);
      Serial.print("Sensor 5: ");
      Serial.println(sensorValues[5]);
      Serial.print("Sensor 6: ");
      Serial.println(sensorValues[6]);
      Serial.print("Sensor 7: ");
      Serial.println(sensorValues[7]);


  
  // put your main code here, to run repeatedly:

if (sensorValues[1] > 900 || sensorValues[2] > 900 || sensorValues[3] > 900 || sensorValues[4] > 900 || sensorValues[5] > 900 || sensorValues[6] > 900){
  if (sensorValues[3] > 900 || sensorValues[4] > 900) {
    analogWrite(motorA1, 0);
    analogWrite(motorA2, 205);
    analogWrite(motorB1, 0);
    analogWrite(motorB2, 210);
  } else if (sensorValues[0] > 900 || sensorValues[1] > 900) {
    analogWrite(motorA1, 0);
    analogWrite(motorA2, 180);
    analogWrite(motorB1, 0);
    analogWrite(motorB2, 200);  
  } else if (sensorValues[2] > 900) {
    analogWrite(motorA1, 0);
    analogWrite(motorA2, 190);
    analogWrite(motorB1, 0);
    analogWrite(motorB2, 210);  
  } else if (sensorValues[5] > 900) {
    analogWrite(motorA1, 0);
    analogWrite(motorA2, 190);
    analogWrite(motorB1, 0);
    analogWrite(motorB2, 230);
  } else if (sensorValues[6] > 900 || sensorValues[7] > 900) {
    analogWrite(motorA1, 0);
    analogWrite(motorA2, 180);
    analogWrite(motorB1, 0);
    analogWrite(motorB2, 220); 
  } 
 }
 
    analogWrite(motorA1, 0);
    analogWrite(motorA2, 180);
    analogWrite(motorB1, 180);
    analogWrite(motorB2, 0); 
}
