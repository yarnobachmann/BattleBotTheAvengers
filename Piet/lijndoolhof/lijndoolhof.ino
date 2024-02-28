const uint8_t numSensors = 8;
const uint8_t sensorPins[numSensors] = {A0, A1, A2, A3, A4, A5, A6, A7};
unsigned int sensorValues[numSensors];
bool lijnGevonden = false;

// Define motor pins and speed
const int motorA1 = 11; // Links Achteruit
const int motorA2 = 10; // Links Vooruit
const int motorB1 = 6; // Rechts Achteruit
const int motorB2 = 5; //Rechts Vooruit

const int snelheid = 210;
const int afwijking = 10;

unsigned long previousMillis = 0;
const long draaiTijd = 1500;

void setup() {
  Serial.begin(9600);

  // Initialize motor pins as output
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
}

void loop() {
  unsigned long totalSensorValue = 0; // Variabele om de totale sensorwaarde bij te houden

  for (uint8_t i = 0; i < numSensors; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    totalSensorValue += sensorValues[i]; // Optellen van de huidige sensorwaarde aan het totaal
  }

  if ((sensorValues[3] > 850 || sensorValues[4] > 850) && sensorValues[0] < 800 && sensorValues[7] < 800) {
    lijnGevonden = true;
  } else lijnGevonden = false;

  if (lijnGevonden) {
    vooruit();
  } else {
    if ((sensorValues[3] > 850 || sensorValues[4] > 850) && sensorValues[0] > 850 && sensorValues[7] < 800) {
      draaiRechts();
    } else if ((sensorValues[3] > 850 || sensorValues[4] > 850) && sensorValues[0] < 800 && sensorValues[7] > 850){
      draaiLinks();
    }
    unsigned long currentMillis = millis();
      if (currentMillis - previousMillis < draaiTijd) {
      draaiLinks();
    } else if (currentMillis - previousMillis < 2 * draaiTijd) {
      draaiRechts();
    } else {
      previousMillis = currentMillis;
    }
  }
  }


void vooruit(){ 
    analogWrite(motorA1, 0);
    analogWrite(motorA2, snelheid - afwijking);
    analogWrite(motorB1, 0);
    analogWrite(motorB2, snelheid);
}

void draaiLinks(){
    analogWrite(motorA1, snelheid - afwijking);
    analogWrite(motorA2, 0);
    analogWrite(motorB1, 0);
    analogWrite(motorB2, snelheid);
}
void draaiRechts(){
    analogWrite(motorA1, 0);
    analogWrite(motorA2, snelheid - afwijking);
    analogWrite(motorB1, snelheid);
    analogWrite(motorB2, 0);
}
