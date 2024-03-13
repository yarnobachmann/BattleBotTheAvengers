


#define TRIG_PIN 8
#define ECHO_PIN 9
#define motorA1 11 // Links Achteruit
#define motorA2 10 // Links Vooruit
#define motorB1 6  // Rechts Achteruit
#define motorB2 5  // Rechts Vooruit
const int snelheid = 210;
const int afwijking = 10;

unsigned long previousMillis = 0;
const long interval = 100; // Interval in milliseconden
bool obstacleDetected = false; // Variabele om bij te houden of een obstakel is gedetecteerd

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
}

void loop() {
  unsigned long currentMillis = millis();
  long distance; // Declaratie van de afstand
  
  if (currentMillis - previousMillis >= interval) {
    // Opslaan van het huidige tijdstip
    previousMillis = currentMillis;
    
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH);
    distance = duration * 0.034 / 2; // Bereken de afstand in centimeters
    
    if (distance < 25) { // Als de afstand minder dan 10 cm is
      obstacleDetected = true; // Obstakel gedetecteerd
      vooruit(); // Rijd vooruit
    } else {
      if (obstacleDetected) {
        // Blijf rijden
        vooruit();
      } else {
        // Stop
        analogWrite(motorA1, 0); 
        analogWrite(motorA2, 0); 
        analogWrite(motorB1, 0); 
        analogWrite(motorB2, 0);
      }
    }
  }
  
  Serial.println(distance);
}

void vooruit() {
  analogWrite(motorA1, 0);
  analogWrite(motorA2, snelheid - afwijking);
  analogWrite(motorB1, 0);
  analogWrite(motorB2, snelheid);
}
