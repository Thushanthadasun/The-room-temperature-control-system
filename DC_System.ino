#include <DHT.h>
#include <PID_v1.h>

// Pin definitions
#define DHTPIN 2
#define ENA 5
#define IN1 3
#define IN2 4
#define DHTTYPE DHT11
#define IR1_PIN 6
#define IR2_PIN 7
#define INCREASE_BUTTON 11
#define DECREASE_BUTTON 9
#define LDR_PIN A0       // LDR sensor pin
#define RELAY_PIN 10     // Relay control pin
#define LDR_THRESHOLD 700 // Adjust based on your light conditions

DHT dht(DHTPIN, DHTTYPE);

double setpoint = 29.2;
double input, output;
double Kp = 2, Ki = 5, Kd = 3;

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, REVERSE);

volatile int peopleCount = 0;
bool ir1State = false;
bool ir2State = false;

unsigned long lastSerialUpdate = 0;
const unsigned long serialUpdateInterval = 1000;

void setup() {
  Serial.begin(9600);
  dht.begin();

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);
  pinMode(INCREASE_BUTTON, INPUT);
  pinMode(DECREASE_BUTTON, INPUT);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(40, 255);
}

void loop() {
  bool ir1Triggered = digitalRead(IR1_PIN) == LOW;
  bool ir2Triggered = digitalRead(IR2_PIN) == LOW;

  if (ir1Triggered && !ir1State) {
    ir1State = true;
  } else if (!ir1Triggered && ir1State) {
    ir1State = false;
    if (ir2Triggered) peopleCount++;
  }

  if (ir2Triggered && !ir2State) {
    ir2State = true;
  } else if (!ir2Triggered && ir2State) {
    ir2State = false;
    if (ir1Triggered && peopleCount > 0) peopleCount--;
  }

  int toggleUpState = digitalRead(INCREASE_BUTTON);
  int toggleDownState = digitalRead(DECREASE_BUTTON);

  if (toggleUpState == LOW) {
    setpoint += 0.5;
    delay(200);
  }

  if (toggleDownState == LOW) {
    setpoint -= 0.5;
    delay(200);
  }

  if (setpoint < 10) setpoint = 10;
  if (setpoint > 40) setpoint = 40;

  // Read LDR value and control relay
  int ldrValue = analogRead(LDR_PIN);
  if (peopleCount >= 1 && ldrValue < LDR_THRESHOLD) {
    digitalWrite(RELAY_PIN, HIGH);
  } else {
    digitalWrite(RELAY_PIN, LOW);
  }

  if (peopleCount >= 1) {
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    if (isnan(temperature) || isnan(humidity)) {
      if (millis() - lastSerialUpdate >= serialUpdateInterval) {
        Serial.println("Failed to read from DHT sensor!");
        lastSerialUpdate = millis();
      }
      return;
    }

    input = temperature;
    myPID.Compute();
    analogWrite(ENA, output);
  } else {
    analogWrite(ENA, 0);
  }

  if (millis() - lastSerialUpdate >= serialUpdateInterval) {
    Serial.print("Setpoint: ");
    Serial.println(setpoint);
    Serial.print("People count: ");
    Serial.println(peopleCount);
    Serial.print("LDR Value: ");
    Serial.println(ldrValue);
    if (peopleCount >= 1) {
      Serial.print("Temperature: ");
      Serial.println(input);
      Serial.print("PID Output: ");
      Serial.println(output);
    } else {
      Serial.println("Temperature control inactive: Room is empty.");
    }
    lastSerialUpdate = millis();
  }

  delay(100);
}
