#include <DHT.h>

// DHT11 Configuration
#define DHTPIN 2
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

// Pin Configuration
const int firing_pin = 3;    // TRIAC Gate control
const int zero_cross = 8;    // Zero crossing detector input
const int IR1_PIN = 6;       // First IR sensor
const int IR2_PIN = 7;       // Second IR sensor
const int INCREASE_BUTTON = 11; // Temperature increase button
const int DECREASE_BUTTON = 9;  // Temperature decrease button
const int LDR_PIN = A0;      // LDR sensor pin
const int RELAY_PIN = 10;    // Relay control pin
const int LDR_THRESHOLD = 700; // Adjust based on your light conditions

// Variables for 50Hz AC and Fan Control
int last_CH1_state = 0;
bool zero_cross_detected = false;
const int minimum_firing_delay = 100;
const int maximum_firing_delay = 10000;

// Timing variables
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
const int temp_read_Delay = 2000;
unsigned long lastSerialUpdate = 0;
const unsigned long serialUpdateInterval = 1000;

// Temperature variables
float real_temperature = 0;
float setpoint = 30.0;  // Initial setpoint
const float temp_adjustment = 0.5; // Temperature adjustment step

// People counting variables
volatile int peopleCount = 0;
bool ir1State = false;
bool ir2State = false;

// PID variables
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
int PID_value = 0;
int kp = 150;
int ki = 0.7;
int kd = 30;
int PID_p = 0;
int PID_i = 0;
int PID_d = 0;

// Hysteresis variables
const float temp_threshold = 1.0;
bool fan_at_minimum = false;

void setup() {
  pinMode(firing_pin, OUTPUT);
  pinMode(zero_cross, INPUT);
  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);
  pinMode(INCREASE_BUTTON, INPUT_PULLUP);
  pinMode(DECREASE_BUTTON, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);

  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);

  dht.begin();
  Serial.begin(9600);
}

void loop() {
  currentMillis = millis();

  // People counting logic
  updatePeopleCount();

  // Temperature setpoint adjustment
  handleTemperatureAdjustment();

  // LDR-based light control
  handleLightControl();

  // Temperature control loop
  if(currentMillis - previousMillis >= temp_read_Delay) {
    previousMillis += temp_read_Delay;

    if(peopleCount >= 1) {
      float newTemp = dht.readTemperature();

      if (!isnan(newTemp)) {
        real_temperature = newTemp;

        // PID computation
        PID_error = setpoint - real_temperature;

        if(abs(PID_error) > 8) {
          PID_i = 0;
        }

        PID_p = kp * PID_error;
        PID_i = PID_i + (ki * PID_error);

        timePrev = Time;
        Time = millis();
        elapsedTime = (Time - timePrev) / 1000;

        PID_d = kd * ((PID_error - previous_error) / elapsedTime);
        PID_value = PID_p + PID_i + PID_d;

        if(PID_error < 0) {
          PID_value = minimum_firing_delay;
          fan_at_minimum = false;
        } else {
          PID_value = map(constrain(PID_value, 0, maximum_firing_delay),
                         0, maximum_firing_delay,
                         maximum_firing_delay, minimum_firing_delay);

          if (PID_error < temp_threshold && !fan_at_minimum) {
            PID_value = maximum_firing_delay;
            fan_at_minimum = true;
          } else if (PID_error >= temp_threshold) {
            fan_at_minimum = false;
          }
        }

        PID_value = constrain(PID_value, minimum_firing_delay, maximum_firing_delay);
        previous_error = PID_error;
      }
    } else {
      PID_value = maximum_firing_delay;
      digitalWrite(firing_pin, LOW);
      delay(100);
    }

    printDebugInfo();
  }

  // TRIAC firing control
  if (zero_cross_detected && peopleCount >= 1) {
    delayMicroseconds(PID_value);
    digitalWrite(firing_pin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(firing_pin, LOW);
    zero_cross_detected = false;
  }
}

void updatePeopleCount() {
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
}

void handleTemperatureAdjustment() {
  if (digitalRead(INCREASE_BUTTON) == HIGH) {
    interrupts();
    digitalWrite(firing_pin, LOW);
    setpoint += temp_adjustment;
    delay(200);
  }

  if (digitalRead(DECREASE_BUTTON) == HIGH) {
    interrupts();
    digitalWrite(firing_pin, LOW);
    setpoint -= temp_adjustment;
    delay(200);
  }

  setpoint = constrain(setpoint, 10, 40);
}

void handleLightControl() {
  int ldrValue = analogRead(LDR_PIN);
  if (peopleCount >= 1 && ldrValue < LDR_THRESHOLD) {
    digitalWrite(RELAY_PIN, HIGH);
  } else {
    digitalWrite(RELAY_PIN, LOW);
  }
}

void printDebugInfo() {
  if (millis() - lastSerialUpdate >= serialUpdateInterval) {
    Serial.print("Setpoint: ");
    Serial.println(setpoint);
    Serial.print("Current Temperature: ");
    Serial.println(real_temperature);
    Serial.print("People Count: ");
    Serial.println(peopleCount);
    Serial.print("Firing Delay: ");
    Serial.println(PID_value);
    Serial.print("LDR Value: ");
    Serial.println(analogRead(LDR_PIN));
    Serial.println("-------------------");

    lastSerialUpdate = millis();
  }
}

// Zero crossing interrupt handler
ISR(PCINT0_vect) {
  if(PINB & B00000001) {
    if(last_CH1_state == 0) {
      zero_cross_detected = true;
    }
    last_CH1_state = 1;
  }
  else if(last_CH1_state == 1) {
    zero_cross_detected = true;
    last_CH1_state = 0;
  }
}
