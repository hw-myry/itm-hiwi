#define STEP_PIN 2
#define DIR_PIN  1

#define STEPS_PER_REV 3200   // 1600/3200/6400

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
}

void stepMotor(int steps, bool dir) {
  digitalWrite(DIR_PIN, dir);

  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1000);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1000);
  }
}

void loop() {
  stepMotor(STEPS_PER_REV, HIGH);
  delay(1000);

  stepMotor(STEPS_PER_REV, LOW);
  delay(1000);
}