#define PIN_A 2
#define PIN_B 42
#define PIN_C 41

void setup() {
  Serial.begin(115200);

  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);
  pinMode(PIN_C, INPUT_PULLUP);
}

void loop() {
  Serial.print("GPIO2=");
  Serial.print(digitalRead(PIN_A));

  Serial.print(" GPIO42=");
  Serial.print(digitalRead(PIN_B));

  Serial.print(" GPIO41=");
  Serial.println(digitalRead(PIN_C));

  delay(100);
}