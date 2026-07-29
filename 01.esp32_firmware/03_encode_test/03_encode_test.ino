#include <math.h>

#define STEP_PIN 39
#define DIR_PIN 40

#define ENCODE_CLK 2
#define ENCODE_DT 42
#define ENCODE_SW 41

const float STEP_ANGLE = 1.8;
const int STEP_DELAY_US = 3000;

// 需要实测：手动转轴一圈，Encoder count 增加多少，就填多少
// 360 / 4 = 9
const float ENCODER_COUNTS_PER_REV = 40.0;

QueueHandle_t angleQueue;

volatile long encoderCount = 0;
volatile int lastCLK = 0;

volatile long motorStepCount = 0;

bool isValidNumber(String s) {
  if (s.length() == 0) return false;

  char *endptr;
  strtof(s.c_str(), &endptr);

  return (*endptr == '\0');
}

void IRAM_ATTR encoderISR() {
  int clkValue = digitalRead(ENCODE_CLK);
  int dtValue = digitalRead(ENCODE_DT);

  if (lastCLK != clkValue) {
    lastCLK = clkValue;

    if (clkValue != dtValue) {
      encoderCount++;
    } else {
      encoderCount--;
    }
  }
}

float normalizeAngle(float angle) {
  float a = fmod(angle, 360.0);
  if (a < 0) a += 360.0;
  return a;
}

void oneStep(bool dir) {
  digitalWrite(DIR_PIN, dir);
  delayMicroseconds(20);

  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_DELAY_US);

  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_DELAY_US);

  if (dir) {
    motorStepCount++;
  } else {
    motorStepCount--;
  }
}

void serialTask(void *pvParameters) {
  while (true) {
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();

      if (!isValidNumber(input)) {
        Serial.println("ERR: invalid number");
        continue;
      }

      float angle = input.toFloat();

      if (angle < -64800 || angle > 64800) {
        Serial.println("ERR: range error");
        continue;
      }

      xQueueSend(angleQueue, &angle, portMAX_DELAY);

      Serial.print("Queued: ");
      Serial.println(angle);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void motorTask(void *pvParameters) {
  float angle;

  while (true) {
    if (xQueueReceive(angleQueue, &angle, portMAX_DELAY)) {
      bool dir = angle >= 0;
      long steps = lround(fabs(angle) / STEP_ANGLE);

      Serial.print("Move command angle: ");
      Serial.print(angle);
      Serial.print(" deg, command steps: ");
      Serial.println(steps);

      for (long i = 0; i < steps; i++) {
        oneStep(dir);

        if (i % 100 == 0) {
          vTaskDelay(1);
        }
      }

      Serial.println("Move done");
    }
  }
}

void statusTask(void *pvParameters) {
  while (true) {
    long encCopy;

    noInterrupts();
    encCopy = encoderCount;
    interrupts();

    float realAngleTotal = encCopy * 360.0 / ENCODER_COUNTS_PER_REV;
    float realAngle360 = normalizeAngle(realAngleTotal);

    bool swState = digitalRead(ENCODE_SW);

    Serial.print("Encoder count: ");
    Serial.print(encCopy);

    Serial.print(" | Real angle: ");
    Serial.print(realAngle360);
    Serial.print(" deg");

    Serial.print(" | Real total angle: ");
    Serial.print(realAngleTotal);
    Serial.print(" deg");

    Serial.print(" | SW: ");
    Serial.println(swState == LOW ? "PRESSED" : "RELEASED");

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Boot OK");

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  pinMode(ENCODE_CLK, INPUT_PULLUP);
  pinMode(ENCODE_DT, INPUT_PULLUP);
  pinMode(ENCODE_SW, INPUT_PULLUP);

  lastCLK = digitalRead(ENCODE_CLK);
  attachInterrupt(digitalPinToInterrupt(ENCODE_CLK), encoderISR, CHANGE);

  angleQueue = xQueueCreate(10, sizeof(float));

  if (angleQueue == NULL) {
    Serial.println("ERR: queue create failed");
    while (true) delay(1000);
  }

  xTaskCreate(serialTask, "Serial Task", 4096, NULL, 1, NULL);
  xTaskCreate(motorTask, "Motor Task", 4096, NULL, 2, NULL);
  xTaskCreate(statusTask, "Status Task", 4096, NULL, 1, NULL);

  Serial.println("Input angle, e.g. 3 or -300");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}