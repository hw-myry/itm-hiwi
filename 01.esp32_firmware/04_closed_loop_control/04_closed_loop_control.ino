#include <math.h>

#define STEP_PIN 39
#define DIR_PIN 40

#define ENCODE_CLK 2
#define ENCODE_DT 42
#define ENCODE_SW 41

const float STEP_ANGLE = 1.8;
const int STEP_DELAY_US = 3000;

const float ENCODER_COUNTS_PER_REV = 40.0;
const float ENCODER_DEG_PER_COUNT = 360.0 / ENCODER_COUNTS_PER_REV;

// 编码器约 8.5°/count，所以允许误差不要设太小
const float ANGLE_TOLERANCE = 8.0;

const long MAX_CORRECTION_STEPS = 20000;

QueueHandle_t targetQueue;

volatile long encoderCount = 0;
volatile int lastCLK = 0;

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

long getEncoderCount() {
  long countCopy;

  noInterrupts();
  countCopy = encoderCount;
  interrupts();

  return countCopy;
}

float getRealAngleTotal() {
  long countCopy = getEncoderCount();
  return countCopy * ENCODER_DEG_PER_COUNT;
}

void oneStep(bool dir) {
  digitalWrite(DIR_PIN, dir);
  delayMicroseconds(20);

  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_DELAY_US);

  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_DELAY_US);
}

void moveSteps(bool dir, int steps) {
  for (int i = 0; i < steps; i++) {
    oneStep(dir);
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

      float targetAngle = input.toFloat();

      if (targetAngle < -64800 || targetAngle > 64800) {
        Serial.println("ERR: range error");
        continue;
      }

      xQueueSend(targetQueue, &targetAngle, portMAX_DELAY);

      Serial.print("Queued target angle: ");
      Serial.println(targetAngle);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void closedLoopTask(void *pvParameters) {
  float moveAngle;

  while (true) {
    if (xQueueReceive(targetQueue, &moveAngle, portMAX_DELAY)) {

      float startAngle = getRealAngleTotal();
      float targetAngle = startAngle + moveAngle;

      Serial.print("Closed-loop relative move: ");
      Serial.print(moveAngle);
      Serial.print(" deg | Start: ");
      Serial.print(startAngle);
      Serial.print(" deg | Target total: ");
      Serial.print(targetAngle);
      Serial.println(" deg");

      // 1. 先按步进电机理论角度粗走 95%
      bool roughDir = moveAngle >= 0;
      long roughSteps = lround(fabs(moveAngle) / STEP_ANGLE * 0.95);

      Serial.print("Rough move steps: ");
      Serial.println(roughSteps);

      for (long i = 0; i < roughSteps; i++) {
        oneStep(roughDir);

        if (i % 100 == 0) {
          vTaskDelay(1);
        }
      }

      delay(100);

      // 2. 再用编码器闭环补到目标
      long correctionSteps = 0;

      while (true) {
        float currentAngle = getRealAngleTotal();
        float err = targetAngle - currentAngle;

        if (fabs(err) <= ANGLE_TOLERANCE) {
          Serial.print("Arrived. Real total angle: ");
          Serial.print(currentAngle);
          Serial.print(" deg, error: ");
          Serial.print(err);
          Serial.println(" deg");
          break;
        }

        bool dir = err > 0;

        int stepBlock = 1;

        if (fabs(err) > 180) {
          stepBlock = 20;
        } else if (fabs(err) > 90) {
          stepBlock = 10;
        } else if (fabs(err) > 45) {
          stepBlock = 5;
        } else {
          stepBlock = 1;
        }

        moveSteps(dir, stepBlock);
        correctionSteps += stepBlock;

        if (correctionSteps >= MAX_CORRECTION_STEPS) {
          Serial.println("ERR: correction timeout / motor may be stuck");
          break;
        }

        if (correctionSteps % 50 == 0) {
          Serial.print("Current total: ");
          Serial.print(currentAngle);
          Serial.print(" deg | Target total: ");
          Serial.print(targetAngle);
          Serial.print(" deg | Error: ");
          Serial.println(err);
          vTaskDelay(1);
        }
      }
    }
  }
}

void statusTask(void *pvParameters) {
  while (true) {
    long encCopy = getEncoderCount();
    float realAngleTotal = encCopy * ENCODER_DEG_PER_COUNT;
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

    vTaskDelay(pdMS_TO_TICKS(1000));
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

  targetQueue = xQueueCreate(10, sizeof(float));

  if (targetQueue == NULL) {
    Serial.println("ERR: queue create failed");
    while (true) delay(1000);
  }

  xTaskCreate(serialTask, "Serial Task", 4096, NULL, 1, NULL);
  xTaskCreate(closedLoopTask, "Closed Loop Task", 4096, NULL, 2, NULL);
  xTaskCreate(statusTask, "Status Task", 4096, NULL, 1, NULL);

  Serial.println("Input relative angle, e.g. 90 or -180");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}