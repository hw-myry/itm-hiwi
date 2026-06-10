#include <math.h>

#define STEP_PIN 39
#define DIR_PIN 40

const float STEP_ANGLE = 1.8;
const int STEP_DELAY_US = 3000;

QueueHandle_t angleQueue;

bool isValidNumber(String s) {
  if (s.length() == 0) return false;

  char *endptr;
  strtof(s.c_str(), &endptr);

  return (*endptr == '\0');
}

void oneStep(bool dir) {
  digitalWrite(DIR_PIN, dir);
  delayMicroseconds(20);

  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_DELAY_US);

  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_DELAY_US);
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

      Serial.print("Move angle: ");
      Serial.print(angle);
      Serial.print(" deg, steps: ");
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

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Boot OK");

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  angleQueue = xQueueCreate(10, sizeof(float));

  if (angleQueue == NULL) {
    Serial.println("ERR: queue create failed");
    while (true) delay(1000);
  }

  xTaskCreate(serialTask, "Serial Task", 4096, NULL, 1, NULL);
  xTaskCreate(motorTask, "Motor Task", 4096, NULL, 2, NULL);

  Serial.println("Input angle, e.g. 3 or -300");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}