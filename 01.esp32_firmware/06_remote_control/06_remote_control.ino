#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

// ===== WiFi =====
const char* ssid = "Freifunk";
WiFiServer server(8080);

// ===== LED =====
#define LED_PIN 48
#define LED_COUNT 1

Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
bool ledEnabled = false;

// ===== Motor =====
#define STEP_PIN 39
#define DIR_PIN 40

#define ENCODE_CLK 2
#define ENCODE_DT 42
#define ENCODE_SW 41

const float STEP_ANGLE = 1.8;
const int STEP_DELAY_US = 3000;

const float ENCODER_COUNTS_PER_REV = 40.0;
const float ENCODER_DEG_PER_COUNT = 360.0 / ENCODER_COUNTS_PER_REV;
const float ANGLE_TOLERANCE = 8.0;

const long MAX_CORRECTION_STEPS = 20000;

QueueHandle_t targetQueue;

volatile long encoderCount = 0;
volatile int lastCLK = 0;

// ===== LED Functions =====
void setColor(int r, int g, int b) {
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}

// ===== Motor Functions =====
bool isValidNumber(String s) {
  if (s.length() == 0) return false;

  char* endptr;
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

// ===== Command Handler =====
String handleCommand(String cmd) {
  cmd.trim();

  if (cmd == "LED_ON") {
    ledEnabled = true;
    setColor(255, 255, 255);
    return "OK LED_ON";
  }

  if (cmd == "LED_OFF") {
    ledEnabled = false;
    pixels.clear();
    pixels.show();
    return "OK LED_OFF";
  }

  if (cmd.startsWith("RGB:")) {
    if (!ledEnabled) {
      return "LED IS OFF";
    }

    String rgb = cmd.substring(4);

    int p1 = rgb.indexOf(',');
    int p2 = rgb.indexOf(',', p1 + 1);

    if (p1 > 0 && p2 > p1) {
      int r = rgb.substring(0, p1).toInt();
      int g = rgb.substring(p1 + 1, p2).toInt();
      int b = rgb.substring(p2 + 1).toInt();

      r = constrain(r, 0, 255);
      g = constrain(g, 0, 255);
      b = constrain(b, 0, 255);

      setColor(r, g, b);
      return "OK RGB";
    } else {
      return "RGB FORMAT ERROR";
    }
  }

  // 支持 ANGLE:90
  if (cmd.startsWith("ANGLE:")) {
    cmd = cmd.substring(6);
    cmd.trim();
  }

  // 支持直接发送 90 / -180
  if (isValidNumber(cmd)) {
    float targetAngle = cmd.toFloat();

    if (targetAngle < -64800 || targetAngle > 64800) {
      return "ERR RANGE";
    }

    xQueueSend(targetQueue, &targetAngle, portMAX_DELAY);

    return "OK ANGLE " + String(targetAngle);
  }

  return "UNKNOWN CMD";
}

// ===== WiFi TCP Task =====
void wifiServerTask(void* pvParameters) {
  while (true) {
    WiFiClient client = server.available();

    if (client) {
      Serial.println("Client Connected");
      client.println("ESP32 READY");

      while (client.connected()) {
        if (client.available()) {
          String cmd = client.readStringUntil('\n');
          cmd.trim();

          Serial.print("WiFi Recv: ");
          Serial.println(cmd);

          String reply = handleCommand(cmd);

          client.println(reply);

          Serial.print("Reply: ");
          Serial.println(reply);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
      }

      client.stop();
      Serial.println("Client Disconnected");
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ===== Serial Task =====
void serialTask(void* pvParameters) {
  while (true) {
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();

      String reply = handleCommand(cmd);
      Serial.println(reply);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ===== Closed Loop Motor Task =====
void closedLoopTask(void* pvParameters) {
  float moveAngle;

  while (true) {
    if (xQueueReceive(targetQueue, &moveAngle, portMAX_DELAY)) {
      float startAngle = getRealAngleTotal();
      float targetAngle = startAngle + moveAngle;

      Serial.print("Move: ");
      Serial.print(moveAngle);
      Serial.print(" deg | Start: ");
      Serial.print(startAngle);
      Serial.print(" deg | Target: ");
      Serial.println(targetAngle);

      bool roughDir = moveAngle >= 0;
      long roughSteps = lround(fabs(moveAngle) / STEP_ANGLE * 0.95);

      for (long i = 0; i < roughSteps; i++) {
        oneStep(roughDir);

        if (i % 100 == 0) {
          vTaskDelay(1);
        }
      }

      delay(100);

      long correctionSteps = 0;

      while (true) {
        float currentAngle = getRealAngleTotal();
        float err = targetAngle - currentAngle;

        if (fabs(err) <= ANGLE_TOLERANCE) {
          Serial.print("Arrived. Angle: ");
          Serial.print(currentAngle);
          Serial.print(" deg | Error: ");
          Serial.println(err);
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
        }

        moveSteps(dir, stepBlock);
        correctionSteps += stepBlock;

        if (correctionSteps >= MAX_CORRECTION_STEPS) {
          Serial.println("ERR: correction timeout / motor may be stuck");
          break;
        }

        if (correctionSteps % 50 == 0) {
          vTaskDelay(1);
        }
      }
    }
  }
}

// ===== Status Task =====
void statusTask(void* pvParameters) {
  while (true) {
    long encCopy = getEncoderCount();
    float realAngleTotal = encCopy * ENCODER_DEG_PER_COUNT;
    float realAngle360 = normalizeAngle(realAngleTotal);
    bool swState = digitalRead(ENCODE_SW);

    Serial.print("Encoder: ");
    Serial.print(encCopy);

    Serial.print(" | Angle360: ");
    Serial.print(realAngle360);

    Serial.print(" | Total: ");
    Serial.print(realAngleTotal);

    Serial.print(" | SW: ");
    Serial.println(swState == LOW ? "PRESSED" : "RELEASED");

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Boot OK");

  // LED
  pixels.begin();
  pixels.setBrightness(80);
  pixels.clear();
  pixels.show();

  // Motor
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

  // WiFi
  // WiFi.mode(WIFI_STA);
  // WiFi.begin(ssid);

  // Serial.print("Connecting WiFi");

  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }

  // Serial.println();
  // Serial.println("WiFi Connected");
  // Serial.print("IP Address: ");
  // Serial.println(WiFi.localIP());

  // 热点模式
  WiFi.mode(WIFI_AP);

  WiFi.softAP(
    "ESP32_Motor",
    "12345678");

  IPAddress ip = WiFi.softAPIP();

  Serial.print("AP IP: ");
  Serial.println(ip);

  server.begin();
  Serial.println("TCP Server Started on port 8080");

  // Tasks
  xTaskCreate(serialTask, "Serial Task", 4096, NULL, 1, NULL);
  xTaskCreate(wifiServerTask, "WiFi Server Task", 4096, NULL, 1, NULL);
  xTaskCreate(closedLoopTask, "Closed Loop Task", 4096, NULL, 2, NULL);
  xTaskCreate(statusTask, "Status Task", 4096, NULL, 1, NULL);

  Serial.println("Input angle via Serial or WiFi, e.g. 90 / -180 / ANGLE:45");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}