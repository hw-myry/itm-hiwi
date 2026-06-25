#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>

// =====================================================
// 网络模式选择
// =====================================================
// NETMODE = 0：连接外部 WiFi
// NETMODE = 1：ESP32 自己开热点 AP
#define NETMODE 1

// ===== WiFi STA 模式参数 =====
const char* WIFI_SSID = "Freifunk";
const char* WIFI_PASS = "";

// ===== AP 热点模式参数 =====
const char* AP_SSID = "ESP32_Motor";
const char* AP_PASS = "12345678";

// ===== TCP Server =====
WiFiServer server(8080);

// =====================================================
// Flash 参数存储
// =====================================================
Preferences prefs;

// =====================================================
// LED
// =====================================================
#define LED_PIN 48
#define LED_COUNT 1

Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
bool ledEnabled = false;

// =====================================================
// Motor
// =====================================================
#define STEP_PIN 39
#define DIR_PIN 40

#define ENCODE_CLK 2
#define ENCODE_DT 42
#define ENCODE_SW 41

// 如果驱动器没有细分，1个STEP = 1.8度
// 如果16细分，改成：1.8 / 16.0
const float STEP_ANGLE = 1.8f;

// =====================================================
// 默认参数：Flash 没有保存过时使用
// =====================================================
const float DEFAULT_SPEED_HZ = 500.0f;
const float DEFAULT_ENCODER_COUNTS_PER_REV = 40.0f;
const float DEFAULT_ANGLE_TOLERANCE = 4.5f;

// =====================================================
// 运行时参数：开机从 Flash 读取，也可以通过命令修改
// =====================================================
float speedHz = DEFAULT_SPEED_HZ;

float encoderCountsPerRev = DEFAULT_ENCODER_COUNTS_PER_REV;
float encoderDegPerCount = 360.0f / DEFAULT_ENCODER_COUNTS_PER_REV;

float angleTolerance = DEFAULT_ANGLE_TOLERANCE;

// =====================================================
// 固定控制参数
// =====================================================
// 单次运动超时
#define MOVE_TIMEOUT_MS 30000

// 步数保护
#define MOVE_MAX_STEP_MULTIPLIER 80
#define MOVE_STEP_GUARD_EXTRA 8000
#define MOVE_MIN_MAX_STEPS 10000

// 调试打印间隔，单位ms
#define MOVE_DEBUG_INTERVAL_MS 800

// 每隔多少ms主动让出CPU，防止 Task Watchdog 重启
#define MOTOR_TASK_YIELD_INTERVAL_MS 30

// 如果发现方向反了，改这里
// 你现在这个方向已经正常，所以保持 1
#define MOTOR_DIR_INVERT 1

// =====================================================
// FreeRTOS
// =====================================================
QueueHandle_t targetQueue;

volatile long encoderCount = 0;
volatile int lastCLK = 0;

// =====================================================
// LED Functions
// =====================================================
void setColor(int r, int g, int b) {
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}

// =====================================================
// Basic Functions
// =====================================================
bool isValidNumber(String s) {
  s.trim();

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
  float a = fmod(angle, 360.0f);
  if (a < 0) a += 360.0f;
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
  return countCopy * encoderDegPerCount;
}

// =====================================================
// Flash Config Functions
// =====================================================
void updateDerivedParams() {
  if (encoderCountsPerRev < 1.0f) {
    encoderCountsPerRev = DEFAULT_ENCODER_COUNTS_PER_REV;
  }

  encoderDegPerCount = 360.0f / encoderCountsPerRev;
}

String getConfigString() {
  return "CFG SPEED=" + String(speedHz, 1) +
         " CPR=" + String(encoderCountsPerRev, 4) +
         " DEG_PER_COUNT=" + String(encoderDegPerCount, 4) +
         " TOL=" + String(angleTolerance, 2);
}

void loadConfigFromFlash() {
  prefs.begin("motorcfg", true);

  speedHz = prefs.getFloat("speed", DEFAULT_SPEED_HZ);
  encoderCountsPerRev = prefs.getFloat("cpr", DEFAULT_ENCODER_COUNTS_PER_REV);
  angleTolerance = prefs.getFloat("tol", DEFAULT_ANGLE_TOLERANCE);

  prefs.end();

  updateDerivedParams();

  Serial.print("Loaded config from Flash: ");
  Serial.println(getConfigString());
}

void saveConfigToFlash() {
  prefs.begin("motorcfg", false);

  prefs.putFloat("speed", speedHz);
  prefs.putFloat("cpr", encoderCountsPerRev);
  prefs.putFloat("tol", angleTolerance);

  prefs.end();

  Serial.print("Saved config to Flash: ");
  Serial.println(getConfigString());
}

void resetConfigToDefault() {
  speedHz = DEFAULT_SPEED_HZ;
  encoderCountsPerRev = DEFAULT_ENCODER_COUNTS_PER_REV;
  angleTolerance = DEFAULT_ANGLE_TOLERANCE;

  updateDerivedParams();

  prefs.begin("motorcfg", false);
  prefs.clear();
  prefs.end();

  Serial.println("Config reset to default");
}

// =====================================================
// Motor Step Functions
// =====================================================
void oneStepWithDelay(bool dir, uint32_t halfPeriodUs) {
#if MOTOR_DIR_INVERT == 1
  dir = !dir;
#endif

  digitalWrite(DIR_PIN, dir ? HIGH : LOW);
  delayMicroseconds(20);

  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(halfPeriodUs);

  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(halfPeriodUs);
}

// =====================================================
// WiFi / AP Setup
// =====================================================
void setupNetwork() {
#if NETMODE == 0

  WiFi.mode(WIFI_STA);

  Serial.print("Connecting WiFi: ");
  Serial.println(WIFI_SSID);

  if (strlen(WIFI_PASS) == 0) {
    WiFi.begin(WIFI_SSID);
  } else {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }

  unsigned long startMs = millis();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");

    if (millis() - startMs > 20000) {
      Serial.println();
      Serial.println("ERR: WiFi connect timeout");
      break;
    }
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi Connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi not connected");
  }

#elif NETMODE == 1

  WiFi.mode(WIFI_AP);

  bool ok = WiFi.softAP(AP_SSID, AP_PASS);

  if (ok) {
    Serial.println("AP Started");
    Serial.print("AP SSID: ");
    Serial.println(AP_SSID);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("ERR: AP start failed");
  }

#else
  #error "NETMODE must be 0 or 1"
#endif

  server.begin();
  Serial.println("TCP Server Started on port 8080");
}

// =====================================================
// Command Handler
// =====================================================
String handleCommand(String cmd) {
  cmd.trim();

  // =====================================================
  // LED
  // =====================================================
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

  // =====================================================
  // 读取全部参数
  // =====================================================
  if (cmd == "CFG?") {
    return getConfigString();
  }

  // =====================================================
  // 保存当前参数到 Flash
  // =====================================================
  if (cmd == "CFG_SAVE") {
    saveConfigToFlash();
    return "OK CFG_SAVE " + getConfigString();
  }

  // =====================================================
  // 恢复默认参数，并清除 Flash
  // =====================================================
  if (cmd == "CFG_RESET") {
    resetConfigToDefault();
    return "OK CFG_RESET " + getConfigString();
  }

  // =====================================================
  // 速度查询
  // =====================================================
  if (cmd == "SPEED?") {
    return "SPEED=" + String(speedHz, 1);
  }

  // =====================================================
  // 设置匀速速度
  // 命令：SPEED:500
  // 单位：step/s
  // 只改 RAM，不立刻写 Flash
  // =====================================================
  if (cmd.startsWith("SPEED:")) {
    String data = cmd.substring(6);
    data.trim();

    if (isValidNumber(data)) {
      float newSpeed = data.toFloat();

      if (newSpeed < 1.0f || newSpeed > 3000.0f) {
        return "ERR SPEED RANGE";
      }

      speedHz = newSpeed;

      return "OK SPEED=" + String(speedHz, 1);
    }

    return "ERR SPEED FORMAT";
  }

  // =====================================================
  // 编码器查询
  // =====================================================
  if (cmd == "ENCODER?") {
    return "ENCODER CPR=" + String(encoderCountsPerRev, 4) +
           " DEG_PER_COUNT=" + String(encoderDegPerCount, 4);
  }

  // =====================================================
  // 编码器比例设置：ENCODER:40
  // 只改 RAM，不立刻写 Flash
  // =====================================================
  if (cmd.startsWith("ENCODER:")) {
    String data = cmd.substring(8);
    data.trim();

    if (isValidNumber(data)) {
      float newCpr = data.toFloat();

      if (newCpr < 1.0f || newCpr > 10000.0f) {
        return "ERR ENCODER RANGE";
      }

      encoderCountsPerRev = newCpr;
      updateDerivedParams();

      return "OK ENCODER CPR=" + String(encoderCountsPerRev, 4) +
             " DEG_PER_COUNT=" + String(encoderDegPerCount, 4);
    }

    return "ERR ENCODER FORMAT";
  }

  // =====================================================
  // 容差查询
  // =====================================================
  if (cmd == "TOL?") {
    return "TOL=" + String(angleTolerance, 2);
  }

  // =====================================================
  // 容差设置：TOL:4.5
  // 只改 RAM，不立刻写 Flash
  // =====================================================
  if (cmd.startsWith("TOL:")) {
    String data = cmd.substring(4);
    data.trim();

    if (isValidNumber(data)) {
      float newTol = data.toFloat();

      if (newTol < 0.1f || newTol > 30.0f) {
        return "ERR TOL RANGE";
      }

      angleTolerance = newTol;

      return "OK TOL=" + String(angleTolerance, 2);
    }

    return "ERR TOL FORMAT";
  }

  // =====================================================
  // 当前角度查询
  // =====================================================
  if (cmd == "ANGLE?") {
    long enc = getEncoderCount();
    float angle = getRealAngleTotal();

    return "ANGLE CURRENT=" + String(angle, 2) +
           " ENCODER=" + String(enc);
  }

  // =====================================================
  // 支持 ANGLE:90
  // =====================================================
  if (cmd.startsWith("ANGLE:")) {
    cmd = cmd.substring(6);
    cmd.trim();
  }

  // =====================================================
  // 支持直接发送 90 / -180
  // =====================================================
  if (isValidNumber(cmd)) {
    float targetAngle = cmd.toFloat();

    if (targetAngle < -64800 || targetAngle > 64800) {
      return "ERR RANGE";
    }

    xQueueSend(targetQueue, &targetAngle, portMAX_DELAY);

    return "OK ANGLE " + String(targetAngle, 2);
  }

  return "UNKNOWN CMD";
}

// =====================================================
// WiFi TCP Task
// =====================================================
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

// =====================================================
// Serial Task
// =====================================================
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

// =====================================================
// Constant Speed Closed Loop Motor Task
// =====================================================
void closedLoopTask(void* pvParameters) {
  float moveAngle;

  while (true) {
    if (xQueueReceive(targetQueue, &moveAngle, portMAX_DELAY)) {

      float startAngle = getRealAngleTotal();
      float targetAngle = startAngle + moveAngle;

      Serial.print("Const Speed Move: ");
      Serial.print(moveAngle);
      Serial.print(" deg | Start: ");
      Serial.print(startAngle);
      Serial.print(" deg | Target: ");
      Serial.print(targetAngle);
      Serial.print(" deg | Speed: ");
      Serial.print(speedHz);
      Serial.println(" step/s");

      long stepCounter = 0;
      long expectedSteps = lround(fabs(moveAngle) / STEP_ANGLE);

      long maxAllowedSteps =
        expectedSteps * MOVE_MAX_STEP_MULTIPLIER + MOVE_STEP_GUARD_EXTRA;

      if (maxAllowedSteps < MOVE_MIN_MAX_STEPS) {
        maxAllowedSteps = MOVE_MIN_MAX_STEPS;
      }

      unsigned long startMs = millis();
      unsigned long lastDebugMs = millis();
      unsigned long lastYieldMs = millis();

      float usedSpeedHz = speedHz;

      if (usedSpeedHz < 1.0f) {
        usedSpeedHz = 1.0f;
      }

      if (usedSpeedHz > 3000.0f) {
        usedSpeedHz = 3000.0f;
      }

      uint32_t halfPeriodUs =
        (uint32_t)(1000000.0f / usedSpeedHz / 2.0f);

      if (halfPeriodUs < 50) {
        halfPeriodUs = 50;
      }

      while (true) {
        float currentAngle = getRealAngleTotal();
        float error = targetAngle - currentAngle;
        float absError = fabs(error);

        // =====================================================
        // 到位判断
        // =====================================================
        if (absError <= angleTolerance) {
          Serial.print("Arrived. Angle: ");
          Serial.print(currentAngle);
          Serial.print(" deg | Error: ");
          Serial.print(error);
          Serial.print(" deg | Steps: ");
          Serial.println(stepCounter);
          break;
        }

        // =====================================================
        // 超时保护
        // =====================================================
        if (millis() - startMs > MOVE_TIMEOUT_MS) {
          Serial.print("ERR: const speed move timeout | Current: ");
          Serial.print(currentAngle);
          Serial.print(" deg | Error: ");
          Serial.println(error);
          break;
        }

        // =====================================================
        // 步数保护
        // =====================================================
        if (stepCounter >= maxAllowedSteps) {
          Serial.print("ERR: too many steps | Current: ");
          Serial.print(currentAngle);
          Serial.print(" deg | Error: ");
          Serial.print(error);
          Serial.print(" | Steps: ");
          Serial.println(stepCounter);
          Serial.println("Check STEP_ANGLE / speed / encoder direction / mechanical jam");
          break;
        }

        // =====================================================
        // 根据误差方向决定转向
        // =====================================================
        bool dir = error > 0;

        oneStepWithDelay(dir, halfPeriodUs);
        stepCounter++;

        // =====================================================
        // 调试打印
        // =====================================================
        unsigned long nowMs = millis();

        if (nowMs - lastDebugMs >= MOVE_DEBUG_INTERVAL_MS) {
          lastDebugMs = nowMs;

          Serial.print("Move Debug | Current: ");
          Serial.print(currentAngle);
          Serial.print(" deg | Error: ");
          Serial.print(error);
          Serial.print(" deg | Speed: ");
          Serial.print(usedSpeedHz);
          Serial.print(" | Dir: ");
          Serial.println(dir ? "POS" : "NEG");
        }

        // =====================================================
        // 防止 Task Watchdog
        // =====================================================
        if (nowMs - lastYieldMs >= MOTOR_TASK_YIELD_INTERVAL_MS) {
          lastYieldMs = nowMs;
          vTaskDelay(pdMS_TO_TICKS(1));
        }
      }
    }
  }
}

// =====================================================
// Status Task
// =====================================================
void statusTask(void* pvParameters) {
  while (true) {
    long encCopy = getEncoderCount();
    float realAngleTotal = encCopy * encoderDegPerCount;
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

// =====================================================
// Setup
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Boot OK");

  // 开机先从 Flash 读取参数
  loadConfigFromFlash();

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
    while (true) {
      delay(1000);
    }
  }

  // Network
  setupNetwork();

  // =====================================================
  // Tasks
  // =====================================================
  // ESP32-S3 双核：
  // WiFi 相关放 Core 0
  // 电机控制放 Core 1
  // 电机任务内部会主动 vTaskDelay，避免看门狗重启

  xTaskCreatePinnedToCore(
    serialTask,
    "Serial Task",
    4096,
    NULL,
    1,
    NULL,
    1
  );

  xTaskCreatePinnedToCore(
    wifiServerTask,
    "WiFi Server Task",
    4096,
    NULL,
    1,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    closedLoopTask,
    "Const Speed Task",
    8192,
    NULL,
    2,
    NULL,
    1
  );

  xTaskCreatePinnedToCore(
    statusTask,
    "Status Task",
    4096,
    NULL,
    1,
    NULL,
    1
  );

  Serial.println("Input angle via Serial or WiFi, e.g. 90 / -180 / ANGLE:45");
  Serial.println("Config commands: CFG? / CFG_SAVE / CFG_RESET / SPEED? / ENCODER? / TOL?");
}

// =====================================================
// Loop
// =====================================================
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}