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

// 配置版本：v3 用来把旧 Flash 里的慢速参数升级为更快的默认值
const uint32_t CONFIG_VERSION = 3;

// =====================================================
// Flash 当前位置 / 单方向回零参数
// =====================================================
// savedHomeOffsetAngle 表示“当前位置相对零点的角度”，只保存 0~360 度。
// 约定：右转为正，左转为负，保存时自动取 360 度余数。
// 例如：右转90 => 90；重启后自动继续右转 360 - 90 = 270 度回零。
// 例如：右转120再左转60 => 60；重启后自动右转300度回零。
const char* FLASH_KEY_HOME_ANGLE = "homeang";
const float AUTO_HOME_MIN_ANGLE = 0.5f;      // 小于这个角度就认为不用回零
const float POSITION_ZERO_EPS = 0.5f;        // 接近0或360时保存成0

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
const float STEP_ANGLE = 1.8f ;

// =====================================================
// 默认参数：Flash 没有保存过时使用
// =====================================================
const float DEFAULT_KP = 5.0f;
const float DEFAULT_KI = 0.015f;
const float DEFAULT_KD = 0.0f;

const float DEFAULT_SPEED_HZ = 1500.0f;
// PID输出很小时的最低步进速度，避免快到目标时速度过慢
// v3 提高最低速度，解决接近目标时太慢的问题
const float DEFAULT_MIN_SPEED_HZ = 500.0f;
const float DEFAULT_ENCODER_COUNTS_PER_REV = 40.0f;
const float DEFAULT_ANGLE_TOLERANCE = 4.5f;

// =====================================================
// 运行时参数：开机从 Flash 读取，也可以通过命令修改
// =====================================================
float pidKp = DEFAULT_KP;
float pidKi = DEFAULT_KI;
float pidKd = DEFAULT_KD;

float speedHz = DEFAULT_SPEED_HZ;
float minSpeedHz = DEFAULT_MIN_SPEED_HZ;

float encoderCountsPerRev = DEFAULT_ENCODER_COUNTS_PER_REV;
float encoderDegPerCount = 360.0f / DEFAULT_ENCODER_COUNTS_PER_REV;

float angleTolerance = DEFAULT_ANGLE_TOLERANCE;

// 保存到 Flash 的当前位置角度：范围 0~360，表示从零点开始正方向转到当前点
float savedHomeOffsetAngle = 0.0f;

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
struct MotorMoveCommand {
  float angle;          // 相对运动角度，右转为正，左转为负
  bool zeroAfterMove;   // 这次运动完成后是否把累计角度清零
  bool savePosition;    // 这次运动结束后是否更新 Flash 里的累计角度
};

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

void setEncoderCount(long newCount) {
  noInterrupts();
  encoderCount = newCount;
  interrupts();
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
  return "CFG KP=" + String(pidKp, 4) +
         " KI=" + String(pidKi, 6) +
         " KD=" + String(pidKd, 6) +
         " SPEED=" + String(speedHz, 1) +
         " MIN_SPEED=" + String(minSpeedHz, 1) +
         " CPR=" + String(encoderCountsPerRev, 4) +
         " DEG_PER_COUNT=" + String(encoderDegPerCount, 4) +
         " TOL=" + String(angleTolerance, 2) +
         " HOME_POS=" + String(savedHomeOffsetAngle, 2);
}

void loadConfigFromFlash() {
  prefs.begin("motorcfg", true);

  uint32_t cfgVer = prefs.getUInt("ver", 0);

  pidKp = prefs.getFloat("kp", DEFAULT_KP);
  pidKi = prefs.getFloat("ki", DEFAULT_KI);
  pidKd = prefs.getFloat("kd", DEFAULT_KD);

  speedHz = prefs.getFloat("speed", DEFAULT_SPEED_HZ);
  minSpeedHz = prefs.getFloat("minspd", DEFAULT_MIN_SPEED_HZ);
  encoderCountsPerRev = prefs.getFloat("cpr", DEFAULT_ENCODER_COUNTS_PER_REV);
  angleTolerance = prefs.getFloat("tol", DEFAULT_ANGLE_TOLERANCE);

  savedHomeOffsetAngle = prefs.getFloat(FLASH_KEY_HOME_ANGLE, 0.0f);

  prefs.end();

  if (savedHomeOffsetAngle < -64800.0f || savedHomeOffsetAngle > 64800.0f) {
    Serial.println("Saved home position out of range, reset to 0");
    savedHomeOffsetAngle = 0.0f;
  }

  savedHomeOffsetAngle = normalizeAngle(savedHomeOffsetAngle);

  if (savedHomeOffsetAngle <= POSITION_ZERO_EPS ||
      savedHomeOffsetAngle >= 360.0f - POSITION_ZERO_EPS) {
    savedHomeOffsetAngle = 0.0f;
  }

  // 如果是旧版本保存的配置，自动把速度相关参数升级到 v3 的快速默认值。
  // 这样烧录新程序后不会继续沿用 v2 中保存的 500 / 180 慢速参数。
  if (cfgVer < CONFIG_VERSION) {
    if (speedHz < DEFAULT_SPEED_HZ) {
      speedHz = DEFAULT_SPEED_HZ;
    }

    if (minSpeedHz < DEFAULT_MIN_SPEED_HZ) {
      minSpeedHz = DEFAULT_MIN_SPEED_HZ;
    }

    Serial.println("Old config detected, speed parameters upgraded to v3 fast defaults");
  }

  updateDerivedParams();

  Serial.print("Loaded config from Flash: ");
  Serial.println(getConfigString());
}

void saveConfigToFlash() {
  prefs.begin("motorcfg", false);

  prefs.putUInt("ver", CONFIG_VERSION);

  prefs.putFloat("kp", pidKp);
  prefs.putFloat("ki", pidKi);
  prefs.putFloat("kd", pidKd);

  prefs.putFloat("speed", speedHz);
  prefs.putFloat("minspd", minSpeedHz);
  prefs.putFloat("cpr", encoderCountsPerRev);
  prefs.putFloat("tol", angleTolerance);
  prefs.putFloat(FLASH_KEY_HOME_ANGLE, savedHomeOffsetAngle);

  prefs.end();

  Serial.print("Saved config to Flash: ");
  Serial.println(getConfigString());
}

void resetConfigToDefault() {
  pidKp = DEFAULT_KP;
  pidKi = DEFAULT_KI;
  pidKd = DEFAULT_KD;

  speedHz = DEFAULT_SPEED_HZ;
  minSpeedHz = DEFAULT_MIN_SPEED_HZ;
  encoderCountsPerRev = DEFAULT_ENCODER_COUNTS_PER_REV;
  angleTolerance = DEFAULT_ANGLE_TOLERANCE;
  savedHomeOffsetAngle = 0.0f;

  updateDerivedParams();

  prefs.begin("motorcfg", false);
  prefs.clear();
  prefs.end();

  Serial.println("Config reset to default");
}

float getAutoHomeMoveAngle() {
  float pos = normalizeAngle(savedHomeOffsetAngle);

  if (pos <= POSITION_ZERO_EPS || pos >= 360.0f - POSITION_ZERO_EPS) {
    return 0.0f;
  }

  return 360.0f - pos;
}

String getHomeString() {
  return "HOME_POS=" + String(savedHomeOffsetAngle, 2) +
         " HOME_BACK=" + String(getAutoHomeMoveAngle(), 2) +
         " ENCODER=" + String(getEncoderCount()) +
         " CURRENT_ANGLE=" + String(getRealAngleTotal(), 2);
}

void saveHomeOffsetToFlash() {
  prefs.begin("motorcfg", false);
  prefs.putUInt("ver", CONFIG_VERSION);
  prefs.putFloat(FLASH_KEY_HOME_ANGLE, savedHomeOffsetAngle);
  prefs.end();

  Serial.print("Saved home position to Flash: ");
  Serial.println(savedHomeOffsetAngle, 2);
}

void updateSavedHomeOffsetByActualDelta(float actualDelta) {
  savedHomeOffsetAngle = normalizeAngle(savedHomeOffsetAngle + actualDelta);

  if (savedHomeOffsetAngle <= POSITION_ZERO_EPS ||
      savedHomeOffsetAngle >= 360.0f - POSITION_ZERO_EPS) {
    savedHomeOffsetAngle = 0.0f;
  }

  saveHomeOffsetToFlash();
}

void clearSavedHomeOffset(bool alsoResetEncoder) {
  savedHomeOffsetAngle = 0.0f;

  if (alsoResetEncoder) {
    setEncoderCount(0);
  }

  saveHomeOffsetToFlash();
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
  // PID 查询
  // =====================================================
  if (cmd == "PID?") {
    return "PID KP=" + String(pidKp, 4) +
           " KI=" + String(pidKi, 6) +
           " KD=" + String(pidKd, 6);
  }

  // =====================================================
  // PID 设置
  // 命令：PID:5.0,0.015,0.0
  // 只改 RAM，不立刻写 Flash
  // =====================================================
  if (cmd.startsWith("PID:")) {
    String data = cmd.substring(4);
    data.trim();

    int p1 = data.indexOf(',');
    int p2 = data.indexOf(',', p1 + 1);

    if (p1 > 0 && p2 > p1) {
      String kpStr = data.substring(0, p1);
      String kiStr = data.substring(p1 + 1, p2);
      String kdStr = data.substring(p2 + 1);

      kpStr.trim();
      kiStr.trim();
      kdStr.trim();

      if (isValidNumber(kpStr) &&
          isValidNumber(kiStr) &&
          isValidNumber(kdStr)) {

        float newKp = kpStr.toFloat();
        float newKi = kiStr.toFloat();
        float newKd = kdStr.toFloat();

        if (newKp < 0.0f || newKp > 100.0f ||
            newKi < 0.0f || newKi > 10.0f ||
            newKd < 0.0f || newKd > 10.0f) {
          return "ERR PID RANGE";
        }

        pidKp = newKp;
        pidKi = newKi;
        pidKd = newKd;

        return "OK PID KP=" + String(pidKp, 4) +
               " KI=" + String(pidKi, 6) +
               " KD=" + String(pidKd, 6);
      }
    }

    return "ERR PID FORMAT";
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
  // 最低速度查询
  // PID 输出很小时，至少用这个 step/s 运行，避免接近目标太慢
  // =====================================================
  if (cmd == "MINSPEED?" || cmd == "MIN_SPEED?") {
    return "MIN_SPEED=" + String(minSpeedHz, 1);
  }

  // =====================================================
  // 最低速度设置：MINSPEED:180
  // 只改 RAM，不立刻写 Flash
  // =====================================================
  if (cmd.startsWith("MINSPEED:") || cmd.startsWith("MIN_SPEED:")) {
    String data;

    if (cmd.startsWith("MINSPEED:")) {
      data = cmd.substring(9);
    } else {
      data = cmd.substring(10);
    }

    data.trim();

    if (isValidNumber(data)) {
      float newMinSpeed = data.toFloat();

      if (newMinSpeed < 1.0f || newMinSpeed > 3000.0f) {
        return "ERR MINSPEED RANGE";
      }

      minSpeedHz = newMinSpeed;

      return "OK MIN_SPEED=" + String(minSpeedHz, 1);
    }

    return "ERR MINSPEED FORMAT";
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
           " ENCODER=" + String(enc) +
           " HOME_POS=" + String(savedHomeOffsetAngle, 2);
  }

  // =====================================================
  // Flash 当前位置查询
  // =====================================================
  if (cmd == "HOME?" || cmd == "POS?") {
    return getHomeString();
  }

  // =====================================================
  // 把当前位置设为零点：清除 Flash 位置，同时把本次开机的编码器计数清零
  // 命令：HOME_ZERO / ZERO
  // =====================================================
  if (cmd == "HOME_ZERO" || cmd == "ZERO") {
    clearSavedHomeOffset(true);
    return "OK HOME_ZERO " + getHomeString();
  }

  // =====================================================
  // 手动设置 Flash 里的当前位置角度
  // 命令：HOME:60 表示当前位置在零点后方正方向 60 度；重启后会继续正方向走 300 度回零
  // =====================================================
  if (cmd.startsWith("HOME:") || cmd.startsWith("POS:")) {
    String data;

    if (cmd.startsWith("HOME:")) {
      data = cmd.substring(5);
    } else {
      data = cmd.substring(4);
    }

    data.trim();

    if (isValidNumber(data)) {
      float newHome = data.toFloat();

      if (newHome < -64800.0f || newHome > 64800.0f) {
        return "ERR HOME RANGE";
      }

      savedHomeOffsetAngle = normalizeAngle(newHome);

      if (savedHomeOffsetAngle <= POSITION_ZERO_EPS ||
          savedHomeOffsetAngle >= 360.0f - POSITION_ZERO_EPS) {
        savedHomeOffsetAngle = 0.0f;
      }

      saveHomeOffsetToFlash();

      return "OK HOME " + getHomeString();
    }

    return "ERR HOME FORMAT";
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

    MotorMoveCommand moveCmd;
    moveCmd.angle = targetAngle;
    moveCmd.zeroAfterMove = false;
    moveCmd.savePosition = true;

    xQueueSend(targetQueue, &moveCmd, portMAX_DELAY);

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
  MotorMoveCommand moveCmd;

  while (true) {
    if (xQueueReceive(targetQueue, &moveCmd, portMAX_DELAY)) {

      float moveAngle = moveCmd.angle;
      float startAngle = getRealAngleTotal();
      float targetAngle = startAngle + moveAngle;

      Serial.print("PID Move: ");
      Serial.print(moveAngle);
      Serial.print(" deg | Start: ");
      Serial.print(startAngle);
      Serial.print(" deg | Target: ");
      Serial.print(targetAngle);
      Serial.print(" deg | MaxSpeed: ");
      Serial.print(speedHz);
      Serial.print(" step/s | MinSpeed: ");
      Serial.print(minSpeedHz);
      Serial.print(" step/s | PID KP=");
      Serial.print(pidKp);
      Serial.print(" KI=");
      Serial.print(pidKi);
      Serial.print(" KD=");
      Serial.print(pidKd);
      Serial.print(" | HomePos=");
      Serial.print(savedHomeOffsetAngle);
      Serial.print(" | ZeroAfterMove=");
      Serial.println(moveCmd.zeroAfterMove ? "YES" : "NO");

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

      float maxSpeedHz = speedHz;
      if (maxSpeedHz < 1.0f) {
        maxSpeedHz = 1.0f;
      }
      if (maxSpeedHz > 3000.0f) {
        maxSpeedHz = 3000.0f;
      }

      float localMinSpeedHz = minSpeedHz;
      if (localMinSpeedHz < 1.0f) {
        localMinSpeedHz = 1.0f;
      }
      if (localMinSpeedHz > maxSpeedHz) {
        localMinSpeedHz = maxSpeedHz;
      }

      float integral = 0.0f;
      float lastError = targetAngle - startAngle;
      unsigned long lastPidUs = micros();

      bool arrived = false;
      float finalAngle = startAngle;

      while (true) {
        float currentAngle = getRealAngleTotal();
        finalAngle = currentAngle;

        float error = targetAngle - currentAngle;
        float absError = fabs(error);

        // =====================================================
        // 到位判断
        // =====================================================
        if (absError <= angleTolerance) {
          arrived = true;

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
          Serial.print("ERR: pid move timeout | Current: ");
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
        // PID 计算
        // 改进点：
        // 1. 方向只由当前位置误差决定，避免积分项把电机继续推过头
        // 2. 误差过零时清积分，防止 windup 导致第二圈越走越多
        // 3. PID 输出很小时使用 minSpeedHz，避免接近目标时过慢
        // =====================================================
        unsigned long nowUs = micros();
        float dt = (nowUs - lastPidUs) / 1000000.0f;

        if (dt <= 0.0f || dt > 1.0f) {
          dt = 0.001f;
        }

        lastPidUs = nowUs;

        bool errorCrossedZero =
          (error > 0.0f && lastError < 0.0f) ||
          (error < 0.0f && lastError > 0.0f);

        float derivative = 0.0f;

        if (errorCrossedZero) {
          integral = 0.0f;
          derivative = 0.0f;
        } else {
          derivative = (error - lastError) / dt;
        }

        integral += error * dt;

        // 积分限幅：I 项最多占最大速度的 35%，避免越过目标后继续被积分项推着走
        if (pidKi > 0.000001f) {
          float integralLimit = (maxSpeedHz * 0.35f) / pidKi;
          if (integralLimit < 1.0f) {
            integralLimit = 1.0f;
          }
          if (integral > integralLimit) {
            integral = integralLimit;
          }
          if (integral < -integralLimit) {
            integral = -integralLimit;
          }
        } else {
          integral = 0.0f;
        }

        lastError = error;

        float pidOutput = pidKp * error + pidKi * integral + pidKd * derivative;

        // 方向只看实际误差，PID 输出只决定速度大小
        bool dir = error > 0.0f;

        float usedSpeedHz = fabs(pidOutput);

        if (usedSpeedHz < localMinSpeedHz) {
          usedSpeedHz = localMinSpeedHz;
        }

        if (usedSpeedHz > maxSpeedHz) {
          usedSpeedHz = maxSpeedHz;
        }

        uint32_t halfPeriodUs =
          (uint32_t)(1000000.0f / usedSpeedHz / 2.0f);

        if (halfPeriodUs < 50) {
          halfPeriodUs = 50;
        }

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
          Serial.print(" | I: ");
          Serial.print(integral);
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

      // =====================================================
      // 一次运动结束后再写 Flash，避免每一步写 Flash 导致寿命下降
      // =====================================================
      if (moveCmd.savePosition) {
        float actualDelta = finalAngle - startAngle;

        if (moveCmd.zeroAfterMove && arrived) {
          // 自动回零完成：Flash 归零，编码器计数也归零
          clearSavedHomeOffset(true);
          Serial.println("Auto home finished, home position cleared to 0");
        } else {
          // 普通运动：用编码器测到的实际角度增量累加保存
          // 如果运动失败，也保存已实际移动的角度，避免 Flash 位置完全不变
          updateSavedHomeOffsetByActualDelta(actualDelta);

          Serial.print("Home position updated by actual delta: ");
          Serial.print(actualDelta, 2);
          Serial.print(" deg | New home position: ");
          Serial.println(savedHomeOffsetAngle, 2);
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

    Serial.print(" | HomePos: ");
    Serial.print(savedHomeOffsetAngle);

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

  targetQueue = xQueueCreate(10, sizeof(MotorMoveCommand));

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
    "PID Task",
    8192,
    NULL,
    2,
    NULL,
    1
  );

  // 开机自动回零：如果 Flash 里保存了当前位置，就继续正方向走 360 - 当前位置 回零
  float autoHomeAngle = getAutoHomeMoveAngle();

  if (fabs(autoHomeAngle) > AUTO_HOME_MIN_ANGLE) {
    MotorMoveCommand homeCmd;
    homeCmd.angle = autoHomeAngle;
    homeCmd.zeroAfterMove = true;
    homeCmd.savePosition = true;

    xQueueSend(targetQueue, &homeCmd, portMAX_DELAY);

    Serial.print("Auto home queued. Saved position: " );
    Serial.print(savedHomeOffsetAngle, 2);
    Serial.print(" deg | Move positive: " );
    Serial.print(homeCmd.angle, 2);
    Serial.println(" deg");
  } else {
    Serial.println("Auto home skipped: saved position is 0");
  }

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
  Serial.println("v3 fast defaults: SPEED=1500 step/s, MIN_SPEED=500 step/s");
  Serial.println("Config commands: CFG? / CFG_SAVE / CFG_RESET / PID? / SPEED? / MINSPEED? / ENCODER? / TOL?");
  Serial.println("Home commands: HOME? / POS? / HOME_ZERO / ZERO / HOME:60");
}

// =====================================================
// Loop
// =====================================================
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}