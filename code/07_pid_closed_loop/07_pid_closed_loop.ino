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
const float DEFAULT_PID_KP = 5.0f;
const float DEFAULT_PID_KI = 0.015f;
const float DEFAULT_PID_KD = 0.0f;

const float DEFAULT_PID_MAX_STEP_HZ = 550.0f;
const float DEFAULT_PID_MIN_STEP_HZ = 140.0f;
const float DEFAULT_PID_ACCEL = 4500.0f;

const float DEFAULT_ENCODER_COUNTS_PER_REV = 42.35f;
const float DEFAULT_ANGLE_TOLERANCE = 4.5f;

// =====================================================
// 运行时参数：开机从 Flash 读取，也可以通过命令修改
// =====================================================
float pidKp = DEFAULT_PID_KP;
float pidKi = DEFAULT_PID_KI;
float pidKd = DEFAULT_PID_KD;

float pidMaxStepHz = DEFAULT_PID_MAX_STEP_HZ;
float pidMinStepHz = DEFAULT_PID_MIN_STEP_HZ;
float pidAccelStepHzPerSec = DEFAULT_PID_ACCEL;

float encoderCountsPerRev = DEFAULT_ENCODER_COUNTS_PER_REV;
float encoderDegPerCount = 360.0f / DEFAULT_ENCODER_COUNTS_PER_REV;

float angleTolerance = DEFAULT_ANGLE_TOLERANCE;

// =====================================================
// 固定控制参数
// =====================================================
// I项只在小误差时启用，防止大角度运动时积分打满
#define PID_I_ENABLE_ERROR_DEG 36.0f
#define PID_INTEGRAL_LIMIT 80.0f

// 连续多少次进入误差范围才算到位
#define PID_SETTLE_COUNT 2

// 单次运动超时
#define PID_MOVE_TIMEOUT_MS 30000

// 步数保护
#define PID_MAX_STEP_MULTIPLIER 60
#define PID_STEP_GUARD_EXTRA 6000
#define PID_MIN_MAX_STEPS 10000

// 调试打印间隔，单位ms
#define PID_DEBUG_INTERVAL_MS 800

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
  return "CFG KP=" + String(pidKp, 4) +
         " KI=" + String(pidKi, 4) +
         " KD=" + String(pidKd, 4) +
         " MAXHZ=" + String(pidMaxStepHz, 1) +
         " MINHZ=" + String(pidMinStepHz, 1) +
         " ACCEL=" + String(pidAccelStepHzPerSec, 1) +
         " CPR=" + String(encoderCountsPerRev, 4) +
         " DEG_PER_COUNT=" + String(encoderDegPerCount, 4) +
         " TOL=" + String(angleTolerance, 2);
}

void loadConfigFromFlash() {
  prefs.begin("motorcfg", true);

  pidKp = prefs.getFloat("kp", DEFAULT_PID_KP);
  pidKi = prefs.getFloat("ki", DEFAULT_PID_KI);
  pidKd = prefs.getFloat("kd", DEFAULT_PID_KD);

  pidMaxStepHz = prefs.getFloat("maxhz", DEFAULT_PID_MAX_STEP_HZ);
  pidMinStepHz = prefs.getFloat("minhz", DEFAULT_PID_MIN_STEP_HZ);
  pidAccelStepHzPerSec = prefs.getFloat("accel", DEFAULT_PID_ACCEL);

  encoderCountsPerRev = prefs.getFloat("cpr", DEFAULT_ENCODER_COUNTS_PER_REV);
  angleTolerance = prefs.getFloat("tol", DEFAULT_ANGLE_TOLERANCE);

  prefs.end();

  updateDerivedParams();

  Serial.print("Loaded config from Flash: ");
  Serial.println(getConfigString());
}

void saveConfigToFlash() {
  prefs.begin("motorcfg", false);

  prefs.putFloat("kp", pidKp);
  prefs.putFloat("ki", pidKi);
  prefs.putFloat("kd", pidKd);

  prefs.putFloat("maxhz", pidMaxStepHz);
  prefs.putFloat("minhz", pidMinStepHz);
  prefs.putFloat("accel", pidAccelStepHzPerSec);

  prefs.putFloat("cpr", encoderCountsPerRev);
  prefs.putFloat("tol", angleTolerance);

  prefs.end();

  Serial.print("Saved config to Flash: ");
  Serial.println(getConfigString());
}

void resetConfigToDefault() {
  pidKp = DEFAULT_PID_KP;
  pidKi = DEFAULT_PID_KI;
  pidKd = DEFAULT_PID_KD;

  pidMaxStepHz = DEFAULT_PID_MAX_STEP_HZ;
  pidMinStepHz = DEFAULT_PID_MIN_STEP_HZ;
  pidAccelStepHzPerSec = DEFAULT_PID_ACCEL;

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

// 误差越大，最低速度越高；防止后半段太慢
float getAdaptiveMinStepHz(float absError) {
  if (absError >= 120.0f) {
    return 520.0f;
  } else if (absError >= 90.0f) {
    return 480.0f;
  } else if (absError >= 45.0f) {
    return 400.0f;
  } else if (absError >= 18.0f) {
    return 280.0f;
  } else if (absError >= 9.0f) {
    return 180.0f;
  } else {
    return pidMinStepHz;
  }
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
           " KI=" + String(pidKi, 4) +
           " KD=" + String(pidKd, 4);
  }

  // =====================================================
  // PID 设置：PID:Kp,Ki,Kd
  // 例子：PID:5.0,0.015,0.0
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

      if (isValidNumber(kpStr) && isValidNumber(kiStr) && isValidNumber(kdStr)) {
        float newKp = kpStr.toFloat();
        float newKi = kiStr.toFloat();
        float newKd = kdStr.toFloat();

        if (newKp < 0 || newKp > 100 ||
            newKi < 0 || newKi > 10 ||
            newKd < 0 || newKd > 10) {
          return "ERR PID RANGE";
        }

        pidKp = newKp;
        pidKi = newKi;
        pidKd = newKd;

        return "OK PID KP=" + String(pidKp, 4) +
               " KI=" + String(pidKi, 4) +
               " KD=" + String(pidKd, 4);
      }
    }

    return "ERR PID FORMAT";
  }

  // =====================================================
  // 速度查询
  // =====================================================
  if (cmd == "SPEED?") {
    return "SPEED MAX=" + String(pidMaxStepHz, 1) +
           " MIN=" + String(pidMinStepHz, 1) +
           " ACCEL=" + String(pidAccelStepHzPerSec, 1);
  }

  // =====================================================
  // 速度设置：SPEED:maxHz,minHz,accel
  // 例子：SPEED:550,140,4500
  // 只改 RAM，不立刻写 Flash
  // =====================================================
  if (cmd.startsWith("SPEED:")) {
    String data = cmd.substring(6);
    data.trim();

    int p1 = data.indexOf(',');
    int p2 = data.indexOf(',', p1 + 1);

    if (p1 > 0 && p2 > p1) {
      String maxStr = data.substring(0, p1);
      String minStr = data.substring(p1 + 1, p2);
      String accStr = data.substring(p2 + 1);

      maxStr.trim();
      minStr.trim();
      accStr.trim();

      if (isValidNumber(maxStr) && isValidNumber(minStr) && isValidNumber(accStr)) {
        float newMaxHz = maxStr.toFloat();
        float newMinHz = minStr.toFloat();
        float newAccel = accStr.toFloat();

        if (newMaxHz < 1 || newMaxHz > 3000 ||
            newMinHz < 1 || newMinHz > newMaxHz ||
            newAccel < 1 || newAccel > 30000) {
          return "ERR SPEED RANGE";
        }

        pidMaxStepHz = newMaxHz;
        pidMinStepHz = newMinHz;
        pidAccelStepHzPerSec = newAccel;

        return "OK SPEED MAX=" + String(pidMaxStepHz, 1) +
               " MIN=" + String(pidMinStepHz, 1) +
               " ACCEL=" + String(pidAccelStepHzPerSec, 1);
      }
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
  // 编码器比例设置：ENCODER:42.35
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
// PI / PID Closed Loop Motor Task
// =====================================================
void closedLoopTask(void* pvParameters) {
  float moveAngle;

  while (true) {
    if (xQueueReceive(targetQueue, &moveAngle, portMAX_DELAY)) {

      float startAngle = getRealAngleTotal();
      float targetAngle = startAngle + moveAngle;

      Serial.print("PI Move: ");
      Serial.print(moveAngle);
      Serial.print(" deg | Start: ");
      Serial.print(startAngle);
      Serial.print(" deg | Target: ");
      Serial.println(targetAngle);

      float integral = 0.0f;
      float lastError = targetAngle - getRealAngleTotal();
      float currentStepHz = 0.0f;

      long stepCounter = 0;
      long expectedSteps = lround(fabs(moveAngle) / STEP_ANGLE);

      long maxAllowedSteps =
        expectedSteps * PID_MAX_STEP_MULTIPLIER + PID_STEP_GUARD_EXTRA;

      if (maxAllowedSteps < PID_MIN_MAX_STEPS) {
        maxAllowedSteps = PID_MIN_MAX_STEPS;
      }

      int settleCounter = 0;

      unsigned long startMs = millis();
      unsigned long lastMicros = micros();
      unsigned long lastDebugMs = millis();
      unsigned long lastYieldMs = millis();

      while (true) {
        float currentAngle = getRealAngleTotal();
        float error = targetAngle - currentAngle;
        float absError = fabs(error);

        // =====================================================
        // 到位判断
        // =====================================================
        if (absError <= angleTolerance) {
          settleCounter++;
          currentStepHz = 0.0f;
          integral = 0.0f;

          if (settleCounter >= PID_SETTLE_COUNT) {
            Serial.print("Arrived. Angle: ");
            Serial.print(currentAngle);
            Serial.print(" deg | Error: ");
            Serial.print(error);
            Serial.print(" deg | Steps: ");
            Serial.println(stepCounter);
            break;
          }

          vTaskDelay(pdMS_TO_TICKS(20));
          continue;
        } else {
          settleCounter = 0;
        }

        // =====================================================
        // 超时保护
        // =====================================================
        if (millis() - startMs > PID_MOVE_TIMEOUT_MS) {
          Serial.print("ERR: PI move timeout | Current: ");
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
          Serial.println("Check STEP_ANGLE / microstep / motor direction / encoder direction");
          break;
        }

        // =====================================================
        // dt
        // =====================================================
        unsigned long nowMicros = micros();
        float dt = (nowMicros - lastMicros) / 1000000.0f;
        lastMicros = nowMicros;

        if (dt <= 0.0005f) {
          dt = 0.0005f;
        }

        // =====================================================
        // I项处理
        // =====================================================
        if (error * lastError < 0) {
          // 越过目标，清积分
          integral = 0.0f;
          currentStepHz = 0.0f;
        }

        float derivative = (error - lastError) / dt;
        lastError = error;

        if (absError <= PID_I_ENABLE_ERROR_DEG) {
          integral += error * dt;
          integral = constrain(
            integral,
            -PID_INTEGRAL_LIMIT,
            PID_INTEGRAL_LIMIT
          );
        } else {
          integral = 0.0f;
        }

        // =====================================================
        // PI / PID 输出
        // =====================================================
        float pidOutputDegPerSec =
          pidKp * error +
          pidKi * integral +
          pidKd * derivative;

        float maxSpeedDegPerSec = pidMaxStepHz * STEP_ANGLE;

        pidOutputDegPerSec = constrain(
          pidOutputDegPerSec,
          -maxSpeedDegPerSec,
          maxSpeedDegPerSec
        );

        bool dir = pidOutputDegPerSec > 0;

        float targetStepHz = fabs(pidOutputDegPerSec) / STEP_ANGLE;

        // 自适应最低速度：解决后半段太慢
        float minStepHz = getAdaptiveMinStepHz(absError);

        if (targetStepHz < minStepHz) {
          targetStepHz = minStepHz;
        }

        if (targetStepHz > pidMaxStepHz) {
          targetStepHz = pidMaxStepHz;
        }

        // =====================================================
        // 加速度限制
        // =====================================================
        float maxDeltaHz = pidAccelStepHzPerSec * dt;

        if (targetStepHz > currentStepHz + maxDeltaHz) {
          currentStepHz += maxDeltaHz;
        } else if (targetStepHz < currentStepHz - maxDeltaHz) {
          currentStepHz -= maxDeltaHz;
        } else {
          currentStepHz = targetStepHz;
        }

        if (currentStepHz < minStepHz) {
          currentStepHz = minStepHz;
        }

        if (currentStepHz > pidMaxStepHz) {
          currentStepHz = pidMaxStepHz;
        }

        uint32_t halfPeriodUs =
          (uint32_t)(1000000.0f / currentStepHz / 2.0f);

        if (halfPeriodUs < 50) {
          halfPeriodUs = 50;
        }

        oneStepWithDelay(dir, halfPeriodUs);
        stepCounter++;

        // =====================================================
        // 调试打印，按时间打印，避免串口拖慢
        // =====================================================
        unsigned long nowMs = millis();

        if (nowMs - lastDebugMs >= PID_DEBUG_INTERVAL_MS) {
          lastDebugMs = nowMs;

          Serial.print("PI Debug | Current: ");
          Serial.print(currentAngle);
          Serial.print(" deg | Error: ");
          Serial.print(error);
          Serial.print(" deg | I: ");
          Serial.print(integral);
          Serial.print(" | Hz: ");
          Serial.print(currentStepHz);
          Serial.print(" | Dir: ");
          Serial.println(dir ? "POS" : "NEG");
        }

        // =====================================================
        // 主动让出CPU，防止 Task Watchdog 重启
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
    "PI Closed Loop",
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
  Serial.println("Config commands: CFG? / CFG_SAVE / CFG_RESET / PID? / SPEED? / ENCODER? / TOL?");
}

// =====================================================
// Loop
// =====================================================
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}