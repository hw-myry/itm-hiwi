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

// =====================================================
// 找零 / Flash 位置保存功能开关
// =====================================================
// ENABLE_AUTO_HOME_FLASH = 1：开启 HOME_POS 写入 Flash，重启后自动用 -HOME_POS 直接反向找零
// ENABLE_AUTO_HOME_FLASH = 0：关闭 HOME_POS 写入 Flash，也不执行开机自动找零
#define ENABLE_AUTO_HOME_FLASH 1

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
// Flash 当前位置 / 反向直接回零参数
// =====================================================
// savedHomeOffsetAngle 表示“当前位置相对零点的有符号角度”。
// 约定：右转为正，左转为负，保存时保留方向，不再用 360 - HOME_POS 回零。
// 例如：右转 60 => HOME_POS=60；重启后自动左转 -60 回零。
// 例如：左转 -60 => HOME_POS=-60；重启后自动右转 60 回零。
// 例如：右转 120 再左转 60 => HOME_POS=60；重启后自动左转 -60 回零。
const char* FLASH_KEY_HOME_ANGLE = "homeang";
const float AUTO_HOME_MIN_ANGLE = 0.5f;      // 小于这个角度就认为不用回零
const float POSITION_ZERO_EPS = 0.5f;        // 接近0或整圈时保存成0

// =====================================================
// 反向间隙补偿参数
// =====================================================
// 如果本次运动方向和上一次运动方向相反，就在本次实际执行角度上多走这个角度。
// 例如：上一次 +90，本次命令 -90，实际执行 -95，用多出的 5 度补机械间隙。
// 注意：Flash 里的 HOME_POS 仍按用户命令角度更新，不把补偿的 5 度算进去。
const char* FLASH_KEY_BACKLASH = "backlash";
const float DEFAULT_BACKLASH_COMP_ANGLE = 5.0f;
const float DIRECTION_DEADBAND_ANGLE = 0.01f;

// =====================================================
// LED
// =====================================================
#define LED_PIN 38
#define LED_COUNT 1

Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
bool ledEnabled = false;

// =====================================================
// Motor
// =====================================================
#define STEP_PIN 39
#define DIR_PIN 40

#define ENCODE_CLK 5
#define ENCODE_DT 6
#define ENCODE_SW 7

// 如果驱动器没有细分，1个STEP = 1.8度
// 如果16细分，改成：1.8 / 16.0
const float STEP_ANGLE = 1.8f ;

// =====================================================
// 默认参数：Flash 没有保存过时使用
// =====================================================
const float DEFAULT_KP = 5.0f;
const float DEFAULT_KI = 0.015f;
const float DEFAULT_KD = 0.0f;

const float DEFAULT_SPEED_HZ = 500.0f;
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

// 反向间隙补偿角度，默认 5 度，可通过 BACKLASH:5 修改，再用 CFG_SAVE 保存
float backlashCompAngle = DEFAULT_BACKLASH_COMP_ANGLE;

// 保存到 Flash 的当前位置角度：有符号角度，正数表示在零点正方向，负数表示在零点反方向
float savedHomeOffsetAngle = 0.0f;

// 记录上一次“用户命令方向”
//  1 = 正转 / 右转
// -1 = 反转 / 左转
//  0 = 还没有有效运动，第一次运动不做间隙补偿
int lastMoveDir = 0;

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
// 上位机异步通知参数
// =====================================================
// 电机任务只把完成消息放进队列，不直接写 TCP，避免 WiFi 发送阻塞电机控制任务
#define NOTIFY_QUEUE_LENGTH 10
#define NOTIFY_TEXT_LEN 256

// 相对运动残差补偿限幅。
// 闭环到位有容差，连续 120+120+120 时，每次少走的几度会自动补到下一次。
// 限幅避免编码器异常时补偿过大。
#define MOTION_RESIDUAL_LIMIT_ANGLE 30.0f

// =====================================================
// FreeRTOS
// =====================================================
struct MotorMoveCommand {
  float angle;          // 相对运动角度，右转为正，左转为负
  bool zeroAfterMove;   // 这次运动完成后是否把累计角度清零
  bool savePosition;    // 这次运动结束后是否更新 Flash 里的累计角度
};

struct UpperNotifyMessage {
  char text[NOTIFY_TEXT_LEN];
};

QueueHandle_t targetQueue;
QueueHandle_t notifyQueue;

SemaphoreHandle_t tcpClientMutex;
WiFiClient activeClient;
bool activeClientValid = false;

// 连续相对运动的累计残差：正数表示前面少走了，需要下一次多补；负数表示前面多走了，需要下一次少走。
float motionResidualAngle = 0.0f;

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

// HOME_POS 专用归一化：保留正负方向，只去掉整圈。
// Arduino/C 的 fmod 对负数会保留负号：fmod(-420, 360) = -60。
// 这样自动回零时可以直接走 -HOME_POS，而不是 360 - HOME_POS。
float normalizeHomeOffsetAngle(float angle) {
  float a = fmod(angle, 360.0f);

  if (fabs(a) <= POSITION_ZERO_EPS ||
      fabs(fabs(a) - 360.0f) <= POSITION_ZERO_EPS) {
    return 0.0f;
  }

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

int getMoveDirection(float angle) {
  if (angle > DIRECTION_DEADBAND_ANGLE) {
    return 1;
  }

  if (angle < -DIRECTION_DEADBAND_ANGLE) {
    return -1;
  }

  return 0;
}

String getDirectionName(int dir) {
  if (dir > 0) {
    return "POS";
  }

  if (dir < 0) {
    return "NEG";
  }

  return "NONE";
}

void limitMotionResidual() {
  if (motionResidualAngle > MOTION_RESIDUAL_LIMIT_ANGLE) {
    motionResidualAngle = MOTION_RESIDUAL_LIMIT_ANGLE;
  }

  if (motionResidualAngle < -MOTION_RESIDUAL_LIMIT_ANGLE) {
    motionResidualAngle = -MOTION_RESIDUAL_LIMIT_ANGLE;
  }
}

float getResidualCompensationForMove(float requestedAngle) {
  int dir = getMoveDirection(requestedAngle);

  if (dir == 0) {
    return 0.0f;
  }

  float comp = motionResidualAngle;

  if (comp > MOTION_RESIDUAL_LIMIT_ANGLE) {
    comp = MOTION_RESIDUAL_LIMIT_ANGLE;
  }

  if (comp < -MOTION_RESIDUAL_LIMIT_ANGLE) {
    comp = -MOTION_RESIDUAL_LIMIT_ANGLE;
  }

  // 不允许补偿把本次运动方向反过来，避免小角度命令被历史残差带反。
  if (getMoveDirection(requestedAngle + comp) != dir) {
    return 0.0f;
  }

  return comp;
}

String getMotionResidualString() {
  return "RESIDUAL=" + String(motionResidualAngle, 2) +
         " LIMIT=" + String(MOTION_RESIDUAL_LIMIT_ANGLE, 2);
}

void enqueueUpperNotify(String msg) {
  if (notifyQueue == NULL) {
    Serial.print("WARN: notify queue not ready, drop: ");
    Serial.println(msg);
    return;
  }

  UpperNotifyMessage item;
  memset(&item, 0, sizeof(item));
  msg.toCharArray(item.text, NOTIFY_TEXT_LEN);

  if (xQueueSend(notifyQueue, &item, 0) != pdTRUE) {
    Serial.print("WARN: notify queue full, drop: ");
    Serial.println(item.text);
  }
}

void sendTcpLineToActiveClient(String line) {
  if (tcpClientMutex == NULL) {
    return;
  }

  if (xSemaphoreTake(tcpClientMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (activeClientValid && activeClient && activeClient.connected()) {
      activeClient.println(line);
      activeClient.flush();
    }

    xSemaphoreGive(tcpClientMutex);
  }
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
         " AUTO_HOME_FLASH=" + String(ENABLE_AUTO_HOME_FLASH == 1 ? "ON" : "OFF") +
         " HOME_POS=" + String(savedHomeOffsetAngle, 2) +
         " BACKLASH=" + String(backlashCompAngle, 2) +
         " LAST_DIR=" + getDirectionName(lastMoveDir) +
         " RESIDUAL=" + String(motionResidualAngle, 2);
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
  backlashCompAngle = prefs.getFloat(FLASH_KEY_BACKLASH, DEFAULT_BACKLASH_COMP_ANGLE);

#if ENABLE_AUTO_HOME_FLASH == 1
  savedHomeOffsetAngle = prefs.getFloat(FLASH_KEY_HOME_ANGLE, 0.0f);
#else
  savedHomeOffsetAngle = 0.0f;
#endif

  prefs.end();

#if ENABLE_AUTO_HOME_FLASH == 1
  if (savedHomeOffsetAngle < -64800.0f || savedHomeOffsetAngle > 64800.0f) {
    Serial.println("Saved home position out of range, reset to 0");
    savedHomeOffsetAngle = 0.0f;
  }

  savedHomeOffsetAngle = normalizeHomeOffsetAngle(savedHomeOffsetAngle);
#else
  Serial.println("Auto home Flash position tracking disabled by ENABLE_AUTO_HOME_FLASH=0");
#endif

  if (backlashCompAngle < 0.0f || backlashCompAngle > 45.0f) {
    backlashCompAngle = DEFAULT_BACKLASH_COMP_ANGLE;
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
  prefs.putFloat(FLASH_KEY_BACKLASH, backlashCompAngle);
#if ENABLE_AUTO_HOME_FLASH == 1
  prefs.putFloat(FLASH_KEY_HOME_ANGLE, savedHomeOffsetAngle);
#endif

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
  backlashCompAngle = DEFAULT_BACKLASH_COMP_ANGLE;
  savedHomeOffsetAngle = 0.0f;
  lastMoveDir = 0;
  motionResidualAngle = 0.0f;

  updateDerivedParams();

  prefs.begin("motorcfg", false);
  prefs.clear();
  prefs.end();

  Serial.println("Config reset to default");
}

float getAutoHomeMoveAngle() {
#if ENABLE_AUTO_HOME_FLASH == 1
  float pos = normalizeHomeOffsetAngle(savedHomeOffsetAngle);

  if (fabs(pos) <= POSITION_ZERO_EPS) {
    return 0.0f;
  }

  // 直接往当前位置的反方向回零：
  // HOME_POS=60  -> HOME_BACK=-60
  // HOME_POS=-60 -> HOME_BACK=60
  return -pos;
#else
  return 0.0f;
#endif
}

String getHomeString() {
  return "AUTO_HOME_FLASH=" + String(ENABLE_AUTO_HOME_FLASH == 1 ? "ON" : "OFF") +
         " HOME_POS=" + String(savedHomeOffsetAngle, 2) +
         " HOME_BACK=" + String(getAutoHomeMoveAngle(), 2) +
         " BACKLASH=" + String(backlashCompAngle, 2) +
         " LAST_DIR=" + getDirectionName(lastMoveDir) +
         " RESIDUAL=" + String(motionResidualAngle, 2) +
         " ENCODER=" + String(getEncoderCount()) +
         " CURRENT_ANGLE=" + String(getRealAngleTotal(), 2);
}

void saveHomeOffsetToFlash() {
#if ENABLE_AUTO_HOME_FLASH == 1
  savedHomeOffsetAngle = normalizeHomeOffsetAngle(savedHomeOffsetAngle);

  prefs.begin("motorcfg", false);
  prefs.putUInt("ver", CONFIG_VERSION);
  prefs.putFloat(FLASH_KEY_HOME_ANGLE, savedHomeOffsetAngle);
  prefs.end();

  Serial.print("Saved home position to Flash: ");
  Serial.println(savedHomeOffsetAngle, 2);
#else
  savedHomeOffsetAngle = 0.0f;
  Serial.println("Auto home Flash disabled, home position not written to Flash");
#endif
}

void updateSavedHomeOffsetByActualDelta(float actualDelta) {
#if ENABLE_AUTO_HOME_FLASH == 1
  savedHomeOffsetAngle = normalizeHomeOffsetAngle(savedHomeOffsetAngle + actualDelta);
  saveHomeOffsetToFlash();
#else
  savedHomeOffsetAngle = 0.0f;
#endif
}

void clearSavedHomeOffset(bool alsoResetEncoder) {
  savedHomeOffsetAngle = 0.0f;

  if (alsoResetEncoder) {
    setEncoderCount(0);
    lastMoveDir = 0;
    motionResidualAngle = 0.0f;
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
  // 反向间隙补偿查询
  // =====================================================
  if (cmd == "BACKLASH?") {
    return "BACKLASH=" + String(backlashCompAngle, 2) +
           " LAST_DIR=" + getDirectionName(lastMoveDir);
  }

  // =====================================================
  // 反向间隙补偿设置
  // 命令：BACKLASH:10
  // 只改 RAM，不立刻写 Flash；需要长期保存就再发 CFG_SAVE
  // =====================================================
  if (cmd.startsWith("BACKLASH:")) {
    String data = cmd.substring(9);
    data.trim();

    if (isValidNumber(data)) {
      float newBacklash = data.toFloat();

      if (newBacklash < 0.0f || newBacklash > 45.0f) {
        return "ERR BACKLASH RANGE";
      }

      backlashCompAngle = newBacklash;

      return "OK BACKLASH=" + String(backlashCompAngle, 2) +
             " LAST_DIR=" + getDirectionName(lastMoveDir);
    }

    return "ERR BACKLASH FORMAT";
  }

  // =====================================================
  // 当前角度查询
  // =====================================================
  if (cmd == "ANGLE?") {
    long enc = getEncoderCount();
    float angle = getRealAngleTotal();

    return "ANGLE CURRENT=" + String(angle, 2) +
           " ENCODER=" + String(enc) +
           " HOME_POS=" + String(savedHomeOffsetAngle, 2) +
           " LAST_DIR=" + getDirectionName(lastMoveDir);
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
  // 命令：HOME:60 表示当前位置在零点后方正方向 60 度；重启后会反向走 -60 度回零
  // 命令：HOME:-60 表示当前位置在零点后方反方向 60 度；重启后会反向走 60 度回零
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

#if ENABLE_AUTO_HOME_FLASH == 1
      savedHomeOffsetAngle = normalizeHomeOffsetAngle(newHome);
      saveHomeOffsetToFlash();

      return "OK HOME " + getHomeString();
#else
      savedHomeOffsetAngle = 0.0f;
      return "ERR HOME DISABLED ENABLE_AUTO_HOME_FLASH=0";
#endif
    }

    return "ERR HOME FORMAT";
  }

  // =====================================================
  // 相对运动残差查询 / 清零
  // =====================================================
  if (cmd == "RESIDUAL?") {
    return getMotionResidualString();
  }

  if (cmd == "RESIDUAL_ZERO") {
    motionResidualAngle = 0.0f;
    return "OK RESIDUAL_ZERO " + getMotionResidualString();
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
#if ENABLE_AUTO_HOME_FLASH == 1
    moveCmd.savePosition = true;
#else
    moveCmd.savePosition = false;
#endif

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

      if (xSemaphoreTake(tcpClientMutex, portMAX_DELAY) == pdTRUE) {
        activeClient = client;
        activeClientValid = true;
        xSemaphoreGive(tcpClientMutex);
      }

      sendTcpLineToActiveClient("ESP32 READY");

      while (client.connected()) {
        if (client.available()) {
          String cmd = client.readStringUntil('\n');
          cmd.trim();

          Serial.print("WiFi Recv: ");
          Serial.println(cmd);

          String reply = handleCommand(cmd);

          sendTcpLineToActiveClient(reply);

          Serial.print("Reply: ");
          Serial.println(reply);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
      }

      if (xSemaphoreTake(tcpClientMutex, portMAX_DELAY) == pdTRUE) {
        activeClientValid = false;
        activeClient.stop();
        activeClient = WiFiClient();
        xSemaphoreGive(tcpClientMutex);
      }

      client.stop();
      Serial.println("Client Disconnected");
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// =====================================================
// 上位机通知 Task
// =====================================================
void upperNotifyTask(void* pvParameters) {
  UpperNotifyMessage msg;

  while (true) {
    if (xQueueReceive(notifyQueue, &msg, portMAX_DELAY)) {
      // 串口也打印一份，方便调试；TCP 发送由本任务异步完成，不阻塞电机任务。
      Serial.println(msg.text);
      sendTcpLineToActiveClient(String(msg.text));
    }
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

      // requestedAngle 是用户/自动回零请求的逻辑角度。
      // moveAngle 是实际送给 PID 执行的角度，可能已经加了反向间隙补偿。
      float requestedAngle = moveCmd.angle;
      int currentMoveDir = getMoveDirection(requestedAngle);

      float residualBeforeMove = motionResidualAngle;
      float residualComp = 0.0f;

      if (!moveCmd.zeroAfterMove) {
        residualComp = getResidualCompensationForMove(requestedAngle);
      }

      float backlashComp = 0.0f;
      bool backlashApplied = false;

      if (!moveCmd.zeroAfterMove && currentMoveDir != 0 &&
          lastMoveDir != 0 && currentMoveDir != lastMoveDir) {
        backlashComp = backlashCompAngle * currentMoveDir;
        backlashApplied = true;
      }

      float compensatedRequestAngle = requestedAngle + residualComp;
      float moveAngle = compensatedRequestAngle + backlashComp;
      float startAngle = getRealAngleTotal();
      float targetAngle = startAngle + moveAngle;

      Serial.print("PID Move: ");
      Serial.print(moveAngle);
      Serial.print(" deg | Requested: ");
      Serial.print(requestedAngle);
      Serial.print(" deg | ResidualComp: ");
      Serial.print(residualComp);
      Serial.print(" deg | ResidualBefore: ");
      Serial.print(residualBeforeMove);
      Serial.print(" deg | BacklashComp: ");
      Serial.print(backlashComp);
      Serial.print(" deg | LastDir: ");
      Serial.print(getDirectionName(lastMoveDir));
      Serial.print(" | NewDir: ");
      Serial.print(getDirectionName(currentMoveDir));
      Serial.print(" | Start: ");
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
      String stopReason = "UNKNOWN";
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
          stopReason = "OK";

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
          stopReason = "TIMEOUT";
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
          stopReason = "STEP_GUARD";
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
      float actualDelta = finalAngle - startAngle;
      float logicalActualDelta = actualDelta - backlashComp;

      if (!moveCmd.zeroAfterMove && currentMoveDir != 0) {
        if (arrived) {
          motionResidualAngle = residualBeforeMove + requestedAngle - logicalActualDelta;
          limitMotionResidual();
        } else {
          // 运动失败时不要保留历史残差，避免下一次补偿异常放大。
          motionResidualAngle = 0.0f;
        }
      }

      if (moveCmd.savePosition) {

        if (moveCmd.zeroAfterMove && arrived) {
          // 自动回零完成：Flash 归零，编码器计数也归零
          clearSavedHomeOffset(true);
          lastMoveDir = getMoveDirection(moveAngle);
          Serial.println("Auto home finished, home position cleared to 0");
        } else {
          // 普通运动：Flash 的 HOME_POS 按“用户命令角度”更新。
          // 反向补偿多走的角度只用于吃掉机械间隙，不计入零点位置。
          float logicalDelta;

          if (arrived) {
            logicalDelta = requestedAngle;
          } else {
            // 运动失败时，用实际编码器增量减去已加的补偿，尽量避免位置完全不变。
            logicalDelta = actualDelta - backlashComp;
          }

          updateSavedHomeOffsetByActualDelta(logicalDelta);

          if (currentMoveDir != 0 && arrived) {
            lastMoveDir = currentMoveDir;
          }

          Serial.print("Home position updated by logical delta: ");
          Serial.print(logicalDelta, 2);
          Serial.print(" deg | Actual motor delta: ");
          Serial.print(actualDelta, 2);
          Serial.print(" deg | Backlash applied: ");
          Serial.print(backlashApplied ? "YES" : "NO");
          Serial.print(" | New home position: ");
          Serial.print(savedHomeOffsetAngle, 2);
          Serial.print(" | LastDir: ");
          Serial.println(getDirectionName(lastMoveDir));
        }
      } else {
        // 关闭 Flash 找零时，也要继续记录上一次方向，保证反向间隙补偿仍然生效。
        if (!moveCmd.zeroAfterMove && currentMoveDir != 0 && arrived) {
          lastMoveDir = currentMoveDir;
          Serial.print("Position save disabled, LastDir updated: ");
          Serial.println(getDirectionName(lastMoveDir));
        }
      }

      String doneMsg =
        String("MOTOR_DONE STATUS=") + stopReason +
        " REQUEST=" + String(requestedAngle, 2) +
        " RESIDUAL_COMP=" + String(residualComp, 2) +
        " MOVE=" + String(moveAngle, 2) +
        " ACTUAL=" + String(actualDelta, 2) +
        " LOGICAL_ACTUAL=" + String(logicalActualDelta, 2) +
        " FINAL=" + String(finalAngle, 2) +
        " ERROR=" + String(targetAngle - finalAngle, 2) +
        " STEPS=" + String(stepCounter) +
        " RESIDUAL=" + String(motionResidualAngle, 2) +
        " HOME_POS=" + String(savedHomeOffsetAngle, 2) +
        " LAST_DIR=" + getDirectionName(lastMoveDir);

      enqueueUpperNotify(doneMsg);
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

    Serial.print(" | LastDir: ");
    Serial.print(getDirectionName(lastMoveDir));

    Serial.print(" | Residual: ");
    Serial.print(motionResidualAngle);

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
  notifyQueue = xQueueCreate(NOTIFY_QUEUE_LENGTH, sizeof(UpperNotifyMessage));
  tcpClientMutex = xSemaphoreCreateMutex();

  if (targetQueue == NULL || notifyQueue == NULL || tcpClientMutex == NULL) {
    Serial.println("ERR: queue/mutex create failed");
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
    upperNotifyTask,
    "Upper Notify Task",
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

#if ENABLE_AUTO_HOME_FLASH == 1
  // 开机自动回零：如果 Flash 里保存了当前位置，就直接往 HOME_POS 的反方向回零
  float autoHomeAngle = getAutoHomeMoveAngle();

  if (fabs(autoHomeAngle) > AUTO_HOME_MIN_ANGLE) {
    MotorMoveCommand homeCmd;
    homeCmd.angle = autoHomeAngle;
    homeCmd.zeroAfterMove = true;
    homeCmd.savePosition = true;

    xQueueSend(targetQueue, &homeCmd, portMAX_DELAY);

    Serial.print("Auto home queued. Saved position: " );
    Serial.print(savedHomeOffsetAngle, 2);
    Serial.print(" deg | Move back: " );
    Serial.print(homeCmd.angle, 2);
    Serial.println(" deg");
  } else {
    Serial.println("Auto home skipped: saved position is 0");
  }
#else
  Serial.println("Auto home disabled: ENABLE_AUTO_HOME_FLASH=0");
#endif

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
  Serial.println("Config commands: CFG? / CFG_SAVE / CFG_RESET / PID? / SPEED? / MINSPEED? / ENCODER? / TOL? / BACKLASH?");
  Serial.println("Home commands: HOME? / POS? / HOME_ZERO / ZERO / HOME:60");
  Serial.println("Set ENABLE_AUTO_HOME_FLASH to 1 to enable Flash home, 0 to disable it");
  Serial.println("Backlash commands: BACKLASH? / BACKLASH:5, use CFG_SAVE to persist");
  Serial.println("Residual commands: RESIDUAL? / RESIDUAL_ZERO");
}

// =====================================================
// Loop
// =====================================================
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}