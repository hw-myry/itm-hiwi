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

// 配置版本：v4 增加 ANGLE_LIMIT Flash 参数，同时保留 v3 的速度升级逻辑
const uint32_t CONFIG_VERSION = 4;

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
// 绝对角度限位参数
// =====================================================
// maxAbsAngleLimit 表示 Current Angle 允许到达的最大绝对值。
// 例如 LIMIT=60：
//   Current=0，命令 60  => 目标 60，允许
//   Current=60，命令 -130 => 目标 -70，拒绝
//   Current=30，命令 40 => 目标 70，拒绝
// 设置为 0 表示关闭限位；修改后用 CFG_SAVE 保存到 Flash。
const char* FLASH_KEY_MAX_ABS_ANGLE = "maxabs";
const float DEFAULT_MAX_ABS_ANGLE_LIMIT = 60.0f;
const float ANGLE_LIMIT_EPS = 0.01f;

// =====================================================
// LED
// =====================================================

#define USE_NEOPIXEL_LED 1
#define LED_PIN 38
#define LED_COUNT 1

#if USE_NEOPIXEL_LED == 1
Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
#endif

bool ledEnabled = false;

// =====================================================
// Motor
// =====================================================
#define STEP_PIN 39
#define DIR_PIN 40

// A4988 EN / ENABLE 引脚：LOW = 使能输出，HIGH = 禁止输出。

#define EN_PIN 41
#define A4988_ENABLE_ACTIVE_LOW 1

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

// =====================================================
// 编码器 / 输出角度换算
// =====================================================
// 你的EC11实测：1 count = 4.5度，所以EC11本身一圈 = 80 count。
// 但当前机械结构里：实际输出轴角度 ≈ EC11编码器角度 × 2。
// 因此：
//   EC11角度 = encoderCount × 4.5
//   输出角度 = EC11角度 × 2
// 这样上位机发送 90 时，程序按输出角度闭环，实际输出约90度。
const float FIXED_ENCODER_DEG_PER_COUNT = 4.5f;
const float OUTPUT_ANGLE_PER_ENCODER_ANGLE = 2.0f;
const float DEFAULT_ENCODER_COUNTS_PER_REV = 360.0f / FIXED_ENCODER_DEG_PER_COUNT;  // 80 count/rev
const float DEFAULT_ANGLE_TOLERANCE = 4.5f;  // 输出角度容差，约等于半个输出count

// =====================================================
// 运行时参数：开机从 Flash 读取，也可以通过命令修改
// =====================================================
float pidKp = DEFAULT_KP;
float pidKi = DEFAULT_KI;
float pidKd = DEFAULT_KD;

float speedHz = DEFAULT_SPEED_HZ;
float minSpeedHz = DEFAULT_MIN_SPEED_HZ;

float encoderCountsPerRev = DEFAULT_ENCODER_COUNTS_PER_REV;
float encoderDegPerCount = FIXED_ENCODER_DEG_PER_COUNT;

float angleTolerance = DEFAULT_ANGLE_TOLERANCE;

// 反向间隙补偿角度，默认 5 度，可通过 BACKLASH:5 修改，再用 CFG_SAVE 保存
float backlashCompAngle = DEFAULT_BACKLASH_COMP_ANGLE;

// Current Angle 的绝对值限位，默认 60 度；0 表示关闭限位。
// 可通过 LIMIT:60 / ANGLE_LIMIT:60 修改，再用 CFG_SAVE 保存。
float maxAbsAngleLimit = DEFAULT_MAX_ABS_ANGLE_LIMIT;

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
#define NOTIFY_TEXT_LEN 512

// 相对运动残差补偿限幅。
// 闭环到位有容差，连续 120+120+120 时，每次少走的几度会自动补到下一次。
// 限幅避免编码器异常时补偿过大。
#define MOTION_RESIDUAL_LIMIT_ANGLE 30.0f

// =====================================================
// 运动结束后的开环微调
// =====================================================
// 主PID停下后，再比较一次 targetAngle 和 Current Angle。
// 如果偏差仍然超过 POST_CORRECT_TOLERANCE，就不用PID，按偏差角度开环补转一次。
// 注意：你的当前输出角度分辨率约为 9 度/count，所以默认容差设为 4.5 度。
#define POST_CORRECT_ENABLE 1
#define POST_CORRECT_MAX_ROUNDS 1
const float POST_CORRECT_TOLERANCE = 8.9f;
const float POST_CORRECT_MAX_ANGLE = 20.0f;
const float POST_CORRECT_SPEED_HZ = 300.0f;
const uint32_t POST_CORRECT_SETTLE_MS = 120;

// =====================================================
// FreeRTOS
// =====================================================
struct MotorMoveCommand {
  float angle;          // 相对运动角度，右转为正，左转为负
  bool zeroAfterMove;   // 这次运动完成后是否把累计角度清零
  bool savePosition;    // 这次运动结束后是否更新 Flash 里的累计角度
  bool resumeMove;      // true：这是 STOP 后的剩余运动，不再叠加残差/反向间隙补偿
};

struct UpperNotifyMessage {
  char text[NOTIFY_TEXT_LEN];
};

QueueHandle_t targetQueue;
QueueHandle_t notifyQueue;

// =====================================================
// 多客户端 TCP
// =====================================================
#define MAX_TCP_CLIENTS 4
#define TCP_RX_BUFFER_LIMIT 512

SemaphoreHandle_t tcpClientsMutex;
WiFiClient tcpClients[MAX_TCP_CLIENTS];
String tcpRxBuffers[MAX_TCP_CLIENTS];

// =====================================================
// 电机运行 / 普通停止 / 恢复状态
// =====================================================
volatile bool motorMoving = false;
volatile bool normalStopRequested = false;
volatile bool motionPaused = false;

// STOP 后保存的剩余任务。ESTOP 会清除它。
MotorMoveCommand pausedMoveCommand = {0.0f, false, false, true};
float pausedRemainingAngle = 0.0f;
float pausedOriginalRequestAngle = 0.0f;
float pausedAtAngle = 0.0f;
portMUX_TYPE motionStateMux = portMUX_INITIALIZER_UNLOCKED;

// 连续相对运动的累计残差：正数表示前面少走了，需要下一次多补；负数表示前面多走了，需要下一次少走。
float motionResidualAngle = 0.0f;

volatile long encoderCount = 0;
volatile int lastCLK = 0;

// 急停标志：true 表示已经拉高 A4988 EN，电机驱动输出被禁止。
// volatile 让 Core0 的 WiFi/Serial task 和 Core1 的电机 task 都能及时看到变化。
volatile bool emergencyStopActive = false;

// 软件重启后保持急停锁定。
// ESTOP / ESTOP_REBOOT 会把它设为 true；ESTOP_CLEAR 才会解除。
// RTC_DATA_ATTR 可以跨 ESP.restart() 保留，但断电后会丢失。
RTC_DATA_ATTR bool estopLatchedAfterRestart = false;

// =====================================================
// LED Functions
// =====================================================
void setColor(int r, int g, int b) {
#if USE_NEOPIXEL_LED == 1
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
#else
  (void)r;
  (void)g;
  (void)b;
#endif
}

// =====================================================
// Emergency Stop / A4988 Enable Functions
// =====================================================
void motorDriverEnable(bool enable) {
#if A4988_ENABLE_ACTIVE_LOW == 1
  digitalWrite(EN_PIN, enable ? LOW : HIGH);
#else
  digitalWrite(EN_PIN, enable ? HIGH : LOW);
#endif
}

void motorDriverDisable() {
  motorDriverEnable(false);
  digitalWrite(STEP_PIN, LOW);
}

void clearPausedMotion() {
  portENTER_CRITICAL(&motionStateMux);
  motionPaused = false;
  pausedRemainingAngle = 0.0f;
  pausedOriginalRequestAngle = 0.0f;
  pausedAtAngle = 0.0f;
  pausedMoveCommand = {0.0f, false, false, true};
  portEXIT_CRITICAL(&motionStateMux);
}

String getMotorStateString() {
  String state;
  if (emergencyStopActive) {
    state = "ESTOP";
  } else if (motorMoving) {
    state = "MOVING";
  } else if (motionPaused) {
    state = "PAUSED";
  } else {
    state = "IDLE";
  }

  return "MOTOR_STATE=" + state +
         " CURRENT=" + String(getRealAngleTotal(), 2) +
         " REMAINING=" + String(pausedRemainingAngle, 2) +
         " DRIVER=" + String(emergencyStopActive ? "DISABLED" : "ENABLED");
}

void requestNormalStop() {
  // 普通停止不关闭 A4988，只要求 PID 循环退出。
  normalStopRequested = true;

  // 清除尚未开始的运动，避免恢复前又执行旧队列。
  if (targetQueue != NULL) {
    xQueueReset(targetQueue);
  }
}

void emergencyStopNow() {
  emergencyStopActive = true;
  estopLatchedAfterRestart = true;

  // 先禁止 A4988 输出，再清 STEP，尽量做到最快停止。
  motorDriverDisable();

  // 清掉还没执行的运动命令，并删除 STOP 保存的恢复任务。
  if (targetQueue != NULL) {
    xQueueReset(targetQueue);
  }
  normalStopRequested = false;
  clearPausedMotion();

  Serial.println("EMERGENCY STOP: A4988 disabled, latched, queue and paused motion cleared");
}

void clearEmergencyStop() {
  emergencyStopActive = false;
  estopLatchedAfterRestart = false;
  motorDriverEnable(true);
  Serial.println("Emergency stop cleared: A4988 enabled");
}

void estopRebootTask(void* pvParameters) {
  (void)pvParameters;

  // 给 TCP/Serial 回包一点时间发出去。
  vTaskDelay(pdMS_TO_TICKS(250));

  Serial.println("ESTOP_REBOOT: restarting now, ESTOP latch will stay ON after reboot");
  Serial.flush();
  ESP.restart();
}

void scheduleEmergencyReboot() {
  xTaskCreatePinnedToCore(
    estopRebootTask,
    "ESTOP Reboot",
    2048,
    NULL,
    3,
    NULL,
    0
  );
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

float getEncoderAngleTotal() {
  long countCopy = getEncoderCount();
  return countCopy * encoderDegPerCount;
}

float getRealAngleTotal() {
  // 对外/闭环使用“实际输出轴角度”。
  // EC11本身仍然是 1 count = 4.5度，只是在这里乘以机械换算比例。
  return getEncoderAngleTotal() * OUTPUT_ANGLE_PER_ENCODER_ANGLE;
}

float getOutputDegPerEncoderCount() {
  return encoderDegPerCount * OUTPUT_ANGLE_PER_ENCODER_ANGLE;
}

long angleToEncoderCount(float angle) {
  float degPerCount = getOutputDegPerEncoderCount();

  if (fabs(degPerCount) < 0.000001f) {
    return 0;
  }

  return lround(angle / degPerCount);
}

void setCurrentAngleTotal(float newAngle) {
  setEncoderCount(angleToEncoderCount(newAngle));
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

String getCurrentAngleString() {
  return "CURRENT=" + String(getRealAngleTotal(), 2) +
         " ENCODER=" + String(getEncoderCount()) +
         " HOME_POS=" + String(savedHomeOffsetAngle, 2) +
         " LAST_DIR=" + getDirectionName(lastMoveDir);
}

String getEmergencyStopString() {
  return "ESTOP=" + String(emergencyStopActive ? "ON" : "OFF") +
         " LATCH=" + String(estopLatchedAfterRestart ? "ON" : "OFF") +
         " EN_PIN=" + String(EN_PIN) +
         " DRIVER=" + String(emergencyStopActive ? "DISABLED" : "ENABLED") +
         " CURRENT=" + String(getRealAngleTotal(), 2);
}

bool isAngleLimitEnabled() {
  return maxAbsAngleLimit > ANGLE_LIMIT_EPS;
}

bool isTargetWithinAbsAngleLimit(float targetAngle) {
  if (!isAngleLimitEnabled()) {
    return true;
  }

  return fabs(targetAngle) <= (maxAbsAngleLimit + ANGLE_LIMIT_EPS);
}

String getAngleLimitString() {
  return "ANGLE_LIMIT=" + String(maxAbsAngleLimit, 2) +
         " MODE=" + String(isAngleLimitEnabled() ? "ON" : "OFF") +
         " CURRENT=" + String(getRealAngleTotal(), 2);
}

String getAngleLimitRejectString(float currentAngle, float requestedAngle) {
  float targetAngle = currentAngle + requestedAngle;

  return "ERR ANGLE_LIMIT CURRENT=" + String(currentAngle, 2) +
         " REQUEST=" + String(requestedAngle, 2) +
         " TARGET=" + String(targetAngle, 2) +
         " LIMIT=" + String(maxAbsAngleLimit, 2);
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

void sendTcpLineToClient(int slot, const String& line) {
  if (slot < 0 || slot >= MAX_TCP_CLIENTS || tcpClientsMutex == NULL) {
    return;
  }

  if (xSemaphoreTake(tcpClientsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (tcpClients[slot] && tcpClients[slot].connected()) {
      tcpClients[slot].println(line);
    }
    xSemaphoreGive(tcpClientsMutex);
  }
}

void broadcastTcpLine(const String& line) {
  if (tcpClientsMutex == NULL) {
    return;
  }

  if (xSemaphoreTake(tcpClientsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
      if (tcpClients[i] && tcpClients[i].connected()) {
        tcpClients[i].println(line);
      }
    }
    xSemaphoreGive(tcpClientsMutex);
  }
}

int findFreeTcpClientSlot() {
  for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
    if (!tcpClients[i] || !tcpClients[i].connected()) {
      return i;
    }
  }
  return -1;
}

// =====================================================
// Flash Config Functions
// =====================================================
void updateDerivedParams() {
  // 固定使用你实测的EC11精度：1 count = 4.5度。
  // 不再让Flash旧参数或上位机误设置把CPR改回40。
  encoderCountsPerRev = DEFAULT_ENCODER_COUNTS_PER_REV;
  encoderDegPerCount = FIXED_ENCODER_DEG_PER_COUNT;
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
         " ANGLE_LIMIT=" + String(maxAbsAngleLimit, 2) +
         " ESTOP=" + String(emergencyStopActive ? "ON" : "OFF") +
         " EN_PIN=" + String(EN_PIN) +
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
  // 固定EC11精度，不读取Flash里的旧CPR，避免旧的40覆盖当前80。
  encoderCountsPerRev = DEFAULT_ENCODER_COUNTS_PER_REV;
  angleTolerance = prefs.getFloat("tol", DEFAULT_ANGLE_TOLERANCE);
  backlashCompAngle = prefs.getFloat(FLASH_KEY_BACKLASH, DEFAULT_BACKLASH_COMP_ANGLE);
  maxAbsAngleLimit = prefs.getFloat(FLASH_KEY_MAX_ABS_ANGLE, DEFAULT_MAX_ABS_ANGLE_LIMIT);

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

  if (maxAbsAngleLimit < 0.0f || maxAbsAngleLimit > 64800.0f) {
    maxAbsAngleLimit = DEFAULT_MAX_ABS_ANGLE_LIMIT;
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
  prefs.putFloat(FLASH_KEY_MAX_ABS_ANGLE, maxAbsAngleLimit);
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
  maxAbsAngleLimit = DEFAULT_MAX_ABS_ANGLE_LIMIT;
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
         " ANGLE_LIMIT=" + String(maxAbsAngleLimit, 2) +
         " ESTOP=" + String(emergencyStopActive ? "ON" : "OFF") +
         " EN_PIN=" + String(EN_PIN) +
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
  if (emergencyStopActive) {
    digitalWrite(STEP_PIN, LOW);
    return;
  }

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

long openLoopRotateAngle(float angle, float rotateSpeedHz) {
  if (emergencyStopActive) {
    return 0;
  }

  int dirSign = getMoveDirection(angle);

  if (dirSign == 0) {
    return 0;
  }

  float localSpeed = rotateSpeedHz;

  if (localSpeed < 1.0f) {
    localSpeed = 1.0f;
  }

  if (localSpeed > 3000.0f) {
    localSpeed = 3000.0f;
  }

  uint32_t halfPeriodUs = (uint32_t)(1000000.0f / localSpeed / 2.0f);

  if (halfPeriodUs < 50) {
    halfPeriodUs = 50;
  }

  long steps = lround(fabs(angle) / STEP_ANGLE);

  if (steps < 1) {
    steps = 1;
  }

  bool dir = angle > 0.0f;
  unsigned long lastYieldMs = millis();

  for (long i = 0; i < steps; i++) {
    if (emergencyStopActive) {
      break;
    }

    oneStepWithDelay(dir, halfPeriodUs);

    unsigned long nowMs = millis();
    if (nowMs - lastYieldMs >= MOTOR_TASK_YIELD_INTERVAL_MS) {
      lastYieldMs = nowMs;
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }

  return steps;
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
  // Emergency Stop / A4988 Enable
  // =====================================================
  if (cmd == "ESTOP" || cmd == "EMERGENCY_STOP") {
    emergencyStopNow();
    return "OK ESTOP " + getEmergencyStopString() + " " + getMotorStateString();
  }

  // 普通停止：不禁止 A4988，保存当前运动剩余角度，之后可 RESUME。
  if (cmd == "STOP" || cmd == "MOTOR_STOP" || cmd == "PAUSE") {
    if (emergencyStopActive) {
      return "ERR ESTOP_ACTIVE " + getEmergencyStopString();
    }
    if (!motorMoving) {
      return "ERR MOTOR_NOT_MOVING " + getMotorStateString();
    }
    requestNormalStop();
    return "OK STOP_REQUESTED " + getMotorStateString();
  }

  if (cmd == "RESUME" || cmd == "MOTOR_RESUME") {
    if (emergencyStopActive) {
      return "ERR ESTOP_ACTIVE USE ESTOP_CLEAR " + getEmergencyStopString();
    }
    if (motorMoving) {
      return "ERR MOTOR_BUSY " + getMotorStateString();
    }

    MotorMoveCommand resumeCmd;
    bool hasPausedMotion = false;

    portENTER_CRITICAL(&motionStateMux);
    if (motionPaused && fabs(pausedRemainingAngle) > DIRECTION_DEADBAND_ANGLE) {
      resumeCmd = pausedMoveCommand;
      resumeCmd.angle = pausedRemainingAngle;
      resumeCmd.resumeMove = true;
      motionPaused = false;
      hasPausedMotion = true;
    }
    portEXIT_CRITICAL(&motionStateMux);

    if (!hasPausedMotion) {
      return "ERR NO_PAUSED_MOTION " + getMotorStateString();
    }

    if (xQueueSend(targetQueue, &resumeCmd, 0) != pdTRUE) {
      portENTER_CRITICAL(&motionStateMux);
      motionPaused = true;
      portEXIT_CRITICAL(&motionStateMux);
      return "ERR MOTOR_QUEUE_FULL";
    }

    return "OK RESUME REMAINING=" + String(resumeCmd.angle, 2);
  }

  if (cmd == "MOTOR_STATE?" || cmd == "STATE?") {
    return getMotorStateString();
  }

  if (cmd == "ESTOP_REBOOT" || cmd == "STOP_REBOOT" || cmd == "EMERGENCY_REBOOT") {
    emergencyStopNow();
    scheduleEmergencyReboot();
    return "OK ESTOP_REBOOT " + getEmergencyStopString() + " REBOOTING=YES";
  }

  if (cmd == "ESTOP_CLEAR" || cmd == "STOP_CLEAR" || cmd == "MOTOR_ENABLE") {
    clearEmergencyStop();
    return "OK ESTOP_CLEAR " + getEmergencyStopString();
  }

  if (cmd == "MOTOR_DISABLE") {
    emergencyStopNow();
    return "OK MOTOR_DISABLE " + getEmergencyStopString();
  }

  if (cmd == "ESTOP?" || cmd == "STOP?" || cmd == "MOTOR_ENABLE?") {
    return getEmergencyStopString();
  }

  // =====================================================
  // LED
  // =====================================================
  if (cmd == "LED_ON") {
#if USE_NEOPIXEL_LED == 1
    ledEnabled = true;
    setColor(255, 255, 255);
    return "OK LED_ON";
#else
    ledEnabled = false;
    return "ERR LED DISABLED GPIO38_USED_BY_A4988_EN";
#endif
  }

  if (cmd == "LED_OFF") {
    ledEnabled = false;
#if USE_NEOPIXEL_LED == 1
    pixels.clear();
    pixels.show();
#endif
    return "OK LED_OFF";
  }

  if (cmd.startsWith("RGB:")) {
#if USE_NEOPIXEL_LED != 1
    return "ERR LED DISABLED GPIO38_USED_BY_A4988_EN";
#endif

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
  // 编码器比例设置：保留命令兼容，但本版本固定为 CPR=80，1 count=4.5度
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
  // 绝对角度限位查询
  // =====================================================
  if (cmd == "LIMIT?" || cmd == "ANGLE_LIMIT?") {
    return getAngleLimitString();
  }

  // =====================================================
  // 绝对角度限位设置
  // 命令：LIMIT:60 / ANGLE_LIMIT:60
  // 单位：度；0 表示关闭限位
  // 只改 RAM，不立刻写 Flash；需要长期保存就再发 CFG_SAVE
  // =====================================================
  if (cmd.startsWith("LIMIT:") || cmd.startsWith("ANGLE_LIMIT:")) {
    String data;

    if (cmd.startsWith("LIMIT:")) {
      data = cmd.substring(6);
    } else {
      data = cmd.substring(12);
    }

    data.trim();

    if (isValidNumber(data)) {
      float newLimit = data.toFloat();

      if (newLimit < 0.0f || newLimit > 64800.0f) {
        return "ERR ANGLE_LIMIT RANGE";
      }

      maxAbsAngleLimit = newLimit;

      return "OK " + getAngleLimitString();
    }

    return "ERR ANGLE_LIMIT FORMAT";
  }

  // =====================================================
  // 当前角度查询
  // =====================================================
  if (cmd == "ANGLE?" || cmd == "CURRENT?") {
    return "ANGLE " + getCurrentAngleString();
  }

  // =====================================================
  // 手动修改 Current Angle 显示值：只改编码器计数，不转电机，也不改 HOME_POS。
  // 命令：CURRENT:90 / ANGLE_SET:90
  // 如果要把 Current Angle 强制同步到 HOME_POS，发 CURRENT_SYNC_HOME。
  // =====================================================
  if (cmd.startsWith("CURRENT:") || cmd.startsWith("ANGLE_SET:")) {
    String data;

    if (cmd.startsWith("CURRENT:")) {
      data = cmd.substring(8);
    } else {
      data = cmd.substring(10);
    }

    data.trim();

    if (isValidNumber(data)) {
      float newCurrent = data.toFloat();

      if (newCurrent < -64800.0f || newCurrent > 64800.0f) {
        return "ERR CURRENT RANGE";
      }

      setCurrentAngleTotal(newCurrent);

      return "OK CURRENT_SET " + getCurrentAngleString();
    }

    return "ERR CURRENT FORMAT";
  }

  if (cmd == "CURRENT_SYNC_HOME" || cmd == "ANGLE_SYNC_HOME") {
    setCurrentAngleTotal(savedHomeOffsetAngle);
    return "OK CURRENT_SYNC_HOME " + getCurrentAngleString();
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
  // 急停锁定时，不接收任何运动角度命令。
  // ANGLE? / CURRENT? 查询仍然允许，方便上位机显示当前位置。
  // =====================================================
  if (emergencyStopActive &&
      (cmd.startsWith("ANGLE:") || isValidNumber(cmd))) {
    return "ERR ESTOP_ACTIVE ANGLE_REJECTED USE ESTOP_CLEAR " + getEmergencyStopString();
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

    if (motorMoving) {
      return "ERR MOTOR_BUSY USE STOP FIRST " + getMotorStateString();
    }

    // 新角度命令优先于旧的暂停任务：明确发送新目标即取消旧 RESUME 内容。
    if (motionPaused) {
      clearPausedMotion();
    }

    if (targetAngle < -64800 || targetAngle > 64800) {
      return "ERR RANGE";
    }

    float currentAngle = getRealAngleTotal();
    float logicalTargetAngle = currentAngle + targetAngle;

    if (!isTargetWithinAbsAngleLimit(logicalTargetAngle)) {
      return getAngleLimitRejectString(currentAngle, targetAngle);
    }

    MotorMoveCommand moveCmd;
    moveCmd.angle = targetAngle;
    moveCmd.zeroAfterMove = false;
#if ENABLE_AUTO_HOME_FLASH == 1
    moveCmd.savePosition = true;
    moveCmd.resumeMove = false;
#else
    moveCmd.savePosition = false;
    moveCmd.resumeMove = false;
#endif

    if (xQueueSend(targetQueue, &moveCmd, 0) != pdTRUE) {
      return "ERR MOTOR_QUEUE_FULL";
    }

    return "OK ANGLE " + String(targetAngle, 2);
  }

  return "UNKNOWN CMD";
}

// =====================================================
// WiFi TCP Task
// =====================================================
void wifiServerTask(void* pvParameters) {
  while (true) {
    // 接收新客户端，但不阻塞在任何一个客户端上。
    WiFiClient newClient = server.available();
    if (newClient) {
      int slot = -1;

      if (xSemaphoreTake(tcpClientsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        slot = findFreeTcpClientSlot();
        if (slot >= 0) {
          tcpClients[slot] = newClient;
          tcpRxBuffers[slot] = "";
        }
        xSemaphoreGive(tcpClientsMutex);
      }

      if (slot >= 0) {
        Serial.print("TCP client connected, slot=");
        Serial.println(slot);
        sendTcpLineToClient(slot, "ESP32 READY SLOT=" + String(slot) + " " + getMotorStateString());
      } else {
        newClient.println("ERR SERVER_FULL");
        newClient.stop();
      }
    }

    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
      if (!tcpClients[i]) {
        continue;
      }

      if (!tcpClients[i].connected()) {
        if (xSemaphoreTake(tcpClientsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          tcpClients[i].stop();
          tcpClients[i] = WiFiClient();
          tcpRxBuffers[i] = "";
          xSemaphoreGive(tcpClientsMutex);
        }
        Serial.print("TCP client disconnected, slot=");
        Serial.println(i);
        continue;
      }

      // 每个客户端独立按换行符组包，避免 readStringUntil 阻塞其他连接。
      while (tcpClients[i].available()) {
        char c = (char)tcpClients[i].read();

        if (c == '\r') {
          continue;
        }

        if (c == '\n') {
          String cmd = tcpRxBuffers[i];
          tcpRxBuffers[i] = "";
          cmd.trim();

          if (cmd.length() == 0) {
            continue;
          }

          Serial.print("TCP[");
          Serial.print(i);
          Serial.print("] Recv: ");
          Serial.println(cmd);

          String reply = handleCommand(cmd);
          sendTcpLineToClient(i, reply);

          Serial.print("TCP[");
          Serial.print(i);
          Serial.print("] Reply: ");
          Serial.println(reply);
        } else {
          if (tcpRxBuffers[i].length() < TCP_RX_BUFFER_LIMIT) {
            tcpRxBuffers[i] += c;
          } else {
            tcpRxBuffers[i] = "";
            sendTcpLineToClient(i, "ERR COMMAND_TOO_LONG");
          }
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5));
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
      broadcastTcpLine(String(msg.text));
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

      normalStopRequested = false;
      motorMoving = true;

      if (emergencyStopActive) {
        motorMoving = false;
        String estopMsg =
          String("MOTOR_DONE STATUS=ESTOP") +
          " REQUEST=" + String(moveCmd.angle, 2) +
          " FINAL=" + String(getRealAngleTotal(), 2) +
          " HOME_POS=" + String(savedHomeOffsetAngle, 2) +
          " ESTOP=ON";
        Serial.println(estopMsg);
        enqueueUpperNotify(estopMsg);
        continue;
      }

      // 正常运动开始前确保 A4988 处于使能状态。
      motorDriverEnable(true);

      // requestedAngle 是用户/自动回零请求的逻辑角度。
      // moveAngle 是实际送给 PID 执行的角度，可能已经加了反向间隙补偿。
      float requestedAngle = moveCmd.angle;
      int currentMoveDir = getMoveDirection(requestedAngle);

      // 执行前再复查一次限位，防止命令排队期间 Current Angle 已经变化。
      // 自动回零 zeroAfterMove 不受这个限位阻挡，避免 Flash 回零被卡住。
      float limitCheckStartAngle = getRealAngleTotal();
      float logicalTargetAngle = limitCheckStartAngle + requestedAngle;

      if (!moveCmd.zeroAfterMove && !isTargetWithinAbsAngleLimit(logicalTargetAngle)) {
        String limitMsg =
          String("MOTOR_DONE STATUS=ANGLE_LIMIT") +
          " REQUEST=" + String(requestedAngle, 2) +
          " CURRENT=" + String(limitCheckStartAngle, 2) +
          " TARGET=" + String(logicalTargetAngle, 2) +
          " LIMIT=" + String(maxAbsAngleLimit, 2) +
          " HOME_POS=" + String(savedHomeOffsetAngle, 2) +
          " LAST_DIR=" + getDirectionName(lastMoveDir);

        Serial.println(limitMsg);
        enqueueUpperNotify(limitMsg);
        continue;
      }

      float residualBeforeMove = motionResidualAngle;
      float residualComp = 0.0f;

      if (!moveCmd.zeroAfterMove && !moveCmd.resumeMove) {
        residualComp = getResidualCompensationForMove(requestedAngle);
      }

      float backlashComp = 0.0f;
      bool backlashApplied = false;

      if (!moveCmd.zeroAfterMove && !moveCmd.resumeMove && currentMoveDir != 0 &&
          lastMoveDir != 0 && currentMoveDir != lastMoveDir) {
        backlashComp = backlashCompAngle * currentMoveDir;
        backlashApplied = true;
      }

      float compensatedRequestAngle = requestedAngle + residualComp;
      float moveAngle = compensatedRequestAngle + backlashComp;
      float startAngle = getRealAngleTotal();
      float targetAngle = startAngle + moveAngle;

      // 如果残差/反向间隙补偿会把实际 PID 目标推出绝对限位，则取消补偿，
      // 保证最终目标仍在 LIMIT 内。限位判断的核心仍然是 Current + 用户输入。
      if (!moveCmd.zeroAfterMove && !isTargetWithinAbsAngleLimit(targetAngle)) {
        residualComp = 0.0f;
        backlashComp = 0.0f;
        backlashApplied = false;
        compensatedRequestAngle = requestedAngle;
        moveAngle = requestedAngle;
        targetAngle = startAngle + moveAngle;
      }

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

      long postCorrectSteps = 0;
      int postCorrectRounds = 0;
      float postCorrectErrorBefore = 0.0f;
      float postCorrectErrorAfter = 0.0f;

      while (true) {
        if (emergencyStopActive) {
          motorDriverDisable();
          arrived = false;
          stopReason = "ESTOP";
          finalAngle = getRealAngleTotal();
          Serial.print("Emergency stop during move | Final: ");
          Serial.print(finalAngle, 2);
          Serial.println(" deg");
          break;
        }

        if (normalStopRequested) {
          // 普通停止只停止 STEP 脉冲，A4988 保持使能和保持力矩。
          digitalWrite(STEP_PIN, LOW);
          arrived = false;
          stopReason = "PAUSED";
          finalAngle = getRealAngleTotal();
          Serial.print("Normal stop during move | Final: ");
          Serial.print(finalAngle, 2);
          Serial.println(" deg | Driver remains enabled");
          break;
        }

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
      // 运动结束后开环微调：比较目标角度和当前角度，有偏差就纯步进补转一次
      // =====================================================
      postCorrectErrorBefore = targetAngle - getRealAngleTotal();
      postCorrectErrorAfter = postCorrectErrorBefore;

#if POST_CORRECT_ENABLE == 1
      bool canPostCorrect =
        !emergencyStopActive &&
        stopReason != "PAUSED" &&
        fabs(postCorrectErrorBefore) > POST_CORRECT_TOLERANCE &&
        fabs(postCorrectErrorBefore) <= POST_CORRECT_MAX_ANGLE;

      if (canPostCorrect) {
        for (int round = 0; round < POST_CORRECT_MAX_ROUNDS; round++) {
          if (emergencyStopActive) {
            break;
          }

          float correctionError = targetAngle - getRealAngleTotal();

          if (fabs(correctionError) <= POST_CORRECT_TOLERANCE) {
            break;
          }

          float correctionAngle = correctionError;

          if (correctionAngle > POST_CORRECT_MAX_ANGLE) {
            correctionAngle = POST_CORRECT_MAX_ANGLE;
          }

          if (correctionAngle < -POST_CORRECT_MAX_ANGLE) {
            correctionAngle = -POST_CORRECT_MAX_ANGLE;
          }

          Serial.print("Post-correct open loop | Target: ");
          Serial.print(targetAngle, 2);
          Serial.print(" deg | Current: ");
          Serial.print(getRealAngleTotal(), 2);
          Serial.print(" deg | Error: ");
          Serial.print(correctionError, 2);
          Serial.print(" deg | Rotate: ");
          Serial.print(correctionAngle, 2);
          Serial.println(" deg");

          long steps = openLoopRotateAngle(correctionAngle, POST_CORRECT_SPEED_HZ);
          postCorrectSteps += steps;
          postCorrectRounds++;

          vTaskDelay(pdMS_TO_TICKS(POST_CORRECT_SETTLE_MS));
        }

        finalAngle = getRealAngleTotal();
        postCorrectErrorAfter = targetAngle - finalAngle;

        if (fabs(postCorrectErrorAfter) <= angleTolerance ||
            fabs(postCorrectErrorAfter) <= POST_CORRECT_TOLERANCE) {
          arrived = true;
          stopReason = "POST_CORRECT_OK";
        } else {
          stopReason = "POST_CORRECT_LEFT_ERROR";
        }

        Serial.print("Post-correct done | Final: ");
        Serial.print(finalAngle, 2);
        Serial.print(" deg | ErrorAfter: ");
        Serial.print(postCorrectErrorAfter, 2);
        Serial.print(" deg | Rounds: ");
        Serial.print(postCorrectRounds);
        Serial.print(" | Steps: ");
        Serial.println(postCorrectSteps);
      } else {
        finalAngle = getRealAngleTotal();
        postCorrectErrorAfter = targetAngle - finalAngle;
      }
#else
      finalAngle = getRealAngleTotal();
      postCorrectErrorAfter = targetAngle - finalAngle;
#endif

      // =====================================================
      // 一次运动结束后再写 Flash，避免每一步写 Flash 导致寿命下降
      // =====================================================
      float actualDelta = finalAngle - startAngle;
      float logicalActualDelta = actualDelta - backlashComp;

      if (stopReason == "PAUSED") {
        float remainingAngle = targetAngle - finalAngle;

        portENTER_CRITICAL(&motionStateMux);
        pausedMoveCommand = moveCmd;
        pausedMoveCommand.angle = remainingAngle;
        pausedMoveCommand.resumeMove = true;
        pausedRemainingAngle = remainingAngle;
        pausedOriginalRequestAngle = requestedAngle;
        pausedAtAngle = finalAngle;
        motionPaused = fabs(remainingAngle) > DIRECTION_DEADBAND_ANGLE;
        portEXIT_CRITICAL(&motionStateMux);

        normalStopRequested = false;
      }

      if (!moveCmd.zeroAfterMove && currentMoveDir != 0) {
        if (arrived) {
          motionResidualAngle = residualBeforeMove + requestedAngle - logicalActualDelta;
          limitMotionResidual();
        } else if (stopReason != "PAUSED") {
          // 真正失败时不要保留历史残差；普通 STOP 的剩余运动由 RESUME 单独保存。
          motionResidualAngle = 0.0f;
        }
      }

      if (moveCmd.savePosition && stopReason != "ESTOP") {

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
        if (stopReason == "ESTOP") {
          Serial.println("Emergency stop: Flash HOME_POS not updated. Re-sync with ZERO or HOME:<angle> if needed.");
        }

        // 关闭 Flash 找零时，也要继续记录上一次方向，保证反向间隙补偿仍然生效。
        if (!moveCmd.zeroAfterMove && currentMoveDir != 0 && arrived) {
          lastMoveDir = currentMoveDir;
          Serial.print("Position save disabled, LastDir updated: ");
          Serial.println(getDirectionName(lastMoveDir));
        }
      }

      if (arrived && moveCmd.resumeMove) {
        clearPausedMotion();
      }

      motorMoving = false;

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
        " POST_ROUNDS=" + String(postCorrectRounds) +
        " POST_STEPS=" + String(postCorrectSteps) +
        " POST_ERR_BEFORE=" + String(postCorrectErrorBefore, 2) +
        " POST_ERR_AFTER=" + String(postCorrectErrorAfter, 2) +
        " RESIDUAL=" + String(motionResidualAngle, 2) +
        " HOME_POS=" + String(savedHomeOffsetAngle, 2) +
        " ANGLE_LIMIT=" + String(maxAbsAngleLimit, 2) +
        " ESTOP=" + String(emergencyStopActive ? "ON" : "OFF") +
        " DRIVER=" + String(emergencyStopActive ? "DISABLED" : "ENABLED") +
        " PAUSED=" + String(motionPaused ? "YES" : "NO") +
        " REMAINING=" + String(pausedRemainingAngle, 2) +
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
    float encoderAngleTotal = encCopy * encoderDegPerCount;
    float realAngleTotal = encoderAngleTotal * OUTPUT_ANGLE_PER_ENCODER_ANGLE;
    float realAngle360 = normalizeAngle(realAngleTotal);
    bool swState = digitalRead(ENCODE_SW);

    Serial.print("Encoder: ");
    Serial.print(encCopy);

    Serial.print(" | Angle360: ");
    Serial.print(realAngle360);

    Serial.print(" | EncoderAngle: ");
    Serial.print(encoderAngleTotal);

    Serial.print(" | Total: ");
    Serial.print(realAngleTotal);

    Serial.print(" | HomePos: ");
    Serial.print(savedHomeOffsetAngle);

    Serial.print(" | LastDir: ");
    Serial.print(getDirectionName(lastMoveDir));

    Serial.print(" | Residual: ");
    Serial.print(motionResidualAngle);

    Serial.print(" | MotorState: ");
    if (emergencyStopActive) Serial.print("ESTOP");
    else if (motorMoving) Serial.print("MOVING");
    else if (motionPaused) Serial.print("PAUSED");
    else Serial.print("IDLE");

    Serial.print(" | Remaining: ");
    Serial.print(pausedRemainingAngle);

    Serial.print(" | ESTOP: ");
    Serial.print(emergencyStopActive ? "ON" : "OFF");

    Serial.print(" | EN_PIN: ");
    Serial.print(EN_PIN);

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
#if USE_NEOPIXEL_LED == 1
  pixels.begin();
  pixels.setBrightness(80);
  pixels.clear();
  pixels.show();
#else
  Serial.println("NeoPixel LED disabled because GPIO38 is used as A4988 EN");
#endif

  // Motor
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  // 默认先禁止驱动输出，等初始化完成后再使能，避免开机瞬间乱动。
  motorDriverDisable();

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  pinMode(ENCODE_CLK, INPUT_PULLUP);
  pinMode(ENCODE_DT, INPUT_PULLUP);
  pinMode(ENCODE_SW, INPUT_PULLUP);

  lastCLK = digitalRead(ENCODE_CLK);
  attachInterrupt(digitalPinToInterrupt(ENCODE_CLK), encoderISR, CHANGE);

  targetQueue = xQueueCreate(10, sizeof(MotorMoveCommand));
  notifyQueue = xQueueCreate(NOTIFY_QUEUE_LENGTH, sizeof(UpperNotifyMessage));
  tcpClientsMutex = xSemaphoreCreateMutex();

  if (targetQueue == NULL || notifyQueue == NULL || tcpClientsMutex == NULL) {
    Serial.println("ERR: queue/mutex create failed");
    while (true) {
      delay(1000);
    }
  }

  if (estopLatchedAfterRestart) {
    emergencyStopActive = true;
    motorDriverDisable();
    Serial.println("Boot in ESTOP latched state: A4988 stays disabled. Use ESTOP_CLEAR to unlock.");
  } else {
    emergencyStopActive = false;
    motorDriverEnable(true);
    Serial.println("A4988 enabled on GPIO38. ESTOP command will pull EN high.");
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
  // 开机自动回零：如果 Flash 里保存了当前位置，就直接往 HOME_POS 的反方向回零。
  // 但 ESTOP_REBOOT 后会保持急停锁定，此时绝对不能自动回零。
  if (emergencyStopActive) {
    Serial.println("Auto home skipped because ESTOP is active. Use ESTOP_CLEAR to unlock first.");
  } else {
    float autoHomeAngle = getAutoHomeMoveAngle();

    if (fabs(autoHomeAngle) > AUTO_HOME_MIN_ANGLE) {
      MotorMoveCommand homeCmd;
      homeCmd.angle = autoHomeAngle;
      homeCmd.zeroAfterMove = true;
      homeCmd.savePosition = true;
      homeCmd.resumeMove = false;

      xQueueSend(targetQueue, &homeCmd, portMAX_DELAY);

      Serial.print("Auto home queued. Saved position: " );
      Serial.print(savedHomeOffsetAngle, 2);
      Serial.print(" deg | Move back: " );
      Serial.print(homeCmd.angle, 2);
      Serial.println(" deg");
    } else {
      Serial.println("Auto home skipped: saved position is 0");
    }
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
  Serial.println("EC11 fixed: 1 count = 4.5 deg, output angle = EC11 angle * 2");
  Serial.println("v6: ESTOP_REBOOT + ESTOP angle lock added, GPIO38 A4988 EN, ANGLE_LIMIT retained");
  Serial.println("Config commands: CFG? / CFG_SAVE / CFG_RESET / PID? / SPEED? / MINSPEED? / ENCODER? / TOL? / BACKLASH? / LIMIT?");
  Serial.println("Angle commands: ANGLE? / CURRENT? / CURRENT:90 / ANGLE_SET:90 / CURRENT_SYNC_HOME");
  Serial.println("Home commands: HOME? / POS? / HOME_ZERO / ZERO / HOME:60");
  Serial.println("Set ENABLE_AUTO_HOME_FLASH to 1 to enable Flash home, 0 to disable it");
  Serial.println("Backlash commands: BACKLASH? / BACKLASH:5, use CFG_SAVE to persist");
  Serial.println("Angle limit commands: LIMIT? / LIMIT:60 / ANGLE_LIMIT:60, 0 means OFF, use CFG_SAVE to persist");
  Serial.println("Residual commands: RESIDUAL? / RESIDUAL_ZERO");
  Serial.println("Motion state commands: STOP / MOTOR_STOP / RESUME / MOTOR_RESUME / MOTOR_STATE?");
  Serial.println("Emergency stop commands: ESTOP / ESTOP_REBOOT / ESTOP_CLEAR / ESTOP?");
}

// =====================================================
// Loop
// =====================================================
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}