#define ENCODE_CLK 5
#define ENCODE_DT  6
#define ENCODE_SW  7

// 先校准：转完整一圈，看 count 变化多少
// 常见 EC11 四倍频可能是 80、72、96 等
float countsPerRev = 80.0;

long encoderCount = 0;
byte lastState = 0;

void readEncoder() {
  byte clk = digitalRead(ENCODE_CLK);
  byte dt  = digitalRead(ENCODE_DT);

  byte currentState = (clk << 1) | dt;
  byte transition = (lastState << 2) | currentState;

  switch (transition) {
    // 正转
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      encoderCount++;
      break;

    // 反转
    case 0b0010:
    case 0b1011:
    case 0b1101:
    case 0b0100:
      encoderCount--;
      break;
  }

  lastState = currentState;
}

void setup() {
  Serial.begin(115200);

  pinMode(ENCODE_CLK, INPUT_PULLUP);
  pinMode(ENCODE_DT, INPUT_PULLUP);
  pinMode(ENCODE_SW, INPUT_PULLUP);

  lastState = (digitalRead(ENCODE_CLK) << 1) | digitalRead(ENCODE_DT);

  Serial.println("EC11 angle test start");
  Serial.println("Press SW to reset angle to 0");
}

void loop() {
  readEncoder();

  // 按下 SW 清零角度
  if (digitalRead(ENCODE_SW) == LOW) {
    encoderCount = 0;
    delay(300); // 简单消抖
  }

  static unsigned long lastPrint = 0;

  if (millis() - lastPrint > 200) {
    lastPrint = millis();

    float angle = encoderCount * 360.0 / countsPerRev;

    Serial.print("count = ");
    Serial.print(encoderCount);

    Serial.print(" , angle = ");
    Serial.print(angle);
    Serial.println(" deg");
  }
}