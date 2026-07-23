#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_NeoPixel.h>

#define WS2812_PIN   38
#define WS2812_COUNT 1

// ESP32 热点配置
const char* AP_SSID = "ESP32_Motor";
const char* AP_PASSWORD = "12345678";

// UDP 配置
constexpr uint16_t UDP_PORT = 8080;

Adafruit_NeoPixel ws2812(
  WS2812_COUNT,
  WS2812_PIN,
  NEO_GRB + NEO_KHZ800
);

WiFiUDP udp;

char serialBuffer[32];
size_t serialBufferIndex = 0;

char udpBuffer[64];

/**
 * 解析并设置 RGB 颜色。
 *
 * 支持格式：
 * 255,0,0
 * 0,255,0
 * 0,0,255
 */
bool setColor(const char* command)
{
  int red = 0;
  int green = 0;
  int blue = 0;

  if (sscanf(command, "%d,%d,%d", &red, &green, &blue) != 3) {
    Serial.printf("ERR invalid command: %s\n", command);
    return false;
  }

  red = constrain(red, 0, 255);
  green = constrain(green, 0, 255);
  blue = constrain(blue, 0, 255);

  ws2812.setPixelColor(
    0,
    ws2812.Color(red, green, blue)
  );

  ws2812.show();

  Serial.printf("OK RGB=%d,%d,%d\n", red, green, blue);
  return true;
}

/**
 * 处理串口输入。
 */
void handleSerial()
{
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());

    if (c == '\n') {
      serialBuffer[serialBufferIndex] = '\0';

      if (serialBufferIndex > 0) {
        setColor(serialBuffer);
      }

      serialBufferIndex = 0;
    }
    else if (c != '\r') {
      if (serialBufferIndex < sizeof(serialBuffer) - 1) {
        serialBuffer[serialBufferIndex++] = c;
      }
      else {
        serialBufferIndex = 0;
        Serial.println("ERR serial command too long");
      }
    }
  }
}

/**
 * 处理 UDP 数据。
 */
void handleUdp()
{
  int packetSize = udp.parsePacket();

  if (packetSize <= 0) {
    return;
  }

  int readLength = udp.read(
    udpBuffer,
    sizeof(udpBuffer) - 1
  );

  if (readLength <= 0) {
    return;
  }

  udpBuffer[readLength] = '\0';

  // 删除换行符和回车符
  for (int i = 0; i < readLength; i++) {
    if (udpBuffer[i] == '\r' || udpBuffer[i] == '\n') {
      udpBuffer[i] = '\0';
      break;
    }
  }

  Serial.printf(
    "UDP from %s:%u -> %s\n",
    udp.remoteIP().toString().c_str(),
    udp.remotePort(),
    udpBuffer
  );

  bool success = setColor(udpBuffer);

  // 给发送方返回处理结果
  udp.beginPacket(udp.remoteIP(), udp.remotePort());

  if (success) {
    udp.print("OK");
  }
  else {
    udp.print("ERR");
  }

  udp.endPacket();
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  // 初始化 WS2812
  ws2812.begin();
  ws2812.setBrightness(40);
  ws2812.clear();
  ws2812.show();

  // 创建 Wi-Fi 热点
  WiFi.mode(WIFI_AP);

  bool apStarted = WiFi.softAP(
    AP_SSID,
    AP_PASSWORD
  );

  if (!apStarted) {
    Serial.println("ERR failed to start Wi-Fi AP");
    return;
  }

  IPAddress apIP = WiFi.softAPIP();

  Serial.println();
  Serial.println("================================");
  Serial.println("ESP32 LED controller ready");
  Serial.printf("Wi-Fi SSID: %s\n", AP_SSID);
  Serial.printf("Wi-Fi password: %s\n", AP_PASSWORD);
  Serial.printf("ESP32 IP: %s\n", apIP.toString().c_str());
  Serial.printf("UDP port: %u\n", UDP_PORT);
  Serial.println("Command format: 255,0,0");
  Serial.println("================================");

  // 启动 UDP
  if (udp.begin(UDP_PORT)) {
    Serial.println("UDP server started");
  }
  else {
    Serial.println("ERR failed to start UDP server");
  }

  // 启动时显示蓝色，表示程序已经运行
  setColor("0,0,30");
}

void loop()
{
  handleSerial();
  handleUdp();

  delay(1);
}