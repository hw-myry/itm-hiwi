#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define WS2812_PIN 10
#define WS2812_COUNT 1

Adafruit_NeoPixel ws2812(
  WS2812_COUNT,
  WS2812_PIN,
  NEO_GRB + NEO_KHZ800
);

char buffer[32];
size_t bufferIndex = 0;

void setColor(const char* command)
{
  int red;
  int green;
  int blue;

  if (sscanf(command, "%d,%d,%d", &red, &green, &blue) != 3) {
    Serial.println("ERR");
    return;
  }

  red = constrain(red, 0, 255);
  green = constrain(green, 0, 255);
  blue = constrain(blue, 0, 255);

  ws2812.setPixelColor(0, ws2812.Color(red, green, blue));
  ws2812.show();

  Serial.printf("OK %d,%d,%d\n", red, green, blue);
}

void setup()
{
  Serial.begin(115200);

  ws2812.begin();
  ws2812.setBrightness(40);
  ws2812.clear();
  ws2812.show();

  delay(500);
  Serial.println("ESP32 WS2812 serial ready");
}

void loop()
{
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());

    if (c == '\n') {
      buffer[bufferIndex] = '\0';

      if (bufferIndex > 0) {
        setColor(buffer);
      }

      bufferIndex = 0;
    }
    else if (c != '\r') {
      if (bufferIndex < sizeof(buffer) - 1) {
        buffer[bufferIndex++] = c;
      }
      else {
        bufferIndex = 0;
        Serial.println("ERR command too long");
      }
    }
  }
}