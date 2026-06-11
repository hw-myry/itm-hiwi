#include <Adafruit_NeoPixel.h>

#define LED_PIN   10
#define LED_COUNT 1

Adafruit_NeoPixel pixels(
    LED_COUNT,
    LED_PIN,
    NEO_GRB + NEO_KHZ800
);

void setup()
{
    pixels.begin();
    pixels.setBrightness(50);
}

void loop()
{
    // 红
    pixels.setPixelColor(0, pixels.Color(25, 0, 0));
    pixels.show();
    delay(1000);

    // 绿
    pixels.setPixelColor(0, pixels.Color(0, 25, 0));
    pixels.show();
    delay(1000);

    // 蓝
    pixels.setPixelColor(0, pixels.Color(0, 0, 25));
    pixels.show();
    delay(1000);

    // 白
    pixels.setPixelColor(0, pixels.Color(25, 25, 25));
    pixels.show();
    delay(1000);

    // 灭
    pixels.clear();
    pixels.show();
    delay(1000);
}