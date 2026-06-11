#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 10
#define LED_COUNT 1

const char* ssid = "MagentaWLAN-7LSH_2.4";
const char* password = "44573549123221016562";

WiFiServer server(8080);

Adafruit_NeoPixel pixels(
    LED_COUNT,
    LED_PIN,
    NEO_GRB + NEO_KHZ800
);

bool ledEnabled = false;

void setColor(int r, int g, int b)
{
    pixels.setPixelColor(0, pixels.Color(r, g, b));
    pixels.show();
}

void setup()
{
    Serial.begin(115200);

    pixels.begin();
    pixels.setBrightness(80);
    pixels.clear();
    pixels.show();

    WiFi.begin(ssid, password);

    Serial.print("Connecting WiFi");

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WiFi Connected");

    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    server.begin();
    Serial.println("TCP Server Started");
}

void loop()
{
    WiFiClient client = server.available();

    if (client)
    {
        Serial.println("Client Connected");

        while (client.connected())
        {
            if (client.available())
            {
                String cmd = client.readStringUntil('\n');
                cmd.trim();

                Serial.print("Recv: ");
                Serial.println(cmd);

                if (cmd == "LED_ON")
                {
                    ledEnabled = true;

                    setColor(255, 255, 255);

                    client.println("OK LED_ON");
                    Serial.println("LED Enabled");
                }
                else if (cmd == "LED_OFF")
                {
                    ledEnabled = false;

                    pixels.clear();
                    pixels.show();

                    client.println("OK LED_OFF");
                    Serial.println("LED Disabled");
                }
                else if (cmd.startsWith("RGB:"))
                {
                    if (ledEnabled == false)
                    {
                        client.println("LED IS OFF");
                        Serial.println("Ignore RGB, LED is OFF");
                        continue;
                    }

                    String rgb = cmd.substring(4);

                    int p1 = rgb.indexOf(',');
                    int p2 = rgb.indexOf(',', p1 + 1);

                    if (p1 > 0 && p2 > p1)
                    {
                        int r = rgb.substring(0, p1).toInt();
                        int g = rgb.substring(p1 + 1, p2).toInt();
                        int b = rgb.substring(p2 + 1).toInt();

                        r = constrain(r, 0, 255);
                        g = constrain(g, 0, 255);
                        b = constrain(b, 0, 255);

                        setColor(r, g, b);

                        client.println("OK RGB");
                    }
                    else
                    {
                        client.println("RGB FORMAT ERROR");
                    }
                }
                else
                {
                    client.println("UNKNOWN CMD");
                }
            }
        }

        client.stop();
        Serial.println("Client Disconnected");
    }
}