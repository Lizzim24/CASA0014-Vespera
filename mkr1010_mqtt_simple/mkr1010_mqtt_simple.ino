// Duncan Wilson Oct 2025 - v1 - MQTT messager to vespera

// works with MKR1010
// VESPERA Multi-Mode Controller (LDR inverse + Modes + Button)
// Modes: CCT / RAINBOW / COMET / BREATH / TWINKLE
// Short press: next mode;  Long press: power toggle

#include <SPI.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>
#include "arduino_secrets.h"
#include <utility/wifi_drv.h>  // library to drive to RGB LED on the MKR1010

/*
**** please enter your sensitive data in the Secret tab/arduino_secrets.h
**** using format below
#define SECRET_SSID "ssid name"
#define SECRET_PASS "ssid password"
#define SECRET_MQTTUSER "user name - eg student"
#define SECRET_MQTTPASS "password";
 */
const char* ssid = SECRET_SSID;
const char* password = SECRET_PASS;
const char* ssid1 = SECRET_SSID1;
const char* password1 = SECRET_PASS1;
const char* mqtt_username = SECRET_MQTTUSER;
const char* mqtt_password = SECRET_MQTTPASS;
const char* mqtt_server = "mqtt.cetools.org";
const int mqtt_port = 1884;

// Pins
const int PIN_LDR = A0;
const int PIN_BTN = 2;

// create wifi object and mqtt object
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Make sure to update your lightid value below with the one you have been allocated
String lightId = "26";  // the topic id number or user number being used.

// Here we define the MQTT topic we will be publishing data to
String mqtt_topic = "student/CASA0014/luminaire/" + lightId;
String clientId = "";  // will set once i have mac address so that it is unique

// Topics
String topic_prefix = "student/CASA0014/luminaire/" + lightId;
// String TOPIC_COLOR = topic_prefix + "/color";            // "#RRGGBB"
String TOPIC_BRIGHTNESS = topic_prefix + "/brightness";  // "0..255"
String TOPIC_POWER = topic_prefix + "/power";            // "on" / "off"
String topic_mode = topic_prefix + "/mode";              // "cct/rainbow/comet/breath/twinkle"
String TOPIC_POWER_CMD = topic_prefix + "/cmd/power";
String TOPIC_MODE_CMD = topic_prefix + "/cmd/mode"; 
// String topic_speed      = topic_prefix + "/speed";      // numeric string interpreted by current mode

// NeoPixel Configuration - we need to know this to know how to send messages
// to vespera
const int num_leds = 72;
const int payload_size = num_leds * 3;  // x3 for RGB

// Create the byte array to send in MQTT payload this stores all the colours
// in memory so that they can be accessed in for example the rainbow function
byte RGBpayload[payload_size];

// Color State - red, orange, yellow, green, cyan, blue, violet
struct RGB {
  uint8_t r, g, b;
};
//RGB COLORS[7] = { { 255, 0, 0 }, { 255, 165, 0 }, { 255, 255, 0 }, { 0, 255, 0 }, { 0, 255, 255 }, { 0, 0, 255 }, { 128, 0, 255 }};
RGB currentColor = { 255, 180, 100 };

//int colorIndex = 0;
enum Mode { M_CCT = 0,
            M_RAINBOW,
            M_COMET,
            M_TWINKLE,
            M_BREATH,
            MODE_COUNT };
Mode mode = M_CCT;

const char* modeToStr(Mode m) {
  switch (m) {
    case M_CCT:     return "cct";
    case M_RAINBOW: return "rainbow";
    case M_COMET:   return "comet";
    case M_TWINKLE: return "twinkle";
    case M_BREATH:  return "breath";
    default:        return "cct";
  }
}

bool strToMode(const String& s, Mode& out) {
  if      (s.equalsIgnoreCase("cct"))     out = M_CCT;
  else if (s.equalsIgnoreCase("rainbow")) out = M_RAINBOW;
  else if (s.equalsIgnoreCase("comet"))   out = M_COMET;
  else if (s.equalsIgnoreCase("twinkle")) out = M_TWINKLE;
  else if (s.equalsIgnoreCase("breath"))  out = M_BREATH;
  else return false;
  return true;
}

// LDR map & gamma
int lastBrightness = -1;
int LDR_MIN = 220;
int LDR_MAX = 820;
float GAMMA = 2.2f;

// Heartbeat & thresholds
const int Bright_delta = 5;
const unsigned long Heartbeat_ms = 2000;
unsigned long lastHeartbeat = 0;

// Button State (debounce + short/long press)
bool lastBtnLevel = HIGH;
unsigned long lastBtnChange = 0;
bool btnPressed = false;
unsigned long pressStart = 0;
const unsigned long Debounce_ms = 40;
const unsigned long Long_Press_ms = 600;
// Power command cooldown (shared by button + MQTT)

bool powerOn = true;
unsigned long lastPowerChange = 0;
const unsigned long POWER_COOLDOWN_MS = 300;

const unsigned long FRAME_MS = 40;  // ~25 FPS
unsigned long lastFrameMs = 0;

// Anim params
float hueBase = 0.0f;  // base hue (0..360)
float rainbowSpeed = 40.0f;
float cometSpeed = 60.0f;
float breathSpeed = 0.6f;
float twinkleProb = 0.02f;


//Helpers
void publish(const String& topic, const String& payload, bool retain) {
  mqttClient.publish(topic.c_str(), payload.c_str(), retain);
  Serial.print("MQTT -> ");
  Serial.print(topic);
  Serial.print(" : ");
  Serial.println(payload);
}
static inline void setPixelRGB(int i, uint8_t R, uint8_t G, uint8_t B) {
  if (i < 0 || i >= num_leds) return;
  RGBpayload[i*3+0] = R;
  RGBpayload[i*3+1] = G;
  RGBpayload[i*3+2] = B;
}

static inline void fillStripRGB(uint8_t R, uint8_t G, uint8_t B) {
  for (int i = 0; i < num_leds; i++) setPixelRGB(i, R, G, B);
}

RGB lerp(RGB a, RGB b, uint8_t t) {
  RGB o;
  o.r = a.r + ((int)b.r - a.r) * t / 255;
  o.g = a.g + ((int)b.g - a.g) * t / 255;
  o.b = a.b + ((int)b.b - a.b) * t / 255;
  return o;
}
RGB hsv2rgb(float H, float S, float V) {
  while (H < 0) H += 360;
  while (H >= 360) H -= 360;
  float C = V * S, X = C * (1 - fabs(fmod(H / 60.0, 2) - 1)), m = V - C;
  float r = 0, g = 0, b = 0;
  if (H < 60) {
    r = C;
    g = X;
  } else if (H < 120) {
    r = X;
    g = C;
  } else if (H < 180) {
    g = C;
    b = X;
  } else if (H < 240) {
    g = X;
    b = C;
  } else if (H < 300) {
    r = X;
    b = C;
  } else {
    r = C;
    b = X;
  }
  RGB o = { (uint8_t)((r + m) * 255), (uint8_t)((g + m) * 255), (uint8_t)((b + m) * 255) };
  return o;
}

/* void applyColorToPayload(RGB c, uint8_t brightness, bool on) {
  uint8_t R = on ? (uint16_t)c.r * brightness / 255 : 0;
  uint8_t G = on ? (uint16_t)c.g * brightness / 255 : 0;
  uint8_t B = on ? (uint16_t)c.b * brightness / 255 : 0;
  for (int i = 0; i < num_leds; i++) {
    RGBpayload[i * 3 + 0] = R;
    RGBpayload[i * 3 + 1] = G;
    RGBpayload[i * 3 + 2] = B;
  }
}
 */

void publishStrip() {
  if (mqttClient.connected()) {
    mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size);
    Serial.println("Published full strip payload.");
  } else {
    Serial.println("MQTT not coneected; cannot publish strip payload.");
  }
}

/* void setOnboardLED(RGB c, uint8_t brightness, bool on) {
  // Simple onboard preview (mkr1010 wifidrv pins 25:R 26:G 27:B)
  uint8_t R = on ? (uint16_t)c.r * brightness / 255 : 0;
  uint8_t G = on ? (uint16_t)c.g * brightness / 255 : 0;
  uint8_t B = on ? (uint16_t)c.b * brightness / 255 : 0;
  WiFiDrv::analogWrite(25, R);
  WiFiDrv::analogWrite(26, G);
  WiFiDrv::analogWrite(27, B);
}
 */

// Brightness (LDR → 0..255, inverted + gamma)
int readBrightnessInverted() {
  // 5-sample smoothing
  const int N = 5;
  long acc = 0;
  for (int i = 0; i < N; i++) {
    acc += analogRead(PIN_LDR);
    delay(2);
  }
  int raw = acc / N;
  /*   // int raw = 1023 = (acc / N);

  // Surrounding env brighter -> light darker
  int mapped = map(raw, 200, 800, 255, 0);
  if (mapped < 0) mapped = 0;
  if (mapped > 255) mapped = 255; 
  return mapped;*/

  int x = map(raw, LDR_MIN, LDR_MAX, 0, 255);
  x = constrain(x, 0, 255);
  int inv = 255 - x;  // brighter outside -> lower brightness
  // gamma
  float f = inv / 255.0f;
  f = powf(f, GAMMA);
  int v = (int)(f * 255.0f + 0.5f);
  v = constrain(v, 0, 255);
  return v;
}

// Button: short→next mode, long→power toggle
void handleButton() {

  bool level = digitalRead(PIN_BTN);
  unsigned long now = millis();

  if (level != lastBtnLevel && (now - lastBtnChange) > Debounce_ms) {
    lastBtnChange = now;
    lastBtnLevel = level;

    if (level == LOW) {  // pressed
      btnPressed = true;
      pressStart = now;
    } else {  // released
      if (!btnPressed) return;
      btnPressed = false;

      unsigned long dur = now - pressStart;
      if (dur >= Long_Press_ms) {

        if (now - lastPowerChange >= POWER_COOLDOWN_MS) {
          lastPowerChange = now;
          powerOn = !powerOn;
          publish(TOPIC_POWER, powerOn ? "on" : "off", true);

          int b = (powerOn && lastBrightness >= 0) ? lastBrightness : 0;
          renderFrame(mode, b, now);
          if (mqttClient.connected()) mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size);
        }
      } else {

        int next = (int)mode + 1;
        if (next >= (int)MODE_COUNT) next = 0;
        mode = (Mode)next; 
        publish(topic_mode, modeToStr(mode), true);
        Serial.print("[btn] mode -> ");


        int b = (powerOn && lastBrightness >= 0) ? lastBrightness : 0;
        renderFrame(mode, b, now);
        if (mqttClient.connected()) mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size);
      }
    }
  }
}

/* if (level != HIGH && (now - lastBtnChange) > Debounce_ms) {
    lastBtnChange = now;
    lastBtnLevel = level;

    if (level == LOW) {
      //pressed
      btnPressed = true;
      pressStart = now;
    } else {
      //released
      if (btnPressed) {
        btnPressed = false;
        unsigned long dur = now - pressStart;
        if (dur >= Long_Press_ms) {
          //long press => toggle power
          powerOn = !powerOn;
          publish(TOPIC_POWER, powerOn ? "on" : "off", true);

          // render immediately after power change
          int b = (powerOn && lastBrightness >= 0) ? lastBrightness : 0;
          renderAndPublish(b);

        } else {
          //short press => cycle mode
          mode = (Mode)((mode + 1) % MODE_COUNT);
          String m = (mode==M_CCT?"cct":mode==M_RAINBOW?"rainbow":mode==M_COMET?"comet":mode==M_BREATH?"breath":"twinkle");
          publish(topic_mode, m, true);
          // also echo current speed for clarity
          publish(topic_speed, String( (mode==M_RAINBOW)? speedRainbow :
                                       (mode==M_COMET)  ? speedComet   :
                                       (mode==M_BREATH) ? speedBreath  :
                                       (mode==M_TWINKLE)? twinkleProb  : 0.0f ),
                  false);
          
          // NEW: render immediately after mode change
          int b = (powerOn && lastBrightness >= 0) ? lastBrightness : 0;
          renderAndPublish(b);
        }
      }
    } 
  }
}*/

/*   // Rendering and Publish
  void renderAndPublish(int brightness) {
  if (!powerOn) brightness = 0;
  unsigned long ms = millis(); 

    switch (mode) {
      case M_CCT:
        {
          // bright->cool (#80D8FF), dark->warm (#FFC180)
          RGB cool = { 128, 216, 255 };
          RGB warm = { 255, 193, 128 };
          currentColor = lerp(cool, warm, (uint8_t)brightness);
          // scale strip by brightness
          uint16_t r = ((uint16_t)currentColor.r * brightness) / 255;
          uint16_t g = ((uint16_t)currentColor.g * brightness) / 255;
          uint16_t b = ((uint16_t)currentColor.b * brightness) / 255;
          fillStrip({ (uint8_t)r, (uint8_t)g, (uint8_t)b });
          break;
        }
        /* case M_RAINBOW:
      {
        hueOffset = fmod(hueOffset + speedRainbow, 360.0f);
        for (int i = 0; i < num_leds; i++) {
          float H = fmod(hueBase + i * 2.0f + hueOffset, 360.0f);
          RGB px = hsv2rgb(H, 1.0f, brightness / 255.0f);
          setPixel(i, px);
        }
        currentColor = hsv2rgb(fmod(hueBase + hueOffset, 360.0f), 1.0f, 1.0f);
        break;
      }
    case M_COMET:
      {
        static float cometPos = 0.0f;
        static unsigned long lastMs = 0;
        unsigned long nowMs = ms;
        float dt = (lastMs == 0) ? 0.0f : (nowMs - lastMs) / 1000.0f;  // 秒
        lastMs = nowMs;

        cometPos += speedComet * dt;
        while (cometPos >= num_leds) cometPos -= num_leds;
        while (cometPos < 0) cometPos += num_leds;

        int head = (int)cometPos;
        float H = 210.0f; 
        for (int i = 0; i < num_leds; i++) {
          int dist = (i <= head) ? (head - i) : (head + num_leds - i);
          float fade = expf(-dist * 0.35f);
          uint8_t v = (uint8_t)(brightness * fade);
          RGB c = hsv2rgb(H, 1.0f, v / 255.0f);
          setPixel(i, c);
        }
        currentColor = hsv2rgb(H, 1.0f, 1.0f);
        break;
      }
    case M_BREATH:
      {
        float t = ms * (speedBreath * 2 * PI / 1000.0f);
        float lfo = 0.35f + 0.65f * (0.5f + 0.5f * sinf(t));  // 0.35..1.0
        uint8_t v = (uint8_t)(brightness * lfo);
        if (currentColor.r == 0 && currentColor.g == 0 && currentColor.b == 0) currentColor = { 200, 200, 255 };
        uint16_t r = ((uint16_t)currentColor.r * v) / 255;
        uint16_t g = ((uint16_t)currentColor.g * v) / 255;
        uint16_t b = ((uint16_t)currentColor.b * v) / 255;
        fillStrip({ (uint8_t)r, (uint8_t)g, (uint8_t)b });
        break;
      }
    case M_TWINKLE:
      {
        RGB base = { 10, 10, 20 };
        uint16_t r = ((uint16_t)base.r * brightness) / 255;
        uint16_t g = ((uint16_t)base.g * brightness) / 255;
        uint16_t b = ((uint16_t)base.b * brightness) / 255;
        fillStrip({ (uint8_t)r, (uint8_t)g, (uint8_t)b });
        int stars = max(1, num_leds / 24);
        for (int k = 0; k < stars; k++) {
          if (random(0, 1000) < (int)(twinkleProb * 1000)) {
            int p = random(0, num_leds);
            float H = fmod(hueBase + random(0, 360), 360.0f);
            RGB s = hsv2rgb(H, 0.2f + (random(0, 80) / 100.0f), 1.0f);
            setPixel(p, s);
          }
        }
        currentColor = { 200, 220, 255 };
        break;
      } 
    }

    // publish prefix payload
    if (mqttClient.connected()) mqttClient.publish(topic_prefix.c_str(), RGBpayload, payload_size);

    // publish readable state
    publish(TOPIC_BRIGHTNESS, String(brightness), false);
    char hexBuf[10];
    snprintf(hexBuf, sizeof(hexBuf), "#%02X%02X%02X", currentColor.r, currentColor.g, currentColor.b);
    publish(TOPIC_COLOR, String(hexBuf), false);
  } */
void renderFrame(Mode m, int brightness, unsigned long ms) {
  if (!powerOn) brightness = 0;

  switch (m) {
    case M_CCT:
      {
        RGB cool = { 128, 216, 255 };
        RGB warm = { 255, 193, 128 };
        currentColor = lerp(cool, warm, (uint8_t)brightness);
        uint8_t R = (uint16_t)currentColor.r * brightness / 255;
        uint8_t G = (uint16_t)currentColor.g * brightness / 255;
        uint8_t B = (uint16_t)currentColor.b * brightness / 255;
        fillStripRGB(R, G, B);
        break;
      }

    case M_RAINBOW:
      {
        float dt = (lastFrameMs == 0) ? 0.0f : (ms - lastFrameMs) / 1000.0f;
        hueBase += rainbowSpeed * dt;
        if (hueBase >= 360.f) hueBase -= 360.f;
        for (int i = 0; i < num_leds; i++) {
          float H = fmodf(hueBase + i * 3.0f, 360.0f);
          RGB px = hsv2rgb(H, 1.0f, brightness / 255.0f);
          setPixelRGB(i, px.r, px.g, px.b);
        }
        break;
      }

    case M_COMET:
      {
        static float cometPos = 0.0f;
        static unsigned long lastMs = 0;
        float dt = (lastMs == 0) ? 0.0f : (ms - lastMs) / 1000.0f;
        lastMs = ms;

        cometPos += cometSpeed * dt;
        while (cometPos >= num_leds) cometPos -= num_leds;

        uint8_t base = (uint8_t)(brightness * 0.05f);
        fillStripRGB(base, base, base);

        int head = (int)cometPos;
        for (int k = 0; k < num_leds; k++) {
          int dist = (k <= head) ? (head - k) : (head + num_leds - k);
          float fade = expf(-dist * 0.35f);
          uint8_t v = (uint8_t)(brightness * fade);
          RGB c = hsv2rgb(200.0f, 1.0f, v / 255.0f);
          int R = min(255, (int)RGBpayload[k * 3 + 0] + c.r);
          int G = min(255, (int)RGBpayload[k * 3 + 1] + c.g);
          int B = min(255, (int)RGBpayload[k * 3 + 2] + c.b);
          setPixelRGB(k, (uint8_t)R, (uint8_t)G, (uint8_t)B);
        }
        break;
      }

    case M_TWINKLE:
      {
        uint8_t baseR = (uint8_t)(brightness * 10 / 255);
        uint8_t baseG = (uint8_t)(brightness * 12 / 255);
        uint8_t baseB = (uint8_t)(brightness * 20 / 255);
        fillStripRGB(baseR, baseG, baseB);
        for (int i = 0; i < num_leds; i++) {
          if ((float)random(0, 1000) / 1000.0f < twinkleProb) {
            float H = random(180, 260);
            RGB s = hsv2rgb(H, 0.3f + random(0, 60) / 100.0f, 1.0f);
            uint8_t R = (uint16_t)s.r * brightness / 255;
            uint8_t G = (uint16_t)s.g * brightness / 255;
            uint8_t B = (uint16_t)s.b * brightness / 255;
            setPixelRGB(i, R, G, B);
          }
        }
        break;
      }

    case M_BREATH:
      {
        float t = ms * (breathSpeed * 2 * PI / 1000.0f);
        float lfo = 0.3f + 0.7f * (0.5f + 0.5f * sinf(t));  // 0.3..1.0
        uint8_t v = (uint8_t)(brightness * lfo);
        RGB base = { 220, 220, 255 };
        uint8_t R = (uint16_t)base.r * v / 255;
        uint8_t G = (uint16_t)base.g * v / 255;
        uint8_t B = (uint16_t)base.b * v / 255;
        fillStripRGB(R, G, B);
        break;
      }
  }

  lastFrameMs = ms;
}


void setup() {
  Serial.begin(115200);
  //while (!Serial); // Wait for serial port to connect (useful for debugging)
  Serial.println("Vespera");

  pinMode(PIN_LDR, INPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);
  lastBtnLevel = digitalRead(PIN_BTN);
  randomSeed(analogRead(PIN_LDR));

  //onboard RGB init (25:R 26:G 27:B)
  WiFiDrv::pinMode(25, OUTPUT);
  WiFiDrv::pinMode(26, OUTPUT);
  WiFiDrv::pinMode(27, OUTPUT);

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  printMacAddress(mac);

  Serial.print("This device is Vespera ");
  Serial.println(lightId);

  // Connect to WiFi
  startWifi();

  // Connect to MQTT broker
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setBufferSize(2000);
  mqttClient.setCallback(callback);

  Serial.println("Set-up complete");

  // init strip to black
  fillStripRGB( 0, 0, 0 );
  if (mqttClient.connected()) mqttClient.publish(topic_prefix.c_str(), RGBpayload, payload_size);

  Serial.println("Set-up complete");
}

void loop() {
  // Reconnect if necessary
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }

  if (WiFi.status() != WL_CONNECTED) {
    startWifi();
  }
  // keep mqtt alive
  mqttClient.loop();

  //handle input
  handleButton();

  //read brightness (inverse mapping)
  int brightness = readBrightnessInverted();
  unsigned long now = millis();

  bool isAnimated = (mode != M_CCT);
  bool timeForFrame = isAnimated && (now - lastFrameMs >= FRAME_MS);

  bool needStateBeat = (now - lastHeartbeat > Heartbeat_ms);
  bool brightnessChanged = (lastBrightness < 0 || abs(brightness - lastBrightness) >= Bright_delta);

  if (isAnimated) {
    if (timeForFrame) {
      renderFrame(mode, brightness, now);
      if (mqttClient.connected()) mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size);
    }
  } else {
    if (brightnessChanged || needStateBeat) {
      renderFrame(mode, brightness, now);
      if (mqttClient.connected()) mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size);
    }
  }

  if (brightnessChanged || needStateBeat) {
    lastHeartbeat = now;
    lastBrightness = brightness;
    publish(TOPIC_BRIGHTNESS, String(powerOn ? brightness : 0), false);
  }

  WiFiDrv::analogWrite(25, RGBpayload[0]);
  WiFiDrv::analogWrite(26, RGBpayload[1]);
  WiFiDrv::analogWrite(27, RGBpayload[2]);

  /*   bool needPublish = false;

  if (lastBrightness < 0 || abs(brightness - lastBrightness) >= Bright_delta) {
    lastBrightness = brightness;
    needPublish = true;
  }
  if ((now - lastHeartbeat) > Heartbeat_ms) {
    lastHeartbeat = now;
    needPublish = true;
  }

  if (needPublish) {
    applyColorToPayload(COLORS[colorIndex], (uint8_t)(powerOn ? brightness : 0), powerOn);
    publishStrip();
    publish(TOPIC_BRIGHTNESS, String(powerOn ? brightness : 0), false);
  }

  setOnboardLED(COLORS[colorIndex], (uint8_t)(powerOn ? brightness : 0), powerOn); */

  delay(10);
}
