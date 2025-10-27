// Duncan Wilson Oct 2025 - v1 - MQTT messager to vespera
// works with MKR1010

// VESPERA Multi-Mode Controller (LDR inverse + Modes + Button)
// Modes: CCT / RAINBOW / COMET /  TWINKLE / BREATH
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
String TOPIC_BRIGHTNESS = topic_prefix + "/brightness";  // readable brightness (0..255)
String TOPIC_POWER = topic_prefix + "/power";            // readable power state: on/off
String topic_mode = topic_prefix + "/mode";              // readable mode name
String TOPIC_POWER_CMD = topic_prefix + "/cmd/power";    // command topic: on/off (not retained)
String TOPIC_MODE_CMD = topic_prefix + "/cmd/mode";      // command topic: cct/rainbow/...

// NeoPixel Configuration - we need to know this to know how to send messages
// to vespera
const int num_leds = 72;
const int payload_size = num_leds * 3;  // x3 for RGB

// Create the byte array to send in MQTT payload this stores all the colours
// in memory so that they can be accessed in for example the rainbow function
byte RGBpayload[payload_size];

// Simple RGB struct for helper functions
struct RGB { uint8_t r, g, b; };
RGB currentColor = { 255, 180, 100 };

// Operating modes
enum Mode { M_CCT = 0,
            M_RAINBOW,
            M_COMET,
            M_TWINKLE,
            M_BREATH,
            MODE_COUNT };
Mode mode = M_CCT;

// Helper: map enum to string
const char* modeToStr(Mode m) {
  switch (m) {
    case M_CCT: return "cct";
    case M_RAINBOW: return "rainbow";
    case M_COMET: return "comet";
    case M_TWINKLE: return "twinkle";
    case M_BREATH: return "breath";
    default: return "cct";
  }
}

// Helper: parse string to enum
bool strToMode(const String& s, Mode& out) {
  if (s.equalsIgnoreCase("cct")) out = M_CCT;
  else if (s.equalsIgnoreCase("rainbow")) out = M_RAINBOW;
  else if (s.equalsIgnoreCase("comet")) out = M_COMET;
  else if (s.equalsIgnoreCase("twinkle")) out = M_TWINKLE;
  else if (s.equalsIgnoreCase("breath")) out = M_BREATH;
  else return false;
  return true;
}

// LDR map & gamma
int lastBrightness = -1; // last published brightness (-1 = not set yet)
int LDR_MIN = 220;  // tunable: raw value at "dark"
int LDR_MAX = 820;  // tunable: raw value at "bright"
float GAMMA = 2.2f;  // gamma correction (perceptual)

// Heartbeat & thresholds
const int Bright_delta = 5;  // publish only if change >= 5
const unsigned long Heartbeat_ms = 2000; // periodic status every 2s
unsigned long lastHeartbeat = 0; // last heartbeat time

// Button State (debounce + short/long press)
bool lastBtnLevel = HIGH;  // remember last raw level (INPUT_PULLUP)
unsigned long lastBtnChange = 0;  // last time we accepted a level change
bool btnPressed = false;  // whether currently pressed
unsigned long pressStart = 0;  // when press started
const unsigned long Debounce_ms = 40;  // debounce window
const unsigned long Long_Press_ms = 600;  // threshold for long press

// Power state and cooldown to avoid rapid flapping
bool powerOn = true;  // current power state
unsigned long lastPowerChange = 0;  // last time power toggled
const unsigned long POWER_COOLDOWN_MS = 300;  // ignore power toggles within 300ms

const unsigned long FRAME_MS = 40;  // ~25 FPS
unsigned long lastFrameMs = 0;

// Anim params
float hueBase = 0.0f;  // base hue (0..360)
float rainbowSpeed = 40.0f;  // degrees per second for rainbow scroll
float cometSpeed = 60.0f;  // pixels per second for comet head
float breathSpeed = 0.6f;  // Hz for breathing
float twinkleProb = 0.02f;  // per-pixel twinkle probability per frame


// Publish helper: string payload
void publish(const String& topic, const String& payload, bool retain) {
  mqttClient.publish(topic.c_str(), payload.c_str(), retain);
  Serial.print("MQTT -> ");
  Serial.print(topic);
  Serial.print(" : ");
  Serial.println(payload);
}

// Set a single pixel in the raw payload (RGBpayload)
static inline void setPixelRGB(int i, uint8_t R, uint8_t G, uint8_t B) {
  if (i < 0 || i >= num_leds) return;
  RGBpayload[i * 3 + 0] = R;
  RGBpayload[i * 3 + 1] = G;
  RGBpayload[i * 3 + 2] = B;
}

// Fill the entire strip with a solid RGB color
static inline void fillStripRGB(uint8_t R, uint8_t G, uint8_t B) {
  for (int i = 0; i < num_leds; i++) setPixelRGB(i, R, G, B);
}

// Linear interpolation between two RGB colors
RGB lerp(RGB a, RGB b, uint8_t t) {
  RGB o;
  o.r = a.r + ((int)b.r - a.r) * t / 255;
  o.g = a.g + ((int)b.g - a.g) * t / 255;
  o.b = a.b + ((int)b.b - a.b) * t / 255;
  return o;
}

// HSV -> RGB conversion (H:0..360, S/V:0..1)
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

// Publish the entire RGBpayload to the raw topic
void publishStrip() {
  if (mqttClient.connected()) {
    mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size);
    Serial.println("Published full strip payload.");
  } else {
    Serial.println("MQTT not coneected; cannot publish strip payload.");
  }
}

// Brightness (LDR → 0..255, inverted + gamma)
int readBrightnessInverted() {
  // 5-sample smoothing
  const int N = 5;
  long acc = 0;
  for (int i = 0; i < N; i++) {
    acc += analogRead(PIN_LDR);  // read raw ADC 0..1023
    delay(2);   // small pause to filter noise
  }
  int raw = acc / N;  // averaged raw

  int x = map(raw, LDR_MIN, LDR_MAX, 0, 255);  // normalize
  x = constrain(x, 0, 255);
  int inv = 255 - x;  // brighter outside -> lower brightness
  // gamma
  float f = inv / 255.0f;  // normalize to 0..1
  f = powf(f, GAMMA);  // gamma correction
  int v = (int)(f * 255.0f + 0.5f);  // back to 0..255
  v = constrain(v, 0, 255);
  return v;
}

// Button: short→next mode, long→power toggle
void handleButton() {
  bool level = digitalRead(PIN_BTN);  // read raw button level
  unsigned long now = millis();  // current time

  // simple debounce: react only if level changed and debounce time passed
  if (level != lastBtnLevel && (now - lastBtnChange) > Debounce_ms) {
    lastBtnChange = now;
    lastBtnLevel = level;

    if (level == LOW) {  // pressed
      btnPressed = true;
      pressStart = now;
    } else {  // released
      if (!btnPressed) return;
      btnPressed = false;

      unsigned long dur = now - pressStart;  // press duration

      if (dur >= Long_Press_ms) {  // long press => power toggle
        if (now - lastPowerChange >= POWER_COOLDOWN_MS) {
          lastPowerChange = now;
          powerOn = !powerOn;  // toggle state
          publish(TOPIC_POWER, powerOn ? "on" : "off", true);  // publish status (retained)
          int b = (powerOn && lastBrightness >= 0) ? lastBrightness : 0;
          renderFrame(mode, b, now);  // immediate render
          if (mqttClient.connected()) mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size);
        }
      } else {  // short press => next mode
        int next = (int)mode + 1;
        if (next >= (int)MODE_COUNT) next = 0;
        mode = (Mode)next;  // set mode
        publish(topic_mode, modeToStr(mode), true);  // announce mode (retained)
        Serial.print("[btn] mode -> ");  // debug
        int b = (powerOn && lastBrightness >= 0) ? lastBrightness : 0;
        renderFrame(mode, b, now);
        if (mqttClient.connected()) mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size);
      }
    }
  }
}

// Render a frame for current mode at given brightness/time
void renderFrame(Mode m, int brightness, unsigned long ms) {
  if (!powerOn) brightness = 0;  // force black when power is off

  switch (m) {
    case M_CCT:   // CCT: cool↔warm based on brightness
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

    case M_RAINBOW:   // Rainbow: hue scroll along time + position
      {
        float dt = (lastFrameMs == 0) ? 0.0f : (ms - lastFrameMs) / 1000.0f;
        hueBase += rainbowSpeed * dt;   // advance hue by speed
        if (hueBase >= 360.f) hueBase -= 360.f;  // wrap hue
        for (int i = 0; i < num_leds; i++) {
          float H = fmodf(hueBase + i * 3.0f, 360.0f);
          RGB px = hsv2rgb(H, 1.0f, brightness / 255.0f);
          setPixelRGB(i, px.r, px.g, px.b);
        }
        break;
      }

    case M_COMET:   // Comet: moving head with exponential tail
      {
        static float cometPos = 0.0f;
        static unsigned long lastMs = 0;
        float dt = (lastMs == 0) ? 0.0f : (ms - lastMs) / 1000.0f;  // time delta in seconds
        lastMs = ms;

        cometPos += cometSpeed * dt;   // move comet by speed
        while (cometPos >= num_leds) cometPos -= num_leds;

        uint8_t base = (uint8_t)(brightness * 0.05f);  // faint background
        fillStripRGB(base, base, base);

        int head = (int)cometPos;  // current head index
        for (int k = 0; k < num_leds; k++) {
          int dist = (k <= head) ? (head - k) : (head + num_leds - k);  // wrap-around distance
          float fade = expf(-dist * 0.35f);   // exponential tail
          uint8_t v = (uint8_t)(brightness * fade);
          RGB c = hsv2rgb(200.0f, 1.0f, v / 255.0f);  // blue comet
          int R = min(255, (int)RGBpayload[k * 3 + 0] + c.r); // additive blend
          int G = min(255, (int)RGBpayload[k * 3 + 1] + c.g);
          int B = min(255, (int)RGBpayload[k * 3 + 2] + c.b);
          setPixelRGB(k, (uint8_t)R, (uint8_t)G, (uint8_t)B);
        }
        break;
      }

    case M_TWINKLE:   // Twinkle: dark base + random stars
      {
        uint8_t baseR = (uint8_t)(brightness * 10 / 255);
        uint8_t baseG = (uint8_t)(brightness * 12 / 255);
        uint8_t baseB = (uint8_t)(brightness * 20 / 255);
        fillStripRGB(baseR, baseG, baseB);
        for (int i = 0; i < num_leds; i++) {
          if ((float)random(0, 1000) / 1000.0f < twinkleProb) {
            float H = random(180, 260);    // blue-ish hues
            RGB s = hsv2rgb(H, 0.3f + random(0, 60) / 100.0f, 1.0f);  // dim saturated star
            uint8_t R = (uint16_t)s.r * brightness / 255;
            uint8_t G = (uint16_t)s.g * brightness / 255;
            uint8_t B = (uint16_t)s.b * brightness / 255;
            setPixelRGB(i, R, G, B);
          }
        }
        break;
      }

    case M_BREATH:   // Breath: sin-based luminance modulation
      {
        float t = ms * (breathSpeed * 2 * PI / 1000.0f);  // radians
        float lfo = 0.3f + 0.7f * (0.5f + 0.5f * sinf(t));  // 0.3..1.0
        uint8_t v = (uint8_t)(brightness * lfo);   // scaled brightness
        RGB base = { 220, 220, 255 };   // soft white-blue
        uint8_t R = (uint16_t)base.r * v / 255;
        uint8_t G = (uint16_t)base.g * v / 255;
        uint8_t B = (uint16_t)base.b * v / 255;
        fillStripRGB(R, G, B);
        break;
      }
  }
  lastFrameMs = ms;   // remember last render time
}


void setup() {
  Serial.begin(115200);
  //while (!Serial); // Wait for serial port to connect (useful for debugging)
  Serial.println("Vespera");

  pinMode(PIN_LDR, INPUT);  // LDR analog input
  pinMode(PIN_BTN, INPUT_PULLUP);  // Button with internal pull-up
  lastBtnLevel = digitalRead(PIN_BTN);  // Initialize last button level
  randomSeed(analogRead(PIN_LDR));  // Seed RNG (for twinkle randomness)

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
  fillStripRGB(0, 0, 0);
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

  // read current brightness from LDR (inverted & gamma)
  int brightness = readBrightnessInverted();
  unsigned long now = millis();

  // Animated modes need frequent frames; static (CCT) only on change/heartbeat
  bool isAnimated = (mode != M_CCT);
  bool timeForFrame = isAnimated && (now - lastFrameMs >= FRAME_MS);

  // Determine whether to publish state change
  bool needStateBeat = (now - lastHeartbeat > Heartbeat_ms);
  bool brightnessChanged = (lastBrightness < 0 || abs(brightness - lastBrightness) >= Bright_delta);

  if (isAnimated) {
    // For animated modes, render every frame tick
    if (timeForFrame) {
      renderFrame(mode, brightness, now);
      if (mqttClient.connected()) mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size);
    }
  } else {
    // For CCT, only render when brightness changes or heartbeat triggers
    if (brightnessChanged || needStateBeat) {
      renderFrame(mode, brightness, now);
      if (mqttClient.connected()) mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size);
    }
  }

  // Publish readable brightness (not raw strip) periodically or on change
  if (brightnessChanged || needStateBeat) {
    lastHeartbeat = now;
    lastBrightness = brightness;
    publish(TOPIC_BRIGHTNESS, String(powerOn ? brightness : 0), false);
  }

  // Onboard LED preview shows first pixel's RGB as a quick indicator
  WiFiDrv::analogWrite(25, RGBpayload[0]);
  WiFiDrv::analogWrite(26, RGBpayload[1]);
  WiFiDrv::analogWrite(27, RGBpayload[2]);

  delay(10);  // small idle delay to reduce CPU noise
}
