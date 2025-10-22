// Duncan Wilson Oct 2025 - v1 - MQTT messager to vespera

// works with MKR1010

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
String TOPIC_COLOR = topic_prefix + "/color";            // "#RRGGBB"
String TOPIC_BRIGHTNESS = topic_prefix + "/brightness";  // "0..255"
String TOPIC_POWER = topic_prefix + "/power";            // "on" / "off"

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
RGB COLORS[7] = {
  { 255, 0, 0 },
  { 255, 165, 0 },
  { 255, 255, 0 },
  { 0, 255, 0 },
  { 0, 255, 255 },
  { 0, 0, 255 },
  { 128, 0, 255 }
};

int colorIndex = 0;
bool powerOn = true;

int lastBrightness = -1;
const int Bright_delta = 5;
const unsigned long Heartbeat_ms = 5000;
unsigned long lastHeartbeat = 0;

// Button State (debounce + short/long press)
bool lastBtnLevel = HIGH;
unsigned long lastBtnChange = 0;
bool btnPressed = false;
unsigned long pressStart = 0;
const unsigned long Debounce_ms = 30;
const unsigned long Long_Press_ms = 800;

// Forward Declarations
void toggleRGB();
void LedRed();
void LedGreen();
void LedBlue();

void startWifi();
void reconnectMQTT();
void printMacAddress(byte mac[]);
void callback(char* topic, byte* payload, unsigned int length);

void publish(const String& topic, const String& payload, bool retain);
void publishStrip();
void applyColorToPayload(RGB c, uint8_t brightness, bool on);
void setOnboardLED(RGB c, uint8_t brightness, bool on);
int readBrightnessInverted();
void handleButton();

//Helpers
void publish(const String& topic, const String& payload, bool retain) {
  mqttClient.publish(topic.c_str(), payload.c_str(), retain);
  Serial.print("MQTT -> ");
  Serial.print(topic);
  Serial.print(" : ");
  Serial.println(payload);
}

void applyColorToPayload(RGB c, uint8_t brightness, bool on) {
  uint8_t R = on ? (uint16_t)c.r * brightness / 255 : 0;
  uint8_t G = on ? (uint16_t)c.g * brightness / 255 : 0;
  uint8_t B = on ? (uint16_t)c.b * brightness / 255 : 0;
  for (int i = 0; i < num_leds; i++) {
    RGBpayload[i * 3 + 0] = R;
    RGBpayload[i * 3 + 1] = G;
    RGBpayload[i * 3 + 2] = B;
  }
}

void publishStrip() {
  if (mqttClient.connected()) {
    mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size);
    Serial.println("Published full strip payload.");
  } else {
    Serial.println("MQTT not coneected; cannot publish strip payload.");
  }
}

void setOnboardLED(RGB c, uint8_t brightness, bool on) {
  // Simple onboard preview (mkr1010 wifidrv pins 25:R 26:G 27:B)
  uint8_t R = on ? (uint16_t)c.r * brightness / 255 : 0;
  uint8_t G = on ? (uint16_t)c.g * brightness / 255 : 0;
  uint8_t B = on ? (uint16_t)c.b * brightness / 255 : 0;
  WiFiDrv::analogWrite(25, R);
  WiFiDrv::analogWrite(26, G);
  WiFiDrv::analogWrite(27, B);
}

int readBrightnessInverted() {
  // 5-sample smoothing
  const int N = 5;
  long acc = 0;
  for (int i = 0; i < N; i++) {
    acc += analogRead(PIN_LDR);
    delay(2);
  }
  int raw = acc / N;
  // int raw = 1023 = (acc / N);

  // Surrounding env brighter -> light darker
  int mapped = map(raw, 200, 800, 255, 0);
  if (mapped < 0) mapped = 0;
  if (mapped > 255) mapped = 255;
  return mapped;
}

void handleButton() {
  bool level = digitalRead(PIN_BTN);
  unsigned long now = millis();

  if (level != lastBtnLevel && (now - lastBtnChange) > Debounce_ms) {
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
        } else {
          //short press => cycle color
          colorIndex = (colorIndex + 1) % 7;
          char hexBuf[10];
          snprintf(hexBuf, sizeof(hexBuf), "#%02X%02X%02X", COLORS[colorIndex].r, COLORS[colorIndex].g, COLORS[colorIndex].b);
          publish(TOPIC_COLOR, String(hexBuf), true);
        }
      }
    }
  }
}


void setup() {
  Serial.begin(115200);
  //while (!Serial); // Wait for serial port to connect (useful for debugging)
  Serial.println("Vespera");

  pinMode(PIN_LDR, INPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);

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
  bool needPublish = false;

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

  setOnboardLED(COLORS[colorIndex], (uint8_t)(powerOn ? brightness : 0), powerOn);

  delay[10];
}
