
// Function to handle incoming MQTT messages
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);

    // Copy payload into a temporary String (payload is not null-terminated)
  String msg;
  msg.reserve(length);
  for (unsigned int i=0; i<length; i++) msg += (char)payload[i];
  msg.trim();

  String t = String(topic);

  // Handle power on/off
  if (t == TOPIC_POWER) {
    if (msg.equalsIgnoreCase("on"))  powerOn = true;
    if (msg.equalsIgnoreCase("off")) powerOn = false;
    // reflect state to strip & onboard
    int brightness = (powerOn && lastBrightness>=0) ? lastBrightness : 0;
    applyColorToPayload(COLORS[colorIndex], (uint8_t)brightness, powerOn);
    publishStrip();
    setOnboardLED(COLORS[colorIndex], (uint8_t)brightness, powerOn);
    Serial.println(String("[cmd] power => ") + (powerOn ? "on" : "off"));
  }
  // Handle color: expect "#RRGGBB"
  else if (t == TOPIC_POWER) {
    if (msg.length() == 7 && msg[0] == '#') {
      long r = strtol(msg.substring(1,3).c_str(), NULL, 16);
      long g = strtol(msg.substring(3,5).c_str(), NULL, 16);
      long b = strtol(msg.substring(5,7).c_str(), NULL, 16);
      // find nearest palette index (optional); here we just set a custom color in slot 0
      COLORS[colorIndex] = { (uint8_t)r, (uint8_t)g, (uint8_t)b };
      int brightness = (powerOn && lastBrightness>=0) ? lastBrightness : 0;
      applyColorToPayload(COLORS[colorIndex], (uint8_t)brightness, powerOn);
      publishStrip();
      setOnboardLED(COLORS[colorIndex], (uint8_t)brightness, powerOn);
      Serial.println(String("[cmd] color => ") + msg);
    }
  }
  // brightness commands are ignored in this design (LDR drives it),
  // but you could optionally accept a manual override here.
}


// Connect wifi and get mac address for unique clientId and print out some setup info
void startWifi(){
  delay(10);
  LedBlue(); // show Blue LED when looking for wifi
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  // Check if the WiFi module is present
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // Don't continue if module is not present
    while (true);
  }

  // Try primary then fallback
  int status = WL_IDLE_STATUS;
  unsigned long t0 = millis();
  while (status != WL_CONNECTED && millis() - t0 < 15000) {
    status = WiFi.begin(ssid, password);
    Serial.print(".");
    delay(500);
  }
  if (status != WL_CONNECTED && ssid1 && strlen(ssid1) > 0) {
    Serial.println("\nPrimary failed. Trying fallback WiFi...");
    t0 = millis();
    while (status != WL_CONNECTED && millis() - t0 < 15000) {
      status = WiFi.begin(ssid1, password1);
      Serial.print(".");
      delay(500);
    }
  }

  if (status == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    LedGreen();
  } else {
    Serial.println("\nWiFi connect timeout.");
    LedRed();
  }
}


void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}


// handle reconnects to MQTT broker
void reconnectMQTT() {
  // Loop until we're reconnected
  LedBlue();

  if (clientId.length() == 0) {
    byte mac[6]; WiFi.macAddress(mac);
    char macStr[18];
    sprintf(macStr, "%02X%02X%02X%02X%02X%02X",
      mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
    clientId = String("vespera-") + macStr;
    Serial.print("ClientId: "); Serial.println(clientId);
    Serial.print("MAC address: ");
    printMacAddress(mac);
  }

  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT...");
    if (mqttClient.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT");
      // Subscribe to control topics (and raw if you want to monitor incoming)
      mqttClient.subscribe(TOPIC_POWER.c_str());
      mqttClient.subscribe(TOPIC_COLOR.c_str());
      // mqttClient.subscribe(topic_raw.c_str()); // optional: not needed usually
      Serial.println("Subscribed to topics: /power, /color");

      // Announce current state (retained)
      publish(TOPIC_POWER, powerOn ? "on" : "off", true);
      char hexBuf[10];
      snprintf(hexBuf, sizeof(hexBuf), "#%02X%02X%02X",
               COLORS[colorIndex].r, COLORS[colorIndex].g, COLORS[colorIndex].b);
      publish(TOPIC_COLOR, String(hexBuf), true);
      if (lastBrightness >= 0) publish(TOPIC_BRIGHTNESS, String(lastBrightness), false);

      LedGreen();
    } else {
      Serial.print("Failed, rc="); Serial.print(mqttClient.state());
      Serial.println(" retry in 5s");
      LedRed();
      delay(5000);
      LedBlue();
    }
  }
}

