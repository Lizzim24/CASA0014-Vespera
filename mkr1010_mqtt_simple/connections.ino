
// Function to handle incoming MQTT messages
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);

  // Copy payload into a temporary String (payload is not null-terminated)
  String msg;
  msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();  // remove whitespace/newlines

  String t = String(topic);

  // /cmd/power : "on"/"off"
  if (t == TOPIC_POWER_CMD) {
    unsigned long now = millis();
    // Cooldown to prevent rapid toggles/flapping
    if (now - lastPowerChange < POWER_COOLDOWN_MS) return;

    bool prev = powerOn;  // remember previous power state

    // Parse message to set desired power state
    if (msg.equalsIgnoreCase("on")) powerOn = true;
    else if (msg.equalsIgnoreCase("off")) powerOn = false;
    else return;  // any other payload ignored

    // Apply only if the state actually changed
    if (powerOn != prev) {
      lastPowerChange = now;

      // Publish device state (retained) for observers (we DO NOT subscribe to /power)
      publish(TOPIC_POWER, powerOn ? "on" : "off", true);

      // Re-render immediately to reflect new power state
      int b = (powerOn && lastBrightness >= 0) ? lastBrightness : 0;
      renderFrame(mode, b, now);

      // Push the new raw payload to the strip topic if connected
      if (mqttClient.connected()) mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size);
    }
    return;  // handled
  }

  // /cmd/mode : expect "cct" / "rainbow" / "comet" / "twinkle" / "breath"
  if (t == TOPIC_MODE_CMD) {
    Mode newMode;
    if (!strToMode(msg, newMode)) {   // parse text into enum
      Serial.println("[cmd] mode: unknown value");
      return;
    }
    if (newMode != mode) {
      mode = newMode;  // set new mode

      // Publish readable state (retained). We don't subscribe to /mode, so no echo loop.
      publish(topic_mode, modeToStr(mode), true); 

      // re-render once so remote sees immediate change
      int b = (powerOn && lastBrightness >= 0) ? lastBrightness : 0;
      unsigned long now = millis();
      renderFrame(mode, b, now);

      if (mqttClient.connected())
        mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size);
    }
    return;
  }

  // Any other topic is ignored (we don't accept direct brightness commands here)
  Serial.println("[cmd] topic ignored.");
}


// Connect wifi and get mac address for unique clientId and print out some setup info
void startWifi() {
  delay(10);
  LedBlue();  // show Blue LED when looking for wifi
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  // Check if the WiFi module is present
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // Don't continue if module is not present
    while (true)
      ;
  }

  // Try primary then fallback
  int status = WL_IDLE_STATUS;
  unsigned long t0 = millis();
  while (status != WL_CONNECTED && millis() - t0 < 15000) {
    status = WiFi.begin(ssid, password);
    Serial.print(".");
    delay(500);
  }

  // If primary failed and a fallback exists, try fallback for another 15s
  if (status != WL_CONNECTED && ssid1 && strlen(ssid1) > 0) {
    Serial.println("\nPrimary failed. Trying fallback WiFi...");
    t0 = millis();
    while (status != WL_CONNECTED && millis() - t0 < 15000) {
      status = WiFi.begin(ssid1, password1);
      Serial.print(".");
      delay(500);
    }
  }

  // Report result + set onboard LED color
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

// Utility: print MAC as hex with colons
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
  LedBlue(); // show searching/connecting

  // Build a unique clientId from MAC once
  if (clientId.length() == 0) {
    byte mac[6];
    WiFi.macAddress(mac);
    char macStr[18];
    sprintf(macStr, "%02X%02X%02X%02X%02X%02X",
            mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
    clientId = String("vespera-") + macStr;
    Serial.print("ClientId: ");
    Serial.println(clientId);
    Serial.print("MAC address: ");
    printMacAddress(mac);
  }

  // Keep trying until connected
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT...");
    if (mqttClient.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT");

      // Subscribe only to *command* topics (we DO NOT subscribe to state topics)
      mqttClient.subscribe(TOPIC_POWER_CMD.c_str());
      mqttClient.subscribe(TOPIC_MODE_CMD.c_str());

      // Announce current state (retained)
      publish(TOPIC_POWER, powerOn ? "on" : "off", true);
      publish(topic_mode, modeToStr(mode), true);
      if (lastBrightness >= 0) publish(TOPIC_BRIGHTNESS, String(lastBrightness), false);

      LedGreen();  // good to go
    } else {

      // If failed, show error and retry after a short delay
      Serial.print("Failed, rc=");
      Serial.println(mqttClient.state());
      LedRed();
      delay(2000);
      LedBlue();
    }
  }
}
