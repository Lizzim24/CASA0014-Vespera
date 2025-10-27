
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
/* // Handle power on/off
  if (t == TOPIC_POWER) {
    if (msg.equalsIgnoreCase("on"))  powerOn = true;
    else if (msg.equalsIgnoreCase("off")) powerOn = false;
    // reflect state to strip & onboard
    int brightness = (powerOn && lastBrightness>=0) ? lastBrightness : 0;
    applyColorToPayload(COLORS[colorIndex], (uint8_t)brightness, powerOn);
    publishStrip();
    setOnboardLED(COLORS[colorIndex], (uint8_t)brightness, powerOn);
    else { Serial.println(String("[cmd] power => ") + (powerOn ? "on" : "off"));}
    publish(topic_power, powerOn ? "on":"off", true);
    return;
  } */
    // Handle color: expect "#RRGGBB"
    //else if (t == TOPIC_POWER) {
      //if (msg.length() == 7 && msg[0] == '#') {
        //long r = strtol(msg.substring(1,3).c_str(), NULL, 16);
        //long g = strtol(msg.substring(3,5).c_str(), NULL, 16);
        //long b = strtol(msg.substring(5,7).c_str(), NULL, 16);
        // find nearest palette index (optional); here we just set a custom color in slot 0
 /*        COLORS[colorIndex] = { (uint8_t)r, (uint8_t)g, (uint8_t)b };
        int brightness = (powerOn && lastBrightness>=0) ? lastBrightness : 0;
        applyColorToPayload(COLORS[colorIndex], (uint8_t)brightness, powerOn);
        publishStrip();
        setOnboardLED(COLORS[colorIndex], (uint8_t)brightness, powerOn);
        Serial.println(String("[cmd] color => ") + msg);
      } */
  // Power control
    // /cmd/power : "on"/"off"
  if (t == TOPIC_POWER_CMD) {
    unsigned long now = millis();
    if (now - lastPowerChange < POWER_COOLDOWN_MS) return;

    bool prev = powerOn;
    if      (msg.equalsIgnoreCase("on"))  powerOn = true;
    else if (msg.equalsIgnoreCase("off")) powerOn = false;
    else return;

    if (powerOn != prev) {
      lastPowerChange = now;
      publish(TOPIC_POWER, powerOn ? "on" : "off", true);
      int b = (powerOn && lastBrightness>=0) ? lastBrightness : 0;
      renderFrame(mode, b, now);
      if (mqttClient.connected()) mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size);
    }
    return;
  }

  /* // Mode control
  if (t == topic_mode) {
    msg.toLowerCase();
    if      (msg=="cct")     mode = M_CCT;
    else if (msg=="rainbow") mode = M_RAINBOW;
    else if (msg=="comet")   mode = M_COMET;
    else if (msg=="breath")  mode = M_BREATH;
    else if (msg=="twinkle") mode = M_TWINKLE;
    else { Serial.println("[cmd] mode: unknown"); return; }

    /* publish(topic_speed, String( (mode==M_RAINBOW)? speedRainbow :
                                 (mode==M_COMET)  ? speedComet   :
                                 (mode==M_BREATH) ? speedBreath  :
                                 (mode==M_TWINKLE)? twinkleProb  : 0.0f ),
            false); 
    int b = (powerOn && lastBrightness >= 0) ? lastBrightness : 0;
    renderAndPublish(b);
    return;
  } */

  // Speed control (applies to current mode)
  /* if (t == topic_speed) {
    float s = msg.toFloat();
    switch (mode) {
      case M_RAINBOW:
        if (s < 0.0f) s = 0.0f; if (s > 5.0f) s = 5.0f;
        speedRainbow = s; publish(topic_speed, String(speedRainbow), false);
        Serial.print("[cmd] rainbow speed = "); Serial.println(speedRainbow);
        break;
      case M_COMET:
        if (s < 1.0f) s = 1.0f; if (s > 200.0f) s = 200.0f;
        speedComet = s; publish(topic_speed, String(speedComet), false);
        Serial.print("[cmd] comet speed = "); Serial.println(speedComet);
        break;
      case M_BREATH:
        if (s < 0.05f) s = 0.05f; if (s > 5.0f) s = 5.0f;
        speedBreath = s; publish(topic_speed, String(speedBreath), false);
        Serial.print("[cmd] breath speed = "); Serial.println(speedBreath);
        break;
      case M_TWINKLE:
        if (s < 0.0f) s = 0.0f; if (s > 0.5f) s = 0.5f;
        twinkleProb = s; publish(topic_speed, String(twinkleProb), false);
        Serial.print("[cmd] twinkle prob = "); Serial.println(twinkleProb);
        break;
      case M_CCT:
        if (s < 0.0f) s = 0.0f; if (s > 2.0f) s = 2.0f;
        // reuse one var to carry gentle drift; optional
        speedRainbow = s; publish(topic_speed, String(speedRainbow), false);
        Serial.print("[cmd] cct drift = "); Serial.println(speedRainbow);
        break;
    }
    int b = (powerOn && lastBrightness >= 0) ? lastBrightness : 0;
    renderAndPublish(b);
    return;
  } */

  if (t == TOPIC_MODE_CMD) {
  Mode newMode;
  if (!strToMode(msg, newMode)) {
    Serial.println("[cmd] mode: unknown value");
    return;
  }
  if (newMode != mode) {
    mode = newMode;
    publish(topic_mode, modeToStr(mode), true);  // update retained state

    // re-render once so remote sees immediate change
    int b = (powerOn && lastBrightness>=0) ? lastBrightness : 0;
    unsigned long now = millis();
    renderFrame(mode, b, now);
    if (mqttClient.connected())
      mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size);
  }
  return;
  }

  Serial.println("[cmd] topic ignored.");
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
      // mqttClient.subscribe(TOPIC_POWER.c_str());
      // mqttClient.subscribe(TOPIC_COLOR.c_str());
      
      mqttClient.subscribe(TOPIC_POWER_CMD.c_str());
      mqttClient.subscribe(TOPIC_MODE_CMD.c_str());
      // mqttClient.subscribe(topic_speed.c_str());
      // mqttClient.subscribe(topic_raw.c_str()); // optional: not needed usually
      // Serial.println("Subscribed to topics: /power, /color");

      // Announce current state (retained)
      publish(TOPIC_POWER, powerOn ? "on" : "off", true);
      publish(topic_mode, modeToStr(mode), true);
      if (lastBrightness >= 0) publish(TOPIC_BRIGHTNESS, String(lastBrightness), false);
      //char hexBuf[10];
      //snprintf(hexBuf, sizeof(hexBuf), "#%02X%02X%02X",
               //COLORS[colorIndex].r, COLORS[colorIndex].g, COLORS[colorIndex].b);
      //publish(TOPIC_COLOR, String(hexBuf), true);
      //if (lastBrightness >= 0) publish(TOPIC_BRIGHTNESS, String(lastBrightness), false);
      // publish(topic_mode, (mode==M_CCT?"cct":mode==M_RAINBOW?"rainbow":mode==M_COMET?"comet":mode==M_BREATH?"breath":"twinkle"), true);
/*       publish(topic_speed, String( (mode==M_RAINBOW)? speedRainbow :
                                   (mode==M_COMET)  ? speedComet   :
                                   (mode==M_BREATH) ? speedBreath  :
                                   (mode==M_TWINKLE)? twinkleProb  : 0.0f ),
              false); */
      LedGreen();
      } else {
        Serial.print("Failed, rc="); Serial.println(mqttClient.state());
        LedRed(); delay(2000); LedBlue();
    }
  }
}

