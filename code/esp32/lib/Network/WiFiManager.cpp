
// #ifdef ESP_WIFI_MQTT

// void setupWiFi()
// {
//   Serial.print("Connecting to WiFi");
//   WiFi.mode(WIFI_STA);

//   if (useStaticIP && staticIP != IPAddress(0, 0, 0, 0))
//   {
//     if (!WiFi.config(staticIP, gateway, subnet, dns1, dns2))
//     {
//       Serial.println("Static IP configuration failed");
//     }
//   }

//   WiFi.begin(ssid.c_str(), password.c_str());

//   int attempts = 0;
//   while (WiFi.status() != WL_CONNECTED && attempts < 20)
//   {
//     delay(500);
//     Serial.print(".");
//     attempts++;
//   }

//   if (WiFi.status() == WL_CONNECTED)
//   {
//     Serial.println("\nWiFi connected");
//     Serial.print("IP address: ");
//     Serial.println(WiFi.localIP());
//   }
//   else
//   {
//     Serial.println("\nFailed to connect to WiFi");
//   }
// }

// float readTMP102Temperature(uint8_t address)
// {
//   Wire.beginTransmission(address);
//   Wire.write(0x00);
//   Wire.endTransmission();
//   Wire.requestFrom(static_cast<uint8_t>(address), static_cast<uint8_t>(2));

//   if (Wire.available() == 2)
//   {
//     byte msb = Wire.read();
//     byte lsb = Wire.read();
//     int temperature = ((msb << 8) | lsb) >> 4;
//     return temperature * 0.0625;
//   }
//   return 0.0;
// }

// String getMotorDirectionString(Motor &motor)
// {
//   if (motor.getDirection() == FORWARD)
//     return "FORWARD";
//   else if (motor.getDirection() == BACKWARD)
//     return "BACKWARD";
//   return "STOPPED";
// }

// void printNetworkSettings()
// {
//   Serial.println("\n===== Network Settings =====");
//   Serial.println("WiFi:");
//   Serial.println("  SSID: " + ssid);
//   Serial.println("  Static IP: " + String(useStaticIP ? "Enabled" : "Disabled"));
//   if (useStaticIP)
//   {
//     Serial.println("    IP: " + staticIP.toString());
//     Serial.println("    Gateway: " + gateway.toString());
//     Serial.println("    Subnet: " + subnet.toString());
//   }

//   Serial.println("\nMQTT:");
//   Serial.println("  Server: " + mqttServer);
//   Serial.println("  Port: " + String(mqttPort));
//   Serial.println("  Client ID: " + mqttClientId);
//   Serial.println("  User: " + mqttUser);
//   Serial.println("  Base Topic: " + mqttBaseTopic);
// }
// #endif
