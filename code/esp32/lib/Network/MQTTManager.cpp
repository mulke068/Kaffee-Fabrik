
// void setupMQTT()
// {
//   mqttClient.setServer(mqttServer.c_str(), mqttPort);
//   mqttClient.setCallback(mqttCallback);
//   reconnectMQTT();
// }

// void reconnectMQTT()
// {
//   // Attempt to connect only if WiFi is connected
//   if (WiFi.status() != WL_CONNECTED)
//     return;

//   if (!mqttClient.connected())
//   {
//     Serial.print("Connecting to MQTT broker...");

//     bool connected = false;
//     if (mqttUser.length() > 0)
//     {
//       connected = mqttClient.connect(mqttClientId.c_str(), mqttUser.c_str(), mqttPassword.c_str());
//     }
//     else
//     {
//       connected = mqttClient.connect(mqttClientId.c_str());
//     }

//     if (connected)
//     {
//       Serial.println("connected");

//       // Subscribe to command topic
//       String commandTopic = mqttBaseTopic + "command";
//       mqttClient.subscribe(commandTopic.c_str());

//       // Publish online status
//       String statusTopic = mqttBaseTopic + "status";
//       mqttClient.publish(statusTopic.c_str(), "online", true);
//     }
//     else
//     {
//       Serial.print("failed, rc=");
//       Serial.println(mqttClient.state());
//     }
//   }
// }

// void mqttCallback(char *topic, byte *payload, unsigned int length)
// {
//   String topicStr = String(topic);
//   String message = "";

//   for (unsigned int i = 0; i < length; i++)
//   {
//     message += (char)payload[i];
//   }

//   Serial.println("MQTT message received [" + topicStr + "]: " + message);

//   // Process the received command
//   if (topicStr == mqttBaseTopic + "command")
//   {
//     processCommand(message);
//   }
// }

// void publishStatus()
// {
//   if (!mqttClient.connected() || WiFi.status() != WL_CONNECTED)
//     return;

//   // Publish all readings as JSON
//   String powerTopic = mqttBaseTopic + "power";
//   String tempTopic = mqttBaseTopic + "temperature";
//   String motorsTopic = mqttBaseTopic + "motors";

//   // Power readings
//   float shuntvoltage = ina219.getShuntVoltage_mV();
//   float busvoltage = ina219.getBusVoltage_V();
//   float current_mA = ina219.getCurrent_mA();
//   float power_mW = ina219.getPower_mW();
//   float loadvoltage = busvoltage + (shuntvoltage / 1000);

//   String powerJson = "{\"busvoltage\":" + String(busvoltage) +
//                      ",\"shuntvoltage\":" + String(shuntvoltage) +
//                      ",\"loadvoltage\":" + String(loadvoltage) +
//                      ",\"current_mA\":" + String(current_mA) +
//                      ",\"power_mW\":" + String(power_mW) + "}";

//   mqttClient.publish(powerTopic.c_str(), powerJson.c_str());

//   // Temperature readings
//   float motorTemp = readTMP102Temperature(TMP102_IC20_ADDR);
//   float powerTemp = readTMP102Temperature(TMP102_IC8_ADDR);
//   bool motorAlert = !digitalRead(TMP102_IC20_ALERT);
//   bool powerAlert = !digitalRead(TMP102_IC8_ALERT);

//   String tempJson = "{\"motorDriver\":{\"temp\":" + String(motorTemp) +
//                     ",\"alert\":" + String(motorAlert ? "true" : "false") +
//                     "},\"powerUnit\":{\"temp\":" + String(powerTemp) +
//                     ",\"alert\":" + String(powerAlert ? "true" : "false") + "}}";

//   mqttClient.publish(tempTopic.c_str(), tempJson.c_str());

//   // Motor status
//   String motorsJson = "{\"m1\":{\"direction\":\"" + getMotorDirectionString(motor1) +
//                       "\",\"speed\":" + String(motor1.getSpeed()) +
//                       "},\"m2\":{\"direction\":\"" + getMotorDirectionString(motor2) +
//                       "\",\"speed\":" + String(motor2.getSpeed()) +
//                       "},\"m3\":{\"direction\":\"" + getMotorDirectionString(motor3) +
//                       "\",\"speed\":" + String(motor3.getSpeed()) +
//                       "},\"m4\":{\"direction\":\"" + getMotorDirectionString(motor4) +
//                       "\",\"speed\":" + String(motor4.getSpeed()) + "}}";

//   mqttClient.publish(motorsTopic.c_str(), motorsJson.c_str());
// }
