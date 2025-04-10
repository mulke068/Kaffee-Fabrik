#ifdef ESP_WIFI_MQTT

#include <WiFi.h>
#include <PubSubClient.h>

WiFiClient espClient;
PubSubClient mqttClient(espClient);

#endif