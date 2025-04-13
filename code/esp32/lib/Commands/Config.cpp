
#include "Config.h"

Config::Config()
{
#ifdef ESP_WIFI_MQTT
  staticIP = IPAddress(0, 0, 0, 0);
  gateway = IPAddress(0, 0, 0, 0);
  subnet = IPAddress(255, 255, 255, 0);
  dns1 = IPAddress(8, 8, 8, 8);
  dns2 = IPAddress(8, 8, 4, 4);
#endif
}

Config::~Config()
{
  preferences.end();
}

void Config::begin()
{
}

void Config::loadSettings()
{
  preferences.begin("controller", true); // Read-only mode

  sensorConsolePrintingEnabled = preferences.getBool("scpe", sensorConsolePrintingEnabled);
  sensorConsolePrintingInterval = preferences.getInt("scpi", sensorConsolePrintingInterval);

#ifdef ESP_STOP_SWITCH
  stopPin1 = preferences.getInt("sp1", stopPin1);
  stopPin2 = preferences.getInt("sp2", stopPin2);
#endif
#ifdef ESP_WIFI_MQTT
  // WiFi settings
  ssid = preferences.getString("ssid", ssid);
  password = preferences.getString("pwd", password);
  useStaticIP = preferences.getBool("usip", useStaticIP);

  // Read IP settings if static IP is enabled
  if (useStaticIP)
  {
    staticIP.fromString(preferences.getString("sip", "0.0.0.0"));
    gateway.fromString(preferences.getString("gtw", "0.0.0.0"));
    subnet.fromString(preferences.getString("sbn", "255.255.255.0"));
    dns1.fromString(preferences.getString("dns1", "8.8.8.8"));
    dns2.fromString(preferences.getString("dns2", "8.8.4.4"));
  }

  // MQTT settings
  mqttServer = preferences.getString("ms", mqttServer);
  mqttPort = preferences.getInt("mp", mqttPort);
  mqttUser = preferences.getString("musr", mqttUser);
  mqttPassword = preferences.getString("mpwd", mqttPassword);
  mqttClientId = preferences.getString("mcid", mqttClientId);
  mqttBaseTopic = preferences.getString("mbt", mqttBaseTopic);

#endif

  preferences.end();

  Serial.println("Settings loaded from preferences");
}

void Config::saveSettings()
{
  preferences.begin("controller", false); // Read-only mode

  preferences.putBool("scpe", sensorConsolePrintingEnabled);
  preferences.putInt("scpi", sensorConsolePrintingInterval);

#ifdef ESP_STOP_SWITCH
  preferences.putInt("sp1", stopPin1);
  preferences.putInt("sp2", stopPin2);
#endif
#ifdef ESP_WIFI_MQTT
  // WiFi settings
  preferences.putString("ssid", ssid);
  preferences.putString("pwd", password);
  preferences.putBool("usip", useStaticIP);

  if (useStaticIP)
  {
    preferences.putString("sip", staticIP.toString());
    preferences.putString("gtw", gateway.toString());
    preferences.putString("sbn", subnet.toString());
    preferences.putString("dns1", dns1.toString());
    preferences.putString("dns2", dns2.toString());
  }

  // MQTT settings
  preferences.putString("ms", mqttServer);
  preferences.putInt("mp", mqttPort);
  preferences.putString("musr", mqttUser);
  preferences.putString("mpwd", mqttPassword);
  preferences.putString("mcid", mqttClientId);
  preferences.putString("mbt", mqttBaseTopic);
#endif

  preferences.end();

  Serial.write("Settings saved to preferences\n");
}