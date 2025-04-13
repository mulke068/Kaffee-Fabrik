
#ifndef Config_h
#define Config_h

#include <Preferences.h> // Include Preferences.h here
// #include "Preference.h"
#include "GlobalConfig.h"

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

class Config
{
public:
    Config();
    ~Config();
    void begin();
    void loadSettings();
    void saveSettings();

    // Sensor Auto Read
    bool sensorConsolePrintingEnabled = false;
    unsigned long sensorConsolePrintingInterval = 5000; // Default 5 seconds
    bool sensorUpdateEnabled = true;
    unsigned long sensorUpdateInterval = 5000; // Default 5 seconds

#ifdef ESP_STOP_SWITCH
    // Motor Stop End-Switches
    uint8_t stopPin1 = 45; // ESP PIN 26
    uint8_t stopPin2 = 46; // ESP PIN 16
#endif

#ifdef ESP_WIFI_MQTT
    // WiFi settings
    char *ssid = "FRITZ!Box 7490";

    char *password = TOSTRING(WIFI_PASSWORD);

    // Set to true to use static IP, false for DHCP
    bool useStaticIP = false; // TODO add DHCP choise

    // WiFi settings for static IP
    IPAddress staticIP;
    IPAddress gateway;
    IPAddress subnet;
    IPAddress dns1;
    IPAddress dns2;

    // MQTT settings
    char *mqttServer = "192.168.0.100";
    int mqttPort = 1883;

    // Set to true to use authentication, false otherwise
    bool useMQTTAuth = false; // TODO add MQTT Auth choise
    char *mqttUser = "";
    char *mqttPassword = "";

    // MQTT Standard Settings
    char *mqttClientId = "ESP32Controller";
    char *mqttBaseTopic = "controller/";
#endif

private:
    Preferences preferences;
};

#endif
