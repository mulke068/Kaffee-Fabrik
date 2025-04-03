

#define SDA_PIN 17
#define SCL_PIN 18
#define PCA9555_ADDR 0x20
#define TMP102_IC20_ADDR 0x48
#define TMP102_IC20_ALERT 1
#define TMP102_IC8_ADDR 0x49
#define TMP102_IC8_ALERT 2

const uint8_t PWMA_1 = 4;
const uint8_t PWMB_1 = 5;
const uint8_t PWMA_2 = 6;
const uint8_t PWMB_2 = 7;

const uint8_t AIN1_1 = 16;
const uint8_t AIN2_1 = 15;
const uint8_t BIN1_1 = 8;
const uint8_t BIN2_1 = 9;

const uint8_t AIN1_2 = 11;
const uint8_t AIN2_2 = 10;
const uint8_t BIN1_2 = 13;
const uint8_t BIN2_2 = 14;

const uint8_t STBY = 19;
const uint8_t BUTTON_PIN = 0;
const uint8_t LED_17 = 48;
const uint8_t LED_18 = 47;

// Globals for sensors and LED state
Adafruit_INA219 ina219(0x40);
byte port0State = 0b00000000;
byte port1State = 0b00000000;
bool sensorReadingEnabled = false;
bool autoUpdateEnabled = true;
unsigned long sensorInterval = 5000; // Default 5 seconds

// LED PCA9555
byte led1 = 0b00000001;
byte led2 = 0b00000010;
byte led3 = 0b00000100;
byte led4 = 0b00001000;
byte led5 = 0b00010000;
byte led6 = 0b00100000;
byte led7 = 0b00100000; // Port 1 mapping (LEDs 7-12)
byte led8 = 0b00010000;
byte led9 = 0b00001000;
byte led10 = 0b00000100;
byte led11 = 0b00000010;
byte led12 = 0b00000001;

// WiFi and MQTT settings
String ssid = "FRITZ!Box 7490";
String password = "05767844606687810343";
String mqttServer = "192.168.0.100";
int mqttPort = 1883;
String mqttUser = "";
String mqttPassword = "";
String mqttClientId = "ESP32Controller";
String mqttBaseTopic = "controller/";
bool useStaticIP = false;
IPAddress staticIP(0, 0, 0, 0);
IPAddress gateway(0, 0, 0, 0);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(8, 8, 8, 8);
IPAddress dns2(8, 8, 4, 4);

// Motor Stop End-Tastern
uint8_t stopPin1 =  15;
uint8_t stopPin2 =  16;