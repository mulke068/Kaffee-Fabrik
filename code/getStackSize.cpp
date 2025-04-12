SET_LOOP_TASK_STACK_SIZE(16 * 1024);  // 16KB
uint32_t chipId = 0;

void setup() {
  Serial.begin(115200);

  Serial.println("Arduino Stack was set to %d bytes: "+ String(getArduinoLoopTaskStackSize()));

  // Print unused stack for the task that is running setup()
  Serial.println("\nSetup() - Free Stack Space: " + String(uxTaskGetStackHighWaterMark(NULL)));

  //  delay(1000);
  // if (esp_spiram_is_initialized())
  // {
  //   Serial.println("spiram is initialized");
  // }
  // else
  // {
  //   Serial.println("spiram failed to initialise");
  // } 
  //    delay(1000);
  // if (esp_psram_is_initialized())
  // {
  //   Serial.println("psram is initialized");
  // }
  // else
  // {
  //   Serial.println("psram failed to initialise");
  // } 

  Serial.println("Total heap: %d" + String(ESP.getHeapSize()));
  Serial.println("Free heap: %d" + String(ESP.getFreeHeap()));
  Serial.println("Total PSRAM: %d" + String(ESP.getPsramSize()));
  Serial.println("Free PSRAM: %d" + String(ESP.getFreePsram()));
}

void loop() {
  delay(1000);

  // Print unused stack for the task that is running loop() - the same as for setup()
  Serial.printf("\nLoop() - Free Stack Space: %d", uxTaskGetStackHighWaterMark(NULL));

    for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }

  Serial.printf("\nESP32 Chip model = %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("This chip has %d cores\n", ESP.getChipCores());
  Serial.print("Chip ID: ");
  Serial.println(chipId);

  delay(3000);
}