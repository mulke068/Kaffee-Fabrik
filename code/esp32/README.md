into the lib folder for the arduino library to work
```bash
git clone --recursive https://github.com/espressif/arduino-esp32.git arduino
```

Aktivate SPIRAM
```bash
pio run -t menuconfig
```
Component config > ESP32-S3-specific > Support for external, SPI-connected RAM


rtos ram binding
```c++
void TaskManager::createCommandTask()
{
  xTaskCreatePinnedToCore(
      commandTaskFunction,
      "CommandTask",
      4096, // Stack size
      this,
      1,
      &_commandTaskHandle,
      APP_CPU_NUM); // Use APP_CPU_NUM for better performance

  // Optionally, use SPIRAM for the task stack
  if (esp_spiram_is_initialized())
  {
    vTaskAllocateStack(&_commandTaskHandle, SPIRAM_STACK);
  }
}
```