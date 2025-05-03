#define Bot_ID 0xBB
#include "UWB.h"

int8_t positions[3] = {2,0,0};
int8_t velocities[3] = {0,0,0};

uint16_t bot_id = Bot_ID;  // must be global/static to safely pass pointer

// Default settings
#define DWM3000_RST 27
#define DWM3000_IRQ 34
#define DWM3000_SS 4

// #define DWM3000_RST 15
// #define DWM3000_IRQ 13
// #define DWM3000_SS 5

void setup() {
  UWB_setup(DWM3000_RST, DWM3000_IRQ, DWM3000_SS);

  // Create the task on Core 0 (ESP32 has core 0 and core 1)
  xTaskCreatePinnedToCore(
    AnchorTask,          // Function to run
    "AnchorTask",        // Name of task
    4096,                // Stack size (words, not bytes)
    &bot_id,             // Parameter to pass
    1,                   // Priority (1 is usually fine)
    NULL,                // Task handle (not used here)
    0                    // Core to pin to (0 = PRO core)
  );
}

void loop() {
  vTaskDelay(portMAX_DELAY); // Suspend main loop forever
}




void AnchorTask(void* parameter) {
  uint16_t sender_id = *(uint16_t*)parameter;

  while (true) {
    Anchor_waiting_for_response(Bot_ID, positions, velocities); 
    // vTaskDelay(1);  // short delay to yield CPU (optional)
  }
}