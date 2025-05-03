uint8_t Bot_ID = 0xDD;
// #define AntennaDelay 16370
#define DEBUG_OUTPUT true

#define DWM3000_RST 15
#define DWM3000_IRQ 13
#define DWM3000_SS 5


// Functional Macros
#include "PS4.h"
#include "Movement.h"
#include "UWB.h"
#include "kalmanFilter.h"
#include "multilateration.h"

DWM3000 dwm3000(DWM3000_RST, DWM3000_IRQ, DWM3000_SS, Bot_ID, false);
RobotSwarm swarm(Bot_ID);

int8_t positions[3] = { 0, 0, 0 };
int8_t velocities[3] = { 0, 0, 0 };

void setup() {
  Serial.begin(115200);

  // PS4_setup();
  Movement_setup();

  // while (!PS4.isConnected()) {
  //   motor(motor_Speed);
  // }


  // Create the task on Core 0 (ESP32 has core 0 and core 1)
  xTaskCreatePinnedToCore(
    DW3000_Module,    // Function to run
    "DW3000_Module",  // Name of task
    4096,             // Stack size (words, not bytes)
    &Bot_ID,          // Parameter to pass
    1,                // Priority (1 is usually fine)
    NULL,             // Task handle (not used here)
    0                 // Core to pin to (0 = PRO core)
  );

  // Add base robot, with starting positions and distance must always be 0
  swarm.update_robot(Bot_ID, 0, 0, 0);  // set base address at origin 

  // This will automatically create a new robot if it doesn't exist, or update it if it has existed
  swarm.update_robot(0xAA, 2, 2, 2.82);
  swarm.update_robot(0xBB, 2, 0, 2);
  swarm.update_robot(0xCC, 0, 2, 2);
}

void DW3000_Module(void* parameter) {
  uint16_t sender_id = *(uint16_t*)parameter;

  dwm3000.begin();

  while (true) {
    static uint32_t last_switch = 0;
    uint32_t now = millis();
    // Measure distances for all robots every 100ms
    if (now - last_switch >= 100) {
      last_switch = now;

      //////////////////////////////////////////////////////////////
      // First remove any stale robots
      swarm.remove_stale_robots();

      // Get all robot addresses in the swarm
      std::vector<int16_t> addresses = swarm.get_all_addresses();

      // Iterate through all robots
      for (int16_t address : addresses) {
        if (address == sender_id) continue;  // Skip our own address

        // Get distance measurement (takes about 5ms per measurement)
        double distance_received = dwm3000.getDistance(address, positions, velocities);

        if (distance_received != -1 && distance_received > 0) {
          // Get current position info
          int8_t dw_pos[3], dw_velocity[3];
          dwm3000.getRobotInfo(dw_pos, dw_velocity);  // update rest of DWM3000 received data

          swarm.update_robot(address, dw_pos[0], dw_pos[1], distance_received);  // Update swarm structure with new measurement

          ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

          if (DEBUG_OUTPUT) {
            Serial.print("Updated robot 0x");
            Serial.print(address, HEX);
            Serial.print(" - Distance: ");
            Serial.print(distance_received);
            Serial.print("m, Position: (");
            Serial.print(dw_pos[0]);
            Serial.print(", ");
            Serial.print(dw_pos[1]);
            Serial.println(")");
          }
        }
      }
      if (DEBUG_OUTPUT) Serial.println("Completed full swarm measurement cycle");

    } else {
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // default back to Anchor mode, listen to any tags and also identify any new addresses that just added into the system
      // - Listen to tags, and if address matches base address, find the distance to it
      // - If address is not base address but another known address, update last addressed time to indicate robot is active in the swarm
      // - If address is unknown, add that as a new address in the system
      dwm3000.processIncoming(positions, velocities);
    }
    vTaskDelay(1);  // short delay to yield CPU
  }
}



void loop() {
  // notify();
  // PS4_move(stick_LX, stick_LY, stick_RX, stick_RY);

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Update base position based on all info
  swarm.update_base_position(Bot_ID);

  // Print all robot information
  swarm.print_Robot_Swarm();
  vTaskDelay(100);  // Suspend main loop
}
