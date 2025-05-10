// uint8_t Bot_ID = 0xDD;
// uint8_t Bot_ID = 0xEE;
// uint8_t Bot_ID = 0xFF;
// uint8_t Bot_ID = 0xCC;
// uint8_t Bot_ID = 0xBB;
// uint8_t Bot_ID = 0xAA;
// #define AntennaDelay 16370
#define DEBUG_OUTPUT false
// #define fixed_base false
#define fixed_base true

// uint8_t Bot_ID = 0xAA;
// #define DWM3000_RST 27
// #define DWM3000_IRQ 34
// #define DWM3000_SS 4

// uint8_t Bot_ID = 0xBB;
// #define DWM3000_RST 27
// #define DWM3000_IRQ 34
// #define DWM3000_SS 4

uint8_t Bot_ID = 0xEE;
#define DWM3000_RST 15
#define DWM3000_IRQ 13
#define DWM3000_SS 5

#define xStart 1
#define yStart 1


// Functional Macros
#include "PS4.h"
// #include "Movement.h"
#include "UWB.h"
#include "kalmanFilter.h"
#include "multilateration.h"
#include "MPU6050.h"
#include "Motor_Subsystem.h"

//{MotorPin1, MotorPin2, EncAnalogPin, EncDir}
MotorControl frontLeftMotor(12, 14, 34, 0);
MotorControl frontRightMotor(4, 2, 39, 1);
MotorControl backLeftMotor(27, 26, 35, 0);
MotorControl backRightMotor(17, 16, 36, 1);

DWM3000 dwm3000(DWM3000_RST, DWM3000_IRQ, DWM3000_SS, Bot_ID, false);
RobotSwarm swarm(Bot_ID);

int8_t positions[3] = { xStart, yStart, 0 };
int8_t velocities[3] = { 0, 0, 0 };
double real_pos[3], real_velocity[3];

void setup() {
  Serial.begin(115200);

  // PS4_setup();
  frontLeftMotor.setupMotorSystem();
  frontRightMotor.setupMotorSystem();
  backLeftMotor.setupMotorSystem();
  backRightMotor.setupMotorSystem();
  // Movement_setup();
  Gyro_setup();

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
  swarm.update_robot(Bot_ID, xStart, yStart, 0);  // set base address at origin

  // This will automatically create a new robot if it doesn't exist, or update it if it has existed
  // swarm.update_robot(0xAA, 2, 2, 2.82);
  // swarm.update_robot(0xBB, 2, 0, 2);
  // swarm.update_robot(0xCC, 0, 2, 2);
}

void DW3000_Module(void* parameter) {
  uint16_t sender_id = *(uint16_t*)parameter;

  dwm3000.begin();

  while (true) {
    static uint32_t last_switch = 0;
    uint32_t now = millis();

    ////////////////////////////////////////////////////////////////////////////////
    if (now - last_switch >= random(150, 501)) {
      last_switch = now;

      //////////////////////////////////////////////////////////////
      // First remove any stale robots
      swarm.remove_stale_robots();

      // Get all robot addresses in the swarm
      std::vector<int16_t> addresses = swarm.get_all_addresses();

      if (addresses.size() == 1) {  //if it doesn't know any other robots, send a msg to itself, so that others can identify it is active
        dwm3000.getDistance(Bot_ID, positions, velocities);
        // Serial.println("send personal address");
      } else {

        // Iterate through all robots
        for (int16_t address : addresses) {
          if (address == sender_id) continue;  // Skip our own address
                                               // for (int i = 0; i < 5; i++) {
          // Get distance measurement (takes about 5ms per measurement)
          // Serial.print("Pinging: ");
          // Serial.println(address,HEX);
          double distance_received = dwm3000.getDistance(address, positions, velocities);

          if (distance_received != -1 && (distance_received > 0 && distance_received < 50)) {
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

          vTaskDelay(50);  // short delay to yield CPU
          // }
        }
      }
      if (fixed_base) {
        double fake_pos[3], dw_velocity[3];
        swarm.update_base_position(Bot_ID, fake_pos);  // do not update current position
      } else {
        swarm.update_base_position(Bot_ID, real_pos);  // update current positions
      }
      // positions[3]
      if (DEBUG_OUTPUT) Serial.println("Completed full swarm measurement cycle");

    } else {
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // default back to Anchor mode, listen to any tags and also identify any new addresses that just added into the system
      // - Listen to tags, and if address matches base address, find the distance to it
      // - If address is not base address but another known address, update last addressed time to indicate robot is active in the swarm
      // - If address is unknown, add that as a new address in the system
      double distance = dwm3000.processIncoming(positions, velocities);

      if (distance != -1) {
        // Serial.print("anchor dist found: ");
        // Serial.println(distance);
        int8_t dw_pos[3], dw_velocity[3];
        uint16_t address = dwm3000.getRobotInfo(dw_pos, dw_velocity);
        swarm.update_robot(address, dw_pos[0], dw_pos[1], (distance > 0 && distance < 50)? distance : -1);
        // Serial.print("New address found: ");
        // Serial.println(address, HEX);
      }
    }
    // vTaskDelay(1);  // short delay to yield CPU
  }
}



void loop() {
  Gyro_update();
  // notify();
  // PS4_move(stick_LX, stick_LY, stick_RX, stick_RY);

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Update base position based on all info

  // Print all robot information
  swarm.print_Robot_Swarm();

  // double target_x = 1;
  // double target_y = 1;
  // float angle = 0;

  // float LX_vector = map(100*(target_x - real_pos[0]), -128, 127, -10000, 10000) / 100;
  // float LY_vector = map(100*(target_y - real_pos[1]), -127, 128, -10000, 10000) / 100;
  // // float angle;
  // if (LY_vector == 0 && LX_vector > 0) angle = PI / 2;
  // else if (LY_vector == 0 && LX_vector < 0) angle = 3 * PI / 2;
  // else if (LY_vector == 0 && LX_vector == 0) angle = 0;
  // else angle = atan(abs(LX_vector) / abs(LY_vector));

  // if (LX_vector > 0 && LY_vector > 0) angle = angle;
  // else if (LX_vector > 0 && LY_vector < 0) angle = PI - angle;
  // else if (LX_vector < 0 && LY_vector < 0) angle = PI + angle;
  // else if (LX_vector < 0 && LY_vector > 0) angle = 2 * PI - angle;

  // //Speed  (range of 0 to 100)
  // float Speed_total_percent = PS4_LeftAnalogStickSpeed(500*(target_x - real_pos[0]), 500*(target_y - real_pos[1]));

  // int motor_Speeds[4];
  // motor_Speeds[0] = map(sin(angle + (1 * PI) / 4) * Speed_total_percent * 100, -10000, 10000, -PWM_resolution_max_value, PWM_resolution_max_value);
  // motor_Speeds[1] = map(sin(angle + (3 * PI) / 4) * Speed_total_percent * 100, -10000, 10000, -PWM_resolution_max_value, PWM_resolution_max_value);
  // motor_Speeds[2] = map(sin(angle + (3 * PI) / 4) * Speed_total_percent * 100, -10000, 10000, -PWM_resolution_max_value, PWM_resolution_max_value);
  // motor_Speeds[3] = map(sin(angle + (1 * PI) / 4) * Speed_total_percent * 100, -10000, 10000, -PWM_resolution_max_value, PWM_resolution_max_value);
  // // motor_Speeds[0] = 255;
  // // motor_Speeds[1] = 255;
  // // motor_Speeds[2] = 255;
  // // motor_Speeds[3] = 255;

  // Serial.print(real_pos[0]);
  // Serial.print("  ");
  // Serial.print(real_pos[1]);
  // Serial.print("  ");
  // Serial.print(angle);
  // Serial.print("  ");
  // Serial.print(Speed_total_percent);
  // Serial.print("  ");


  // Serial.print(motor_Speeds[0]);
  // Serial.print("  ");
  // Serial.print(motor_Speeds[1]);
  // Serial.print("  ");
  // Serial.print(motor_Speeds[2]);
  // Serial.print("  ");
  // Serial.print(motor_Speeds[3]);
  // Serial.println("  ");

  // Serial.print("Gyro z: ");
  // Serial.print(MPU_Z_angle());
  // Serial.println("");

  //apply turn Speed to allow control using right analog stick
  // motor_Speed[0] = motor_Speed[0] + turn_Speed;
  // motor_Speed[1] = motor_Speed[1] - turn_Speed;
  // motor_Speed[2] = motor_Speed[2] + turn_Speed;
  // motor_Speed[3] = motor_Speed[3] - turn_Speed;

  // run all motors with according speeds
  // if(fixed_base==false)  motor(motor_Speeds);
  // vTaskDelay(100);  // Suspend main loop
}
