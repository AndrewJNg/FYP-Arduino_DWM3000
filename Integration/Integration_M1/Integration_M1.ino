uint8_t Bot_ID = 0xDD;  // must be global/static to safely pass pointer
// #define AntennaDelay 16370

#define DWM3000_RST 15
#define DWM3000_IRQ 13
#define DWM3000_SS 5

double x_current;
double y_current;

// Functional Macros
#include "PS4.h"
#include "Movement.h"
#include "UWB.h"
#include "kalmanFilter.h"
#include "multilateration.h"


int8_t positions[3] = { 0, 0, 0 };
int8_t velocities[3] = { 0, 0, 0 };



int8_t dw_pos_a[3] = { 0, 0, 0 };
int8_t dw_velocity_a[3] = { 0, 0, 0 };
static double dist_2_A;
Kalman filter_A(0.02, 0.02, 0.1);

int8_t dw_pos_b[3] = { 0, 0, 0 };
int8_t dw_velocity_b[3] = { 0, 0, 0 };
static double dist_2_B;
Kalman filter_B(0.02, 0.02, 0.1);

int8_t dw_pos_c[3] = { 0, 0, 0 };
int8_t dw_velocity_c[3] = { 0, 0, 0 };
static double dist_2_C;
Kalman filter_C(0.02, 0.02, 0.1);

void setup() {
  Serial.begin(115200);

  // PS4_setup();
  Movement_setup();

  motor(motor_Speed);  // set all wheels to stop
  // while (!PS4.isConnected()) {
  //   motor(motor_Speed);
  // }


  UWB_setup(DWM3000_RST, DWM3000_IRQ, DWM3000_SS);

  // Create the task on Core 0 (ESP32 has core 0 and core 1)
  // xTaskCreatePinnedToCore(
  //   AnchorTask,    // Function to run
  //   "AnchorTask",  // Name of task
  //   4096 * 2,      // Stack size (words, not bytes)
  //   &Bot_ID,       // Parameter to pass
  //   1,             // Priority (1 is usually fine)
  //   NULL,          // Task handle (not used here)
  //   0              // Core to pin to (0 = PRO core)
  // );
}

void loop() {
  // notify();
  // PS4_move(stick_LX, stick_LY, stick_RX, stick_RY);

  static uint32_t last_switch = 0;
  uint32_t now = millis();

  // Tag mode for 100ms every second (adjust as needed)
  if (now - last_switch >= 50) {
    last_switch = now;
    if (anchor) switchToTagMode();  // Only switch if not already in tag mode



    // Perform tag operations
    static int swap = 0;
    double distance_received = -1;
    if (swap % 3 == 0) {
      distance_received = get_UWB_Distance(Bot_ID, 0xAA, positions, velocities);  //takes around 5ms to obtain information for TWR

      Serial.print("DIST A: ");
      Serial.print(distance_received);

      if (distance_received != -1 && distance_received > 0) {
        dist_2_A = filter_A.updateFilter(distance_received);
        Serial.print("\tDIST A filtered: ");
        Serial.print(dist_2_A);
        DW3000_get_robot_info(dw_pos_a, dw_velocity_a);
      }

    } else if (swap % 3 == 1) {
      distance_received = get_UWB_Distance(Bot_ID, 0xDD, positions, velocities);
      Serial.print("DIST D: ");
      Serial.print(dist_2_B);

      if (distance_received != -1 && distance_received > 0) {
        dist_2_B = filter_B.updateFilter(distance_received);
        Serial.print("\tDIST D filtered: ");
        Serial.print(dist_2_B);
        DW3000_get_robot_info(dw_pos_b, dw_velocity_b);
      }

    } else {
      distance_received = get_UWB_Distance(Bot_ID, 0xEE, positions, velocities);
      Serial.print("DIST E: ");
      Serial.print(distance_received);

      if (distance_received != -1 && distance_received > 0) {
        dist_2_C = filter_C.updateFilter(distance_received);
        Serial.print("\tDIST E filtered: ");
        Serial.print(dist_2_C);
        DW3000_get_robot_info(dw_pos_c, dw_velocity_c);
      }
    }
    Serial.println();

    swap++;

  } else {
    // Anchor mode the rest of the time
    if (anchor == 0) switchToAnchorMode();  // Only switch if not already in anchor mode

    // unsigned long pong = millis();
    Serial.print("Incoming : ");
    Serial.print(Anchor_waiting_for_response(Bot_ID, positions, velocities));
    Serial.print("\t Address: ");
    Serial.print(DW3000_get_robot_info(dw_pos_b, dw_velocity_b), HEX);
    Serial.println("");

    // Serial.print("time: ");
    // Serial.println(millis() - pong);
  }



  // Robot robots[3] = {
  //   { .bot_id = 0xAA, .x = dw_pos_a[0], .y = dw_pos_a[1], .theta = dw_pos_a[2], .last_sent_time = 0, .distances = dist_2_A },
  //   { .bot_id = 0xBB, .x = dw_pos_b[0], .y = dw_pos_b[1], .theta = dw_pos_b[2], .last_sent_time = 0, .distances = dist_2_B },
  //   // { .bot_id = 0xCC, .x = dw_pos_c[0], .y = dw_pos_c[1], .theta = dw_pos_c[2], .distances = dist_2_C },
  //   { .bot_id = 0xDD, .x = 0, .y = 0.8, .theta = 0, .last_sent_time = 0, .distances = 0 }
  // };



  // double x, y;
  // int status = Trilateration_2D(robots, 3, Bot_ID, &x, &y);
  // if (status == 0) {
  //   printf("  Estimated position: (%.2f, %.2f)\n", x, y);
  //   x_current = x;
  //   y_current = y;
  // } else {
  //   printf("  Trilateration failed. Error code: %d\n", status);
  // }

  // Serial.println();


  // vTaskDelay(1000);  // Suspend main loop
}
