#define Bot_ID 0xDD
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

void setup() {
  // Setup Read Spead and Serial monitor's Baud Rate
  Serial.begin(115200);

  // Macro Setups
  PS4_setup();
  Movement_setup();

  motor(motor_Speed);  // set all wheels to stop
  while (!PS4.isConnected()) {
    motor(motor_Speed);
  }


  UWB_setup(DWM3000_RST, DWM3000_IRQ, DWM3000_SS);
}

unsigned long previousMillis = 0;
const unsigned long interval = 100;  // milliseconds

Kalman filter_A(0.02, 0.02, 0.1);  // test val
Kalman filter_B(0.02, 0.02, 0.1);  // test val
Kalman filter_C(0.02, 0.02, 0.1);  // test val

int8_t positions[3] = { 0, 0, 0 };
int8_t velocities[3] = { 0, 0, 0 };
// int dw_pos[3] = { 0, 0, 0 };
// int dw_velocity[3] = { 0, 0, 0 };

int8_t dw_pos_a[3] = { 0, 0, 0 };
int8_t dw_velocity_a[3] = { 0, 0, 0 };
int8_t dw_pos_b[3] = { 0, 0, 0 };
int8_t dw_velocity_b[3] = { 0, 0, 0 };
int8_t dw_pos_c[3] = { 0, 0, 0 };
int8_t dw_velocity_c[3] = { 0, 0, 0 };


void loop() {
  notify();
  PS4_move(stick_LX, stick_LY, stick_RX, stick_RY);

  // Anchor_waiting_for_response(Bot_ID, positions, velocities);

  unsigned long currentMillis = millis();
  static int swap = 0;
  static double dist_2_A;
  static double dist_2_B;
  static double dist_2_C;
  static double distance_received;

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (swap % 3 == 0) {
      distance_received = get_UWB_Distance(Bot_ID, 0xAA, positions, velocities);  //takes around 2ms to obtain information
      Serial.print("DIST A: ");
      Serial.print(distance_received);

      if (distance_received != -1 && distance_received > 0) {
        dist_2_A = filter_A.updateFilter(distance_received);
        Serial.print("\tDIST A filtered: ");
        Serial.print(dist_2_A);
        DW3000_get_robot_info(dw_pos_a, dw_velocity_a);
      }

    } else if (swap % 3 == 1) {
      distance_received = get_UWB_Distance(Bot_ID, 0xBB, positions, velocities);
      Serial.print("DIST B: ");
      Serial.print(dist_2_B);

      if (distance_received != -1 && distance_received > 0) {
        dist_2_B = filter_B.updateFilter(distance_received);
        Serial.print("\tDIST B filtered: ");
        Serial.print(dist_2_B);
        DW3000_get_robot_info(dw_pos_b, dw_velocity_b);
      }

    } else {
      distance_received = get_UWB_Distance(Bot_ID, 0xCC, positions, velocities);
      Serial.print("DIST C: ");
      Serial.print(distance_received);

      if (distance_received != -1 && distance_received > 0) {
        dist_2_C = filter_C.updateFilter(distance_received);
        Serial.print("\tDIST C filtered: ");
        Serial.print(dist_2_C);
        DW3000_get_robot_info(dw_pos_c, dw_velocity_c);
      }
    }
    swap++;

    Robot robots[3] = {
      { .bot_id = 0xAA, .x = dw_pos_a[0], .y = dw_pos_a[1], .theta = dw_pos_a[2], .last_sent_time = 0, .distances = dist_2_A },
      { .bot_id = 0xBB, .x = dw_pos_b[0], .y = dw_pos_b[1], .theta = dw_pos_b[2], .last_sent_time = 0, .distances = dist_2_B },
      // { .bot_id = 0xCC, .x = dw_pos_c[0], .y = dw_pos_c[1], .theta = dw_pos_c[2], .distances = dist_2_C },
      { .bot_id = 0xDD, .x = 0, .y = 0.8, .theta = 0, .last_sent_time = 0, .distances = 0 }
    };



    double x, y;
    int status = Trilateration_2D(robots, 3, Bot_ID, &x, &y);
    if (status == 0) {
      printf("  Estimated position: (%.2f, %.2f)\n", x, y);
      x_current = x;
      y_current = y;
    } else {
      printf("  Trilateration failed. Error code: %d\n", status);
    }

    Serial.println();
  }
}
