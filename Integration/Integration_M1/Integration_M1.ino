#define Bot_ID 0xCC
#define rec_Bot_ID 0xAA

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
  // analogReadResolution(12);
  Serial.begin(115200);

  // Macro Setups
  PS4_setup();
  Movement_setup();

  motor(motor_Speed);
  while (!PS4.isConnected()) {
    motor(motor_Speed);
  }


  UWB_setup();
}

unsigned long previousMillis = 0;
const unsigned long interval = 500;  // milliseconds

Kalman filter_A(0.02, 0.02, 0.1);  // test val
Kalman filter_B(0.02, 0.02, 0.1);  // test val
Kalman filter_D(0.02, 0.02, 0.1);  // test val

void loop() {
  //     double delta_phi[4] = {-377, 377, -377, 377};  // distance in mm moved by each wheel
  //     double L = 60.0;  // in mm
  //     double W = 60.0;  // in mm

  //     double x = 0.0, y = 0.0, theta = 0.0;  // Pose in mm and radians

  // updateMecanumOdometry(delta_phi, L, W, &x, &y, &theta);

  // printf("x = %.6f mm\ny = %.6f mm\ntheta = %.6f degree\n", x, y, theta*180/3.142);


  notify();
  // Serial.print(stick_LX);
  // Serial.print("  ");
  // Serial.print(stick_LY);
  // Serial.print("  ");
  // Serial.print(stick_RX);
  // Serial.print("  ");
  // Serial.print(stick_RY);
  // Serial.println("  ");
  PS4_move(stick_LX, stick_LY, stick_RX, stick_RY);
  // Anchor_waiting_for_response(Bot_ID);
  unsigned long currentMillis = millis();
  static int swap = 0;
  static double dist_2_A;
  static double dist_2_B;
  static double dist_2_D;
  static double distance_received;

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (swap % 3 == 0) {
      distance_received = get_UWB_Distance(Bot_ID, 0xAA);  //takes around 2ms to obtain information
      Serial.print("DIST A: ");
      Serial.print(distance_received);

      if (distance_received != -1 && distance_received > 0) {
        dist_2_A = filter_A.updateFilter(distance_received);
        Serial.print("DIST A filtered: ");
        Serial.print(dist_2_A);
      }

    } 
    else if (swap % 3 == 1) {
      distance_received = get_UWB_Distance(Bot_ID, 0xBB);
      Serial.print("DIST B: ");
      Serial.print(dist_2_B);

      if (distance_received != -1 && distance_received > 0) {
        dist_2_B = filter_B.updateFilter(distance_received);
        Serial.print("DIST B filtered: ");
        Serial.print(dist_2_B);
      }

    } 
    else {
      distance_received = get_UWB_Distance(Bot_ID, 0xDD);
      Serial.print("DIST D: ");
      Serial.print(distance_received);

      if (distance_received != -1 && distance_received > 0) {
        dist_2_D = filter_B.updateFilter(distance_received);
        Serial.print("DIST D filtered: ");
        Serial.print(dist_2_D);
      }
    }
    swap++;
    // double dist_2_A = filter_A.updateFilter(get_UWB_Distance(Bot_ID, 0xAA)); //takes around 2ms to obtain information
    // double dist_2_B = filter_B.updateFilter(get_UWB_Distance(Bot_ID, 0xBB));



    Robot robots[4] = {
      { .bot_id = 0xAA, .x = -0.8, .y = 0, .theta = 0, .last_sent_time = 0, .distances = dist_2_A },
      { .bot_id = 0xBB, .x = 0.8, .y = 0, .theta = 0, .last_sent_time = 0, .distances = dist_2_B },
      { .bot_id = 0xCC, .x = 1, .y = 0, .theta = 0, .last_sent_time = 0, .distances = 0.0000 },
      {.bot_id = 0xDD, .x = 0, .y =  -0.8, .theta = 0, .last_sent_time = 0, .distances = dist_2_D}
    };



    double x, y;
    int status = Trilateration_2D(robots, 3, 0xCC, &x, &y);
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
