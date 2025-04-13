#define Bot_ID 0xCC
#define rec_Bot_ID 0xAA

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
const unsigned long interval = 500; // milliseconds

void loop() {
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
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    Serial.print("DIST A: \t");
    Serial.print(get_UWB_Distance(Bot_ID, 0xAA));
    Serial.print("\tDIST B: \t");
    Serial.print(get_UWB_Distance(Bot_ID, 0xBB));
    
    //     Position positions[4] = {{-5, -2}, {2, -2}, {-2, 2}, {2, 2}};
    // double distances[4][MAX_ROBOTS] = {
    //     {0 ,   7.0000 ,   5.0000 ,   8.0623},
    //     {7.0000,         0,    5.6569 ,   4.0000},
    //     {5.0000 ,   5.6569,         0,    4.0000},
    //     {8.0623,    4.0000,    4.0000 ,        0}
    // };

    // double x, y;
    // int status = Trilateration_2D(positions, distances, 4, 0, &x, &y);
    // if (status == 0) {
    //     // printf("Estimated position: (%.2f, %.2f)\n", x, y);
        
    //   Serial.print("\tEstimated position: ");
    //   Serial.print(x);
    //   Serial.print(" ");
    //   Serial.print(y);

    // } else {
    //     // printf("Trilateration failed. Error code: %d\n", status);
    //   Serial.print("Trilateration failed." );
    // }
    
    Serial.println();

  }

}
