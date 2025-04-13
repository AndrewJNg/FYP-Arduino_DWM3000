#define Bot_ID 0xCC
#define rec_Bot_ID 0xAA

// Functional Macros
#include "PS4.h"
#include "Movement.h"
#include "UWB.h"
#include "kalmanFilter.h"

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
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    Serial.print("DIST A: \t");
    Serial.print(get_UWB_Distance(Bot_ID, 0xAA));
    Serial.print("\tDIST B: \t");
    Serial.print(get_UWB_Distance(Bot_ID, 0xBB));
    Serial.println();
  }

}
