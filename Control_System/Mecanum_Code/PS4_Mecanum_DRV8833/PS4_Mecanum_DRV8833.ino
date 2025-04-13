// // Additional Functionalities
// #include "OTA.h"
// #include "User_interface.h"
// #include "MPU6050.h"

// Functional Macros
#include "PS4.h"
#include "Movement.h"

void setup() {
  // Setup Read Spead and Serial monitor's Baud Rate
  // analogReadResolution(12);
  Serial.begin(115200);

  // // Additional Macro Setups
  // OTA_setup();
  // User_interface_setup();
  // MPU6050_setup();

  // Macro Setups
  PS4_setup();
  Movement_setup();

  motor(motor_Speed);
  while (!PS4.isConnected()) {
    motor(motor_Speed);
  }

}

void loop() {
  notify();
    Serial.print(stick_LX);
    Serial.print("  ");
    Serial.print(stick_LY);
    Serial.print("  ");
    Serial.print(stick_RX);
    Serial.print("  ");
    Serial.print(stick_RY);
    Serial.println("  ");
  PS4_move(stick_LX, stick_LY, stick_RX, stick_RY);


}
