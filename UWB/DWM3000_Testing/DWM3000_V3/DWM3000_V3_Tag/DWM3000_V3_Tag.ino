#define Bot_ID 0xCC
#define rec_Bot_ID 0xAA
#include "UWB.h"
#include "kalmanFilter.h"


Kalman filter_A(0.02, 0.02, 0.1);  // test val

void setup() {
  UWB_setup();
}

void loop() {

  double distance_received = get_UWB_Distance(Bot_ID, rec_Bot_ID);
  Serial.print("DIST: \t");
  Serial.print(distance_received);

  if (distance_received != -1) {
    Serial.print("\t DIST filtered: ");
    Serial.print(filter_A.updateFilter(distance_received));
  }
  Serial.println();
  Sleep(500);
}

