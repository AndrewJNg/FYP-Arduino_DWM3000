#define Bot_ID 0xAA
#include "UWB.h"



void setup() {
  UWB_setup();
}

void loop() {
  Anchor_waiting_for_response(Bot_ID);  // Respond to messages if it is called
}
