#define Bot_ID 0xAA
#define rec_Bot_ID 0xBB
#include "UWB.h"
#include "kalmanFilter.h"


Kalman filter_A(0.02, 0.02, 0.01);  // test val

float this_anchor_target_distance = 1; //measured distance to anchor in m
uint16_t this_anchor_Adelay = 16600; //starting value
uint16_t Adelay_delta = 100; //initial binary search step size

void setup() {
  UWB_setup();
  // dwt_setrxantennadelay(this_anchor_Adelay);
  // dwt_settxantennadelay(this_anchor_Adelay);
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
  Sleep(100);

  
//   static float last_delta = 0.0;
//   // Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), DEC);

//     float dist = get_UWB_Distance(Bot_ID, rec_Bot_ID);

//   Serial.print(",");
//   Serial.print(dist); 
//   if (Adelay_delta < 3) {
//     Serial.print(", final Adelay ");
//     Serial.println(this_anchor_Adelay);
// //    Serial.print("Check: stored Adelay = ");
// //    Serial.println(DW1000.getAntennaDelay());
//     while(1);  //done calibrating
//   }

//   float this_delta = dist - this_anchor_target_distance;  //error in measured distance

//   if ( this_delta * last_delta < 0.0) Adelay_delta = Adelay_delta / 2; //sign changed, reduce step size
//     last_delta = this_delta;
  
//   if (this_delta > 0.0 ) this_anchor_Adelay += Adelay_delta; //new trial Adelay
//   else this_anchor_Adelay -= Adelay_delta;
  
//   Serial.print(", Adelay = ");
//   Serial.println (this_anchor_Adelay);
// //  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
//   dwt_setrxantennadelay(this_anchor_Adelay);
//   dwt_settxantennadelay(this_anchor_Adelay);
  
//   Sleep(100);
}

 