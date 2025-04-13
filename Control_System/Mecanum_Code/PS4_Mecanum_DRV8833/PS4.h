// Allows control using a PS3 controller by using the github library by jvpernis
//https://github.com/jvpernis/esp32-ps3
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include"esp_gap_bt_api.h"
#include "esp_err.h"

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

#define REMOVE_BONDED_DEVICES 1   // <- Set to 0 to view all bonded devices addresses, set to 1 to remove

#define PAIR_MAX_DEVICES 20
uint8_t pairedDeviceBtAddr[PAIR_MAX_DEVICES][6];
char bda_str[18];

bool initBluetooth()
{
  if(!btStart()) {
    Serial.println("Failed to initialize controller");
    return false;
  }
 
  if(esp_bluedroid_init() != ESP_OK) {
    Serial.println("Failed to initialize bluedroid");
    return false;
  }
 
  if(esp_bluedroid_enable() != ESP_OK) {
    Serial.println("Failed to enable bluedroid");
    return false;
  }
  return true;
}

char *bda2str(const uint8_t* bda, char *str, size_t size)
{
  if (bda == NULL || str == NULL || size < 18) {
    return NULL;
  }
  sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
          bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  return str;
}

#include <PS4Controller.h>

// PS3 Status Variables
int player = 2;
int battery = 0;

// PS3 Control Variables (Movement)
int stick_LX;
int stick_LY;
int stick_RX;
int stick_RY;

// PS3 Control Variables (Servos)
int Triangle = 0;  // 0 = not pressed (hold position), 1 = pressed (Request Scoop)
int Square = 0;    // 0 = not pressed (hold position), 1 = pressed (Request Ball Release)
int L1 = 0;        // 0 = not pressed (hold position), 1 = pressed (Tighten Gripper)
int R1 = 0;        // 0 = not pressed (hold position), 1 = pressed (Loosen Gripper)

// PS3 Control Variables (Stepper)
int L2 = 0;  // 0 = not pressed (hold position), 1 = pressed (Move Gripper Up)
int R2 = 0;  // 0 = not pressed (hold position), 1 = pressed (Move Gripper Down)

// Internal Variable Changes when PS3 Control changes
void notify() {
  // Movement
  if (abs(PS4.LStickX()) + abs(PS4.LStickY()) > 2) {
    stick_LX = PS4.LStickX();
    stick_LY = PS4.LStickY();
  }
  if (abs(PS4.RStickX()) + abs(PS4.RStickY()) > 2) {
    stick_RX = PS4.RStickX();
    stick_RY = PS4.RStickY();
  }
}

void onConnect() {
  Serial.println("Connected.");
}

void PS4_setup() {
  
  initBluetooth();
  Serial.print("ESP32 bluetooth address: "); Serial.println(bda2str(esp_bt_dev_get_address(), bda_str, 18));
  // Get the numbers of bonded/paired devices in the BT module
  int count = esp_bt_gap_get_bond_device_num();
  if(!count) {
    Serial.println("No bonded device found.");
  } else {
    Serial.print("Bonded device count: "); Serial.println(count);
    if(PAIR_MAX_DEVICES < count) {
      count = PAIR_MAX_DEVICES; 
      Serial.print("Reset bonded device count: "); Serial.println(count);
    }
    esp_err_t tError =  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
    if(ESP_OK == tError) {
      for(int i = 0; i < count; i++) {
        Serial.print("Found bonded device # "); Serial.print(i); Serial.print(" -> ");
        Serial.println(bda2str(pairedDeviceBtAddr[i], bda_str, 18));     
        if(REMOVE_BONDED_DEVICES) {
          esp_err_t tError = esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
          if(ESP_OK == tError) {
            Serial.print("Removed bonded device # "); 
          } else {
            Serial.print("Failed to remove bonded device # ");
          }
          Serial.println(i);
        }
      }        
    }
  }
  // Serial.begin(115200);
  // PS4.begin("1a:2b:3c:01:01:01");
  // Serial.println("Ready.");
  
  PS4.begin("aa:aa:aa:aa:aa:aa");
  // Ps4.begin("bb:bb:bb:bb:bb:bb");
  // Ps4.begin("cc:cc:cc:cc:cc:cc");
  // PS4.begin("dd:dd:dd:dd:dd:dd");
  // PS4.begin("ee:ee:ee:ee:ee:ee");
  // PS4.begin("f8:9e:94:7f:48:37");
  // PS4.begin("1a:2b:3c:01:01:01");
  // PS4.begin("c8:f0:9e:75:04:26");
}

// Helper Function to Calculate Analog Stick Angle
float PS4_LeftAnalogStickAngle(int LX, int LY) {
  float LX_vector = map(LX, -128, 127, -10000, 10000) / 100;
  float LY_vector = map(LY, -127, 128, -10000, 10000) / 100;
  float angle;
  if (LY_vector == 0 && LX_vector > 0) angle = PI / 2;
  else if (LY_vector == 0 && LX_vector < 0) angle = 3 * PI / 2;
  else if (LY_vector == 0 && LX_vector == 0) angle = 0;
  else angle = atan(abs(LX_vector) / abs(LY_vector));

  if (LX_vector > 0 && LY_vector > 0) angle = angle;
  else if (LX_vector > 0 && LY_vector < 0) angle = PI - angle;
  else if (LX_vector < 0 && LY_vector < 0) angle = PI + angle;
  else if (LX_vector < 0 && LY_vector > 0) angle = 2 * PI - angle;

  return angle;
}

// Helper Function to Calculate Analog Stick Speed
float PS4_LeftAnalogStickSpeed(int LX, int LY) {
  float LX_vector = 0;
  float LY_vector = 0;
  if (abs(LX) > 15) LX_vector = map(LX, -128, 127, -10000, 10000) / 100;
  //  else LX_vector = 0;
  if (abs(LY) > 15) LY_vector = map(LY, -127, 128, -10000, 10000) / 100;
  //  else LY_vector = 0;
  float Speed = sqrt(LX_vector * LX_vector + LY_vector * LY_vector);
  if (Speed > 100) Speed = 100;

  return Speed;
}
