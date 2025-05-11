// uint8_t Bot_ID = 0xDD;
// uint8_t Bot_ID = 0xEE;
// uint8_t Bot_ID = 0xFF;
// uint8_t Bot_ID = 0xCC;
// uint8_t Bot_ID = 0xBB;
// uint8_t Bot_ID = 0xAA;
// #define AntennaDelay 16370
#define DEBUG_OUTPUT false
// #define fixed_base false
#define fixed_base true

// uint8_t Bot_ID = 0xAA;
// #define DWM3000_RST 27
// #define DWM3000_IRQ 34
// #define DWM3000_SS 4

// uint8_t Bot_ID = 0xBB;
// #define DWM3000_RST 27
// #define DWM3000_IRQ 34
// #define DWM3000_SS 4

uint8_t Bot_ID = 0xEE;
#define DWM3000_RST 15
#define DWM3000_IRQ 13
#define DWM3000_SS 5

#define xStart 1
#define yStart 1


// Functional Macros
#include "PS4.h"
// #include "Movement.h"
#include "UWB.h"
#include "kalmanFilter.h"
#include "multilateration.h"
#include "MPU6050.h"
#include "Motor_Subsystem.h"

//{MotorPin1, MotorPin2, EncAnalogPin, EncDir}
MotorControl frontLeftMotor(12, 14, 34, 0);
MotorControl frontRightMotor(4, 2, 39, 1);
MotorControl backLeftMotor(27, 26, 35, 0);
MotorControl backRightMotor(17, 16, 36, 1);

DWM3000 dwm3000(DWM3000_RST, DWM3000_IRQ, DWM3000_SS, Bot_ID, false);
RobotSwarm swarm(Bot_ID);

int8_t positions[3] = { xStart, yStart, 0 };
int8_t velocities[3] = { 0, 0, 0 };
double real_pos[3], real_velocity[3];

// void followXYProfile(MotionParameters* motionParamsX, MotionParameters* motionParamsY);
void followXYProfile(MotionParameters* motionParamsX, MotionParameters* motionParamsY, float targetAngle);
void setup() {
  Serial.begin(115200);

  // PS4_setup();
  frontLeftMotor.setupMotorSystem();
  frontRightMotor.setupMotorSystem();
  backLeftMotor.setupMotorSystem();
  backRightMotor.setupMotorSystem();
  // Movement_setup();
  Gyro_setup();

  // while (!PS4.isConnected()) {
  //   motor(motor_Speed);
  // }


  // Create the task on Core 0 (ESP32 has core 0 and core 1)
  // xTaskCreatePinnedToCore(
  //   DW3000_Module,    // Function to run
  //   "DW3000_Module",  // Name of task
  //   4096,             // Stack size (words, not bytes)
  //   &Bot_ID,          // Parameter to pass
  //   1,                // Priority (1 is usually fine)
  //   NULL,             // Task handle (not used here)
  //   0                 // Core to pin to (0 = PRO core)
  // );

  // Add base robot, with starting positions and distance must always be 0
  // swarm.update_robot(Bot_ID, xStart, yStart, 0);  // set base address at origin

  // This will automatically create a new robot if it doesn't exist, or update it if it has existed
  // swarm.update_robot(0xAA, 2, 2, 2.82);
  // swarm.update_robot(0xBB, 2, 0, 2);
  // swarm.update_robot(0xCC, 0, 2, 2);
}

void DW3000_Module(void* parameter) {
  uint16_t sender_id = *(uint16_t*)parameter;

  dwm3000.begin();

  while (true) {
    static uint32_t last_switch = 0;
    uint32_t now = millis();

    ////////////////////////////////////////////////////////////////////////////////
    if (now - last_switch >= random(150, 501)) {
      last_switch = now;

      //////////////////////////////////////////////////////////////
      // First remove any stale robots
      swarm.remove_stale_robots();

      // Get all robot addresses in the swarm
      std::vector<int16_t> addresses = swarm.get_all_addresses();

      if (addresses.size() == 1) {  //if it doesn't know any other robots, send a msg to itself, so that others can identify it is active
        dwm3000.getDistance(Bot_ID, positions, velocities);
        // Serial.println("send personal address");
      } else {

        // Iterate through all robots
        for (int16_t address : addresses) {
          if (address == sender_id) continue;  // Skip our own address
                                               // for (int i = 0; i < 5; i++) {
          // Get distance measurement (takes about 5ms per measurement)
          // Serial.print("Pinging: ");
          // Serial.println(address,HEX);
          double distance_received = dwm3000.getDistance(address, positions, velocities);

          if (distance_received != -1 && (distance_received > 0 && distance_received < 50)) {
            // Get current position info
            int8_t dw_pos[3], dw_velocity[3];
            dwm3000.getRobotInfo(dw_pos, dw_velocity);  // update rest of DWM3000 received data

            swarm.update_robot(address, dw_pos[0], dw_pos[1], distance_received);  // Update swarm structure with new measurement

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            if (DEBUG_OUTPUT) {
              Serial.print("Updated robot 0x");
              Serial.print(address, HEX);
              Serial.print(" - Distance: ");
              Serial.print(distance_received);
              Serial.print("m, Position: (");
              Serial.print(dw_pos[0]);
              Serial.print(", ");
              Serial.print(dw_pos[1]);
              Serial.println(")");
            }
          }

          vTaskDelay(50);  // short delay to yield CPU
          // }
        }
      }
      if (fixed_base) {
        double fake_pos[3], dw_velocity[3];
        swarm.update_base_position(Bot_ID, fake_pos);  // do not update current position
      } else {
        swarm.update_base_position(Bot_ID, real_pos);  // update current positions
      }
      // positions[3]
      if (DEBUG_OUTPUT) Serial.println("Completed full swarm measurement cycle");

    } else {
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // default back to Anchor mode, listen to any tags and also identify any new addresses that just added into the system
      // - Listen to tags, and if address matches base address, find the distance to it
      // - If address is not base address but another known address, update last addressed time to indicate robot is active in the swarm
      // - If address is unknown, add that as a new address in the system
      double distance = dwm3000.processIncoming(positions, velocities);

      if (distance != -1) {
        // Serial.print("anchor dist found: ");
        // Serial.println(distance);
        int8_t dw_pos[3], dw_velocity[3];
        uint16_t address = dwm3000.getRobotInfo(dw_pos, dw_velocity);
        swarm.update_robot(address, dw_pos[0], dw_pos[1], (distance > 0 && distance < 50) ? distance : -1);
        // Serial.print("New address found: ");
        // Serial.println(address, HEX);
      }
    }
    // vTaskDelay(1);  // short delay to yield CPU
  }
}



void loop() {

  delay(2000);
  MotionParameters motionParamsX = frontLeftMotor.calculateTrapezoidalProfile(-1000, 0, 0, 400);   // X movement
  MotionParameters motionParamsY = frontRightMotor.calculateTrapezoidalProfile(-1000, 0, 0, 400);  // Y movement

  // Reset all PIDs
  frontLeftMotor.resetPID();
  frontRightMotor.resetPID();
  backLeftMotor.resetPID();
  backRightMotor.resetPID();

  float targetAngle = MPU_Z_angle();

  // Run the combined profile
  while ((motionParamsX.time_step < motionParamsX.T) || (motionParamsY.time_step < motionParamsY.T)) {
    // followXYProfile(&motionParamsX, &motionParamsY);  // 1.0, 1.0 are direction coefficients
    followXYProfile(&motionParamsX, &motionParamsY, targetAngle);  // 1.0, 1.0 are direction coefficients
    Gyro_update();
  }

  while (true) {
    // Gyro_update();

    // Print all robot information
    // swarm.print_Robot_Swarm();

    frontLeftMotor.setMotorPWM(0);
    frontRightMotor.setMotorPWM(0);
    backLeftMotor.setMotorPWM(0);
    backRightMotor.setMotorPWM(0);

    // notify();
    // PS4_move(stick_LX, stick_LY, stick_RX, stick_RY);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Update base position based on all info
    // double target_x = 1;
    // double target_y = 1;
    // float angle = 0;


    // Serial.print(real_pos[0]);
    // Serial.print("  ");
    // Serial.print(real_pos[1]);
    // Serial.print("  ");
    // Serial.print(angle);
    // Serial.print("  ");
    // Serial.print(Speed_total_percent);
    // Serial.print("  ");


    // Serial.print(motor_Speeds[0]);
    // Serial.print("  ");
    // Serial.print(motor_Speeds[1]);
    // Serial.print("  ");
    // Serial.print(motor_Speeds[2]);
    // Serial.print("  ");
    // Serial.print(motor_Speeds[3]);
    // Serial.println("  ");

    // Serial.print("Gyro z: ");
    // Serial.print(MPU_Z_angle());
    // Serial.println("");

    //apply turn Speed to allow control using right analog stick
    // motor_Speed[0] = motor_Speed[0] + turn_Speed;
    // motor_Speed[1] = motor_Speed[1] - turn_Speed;
    // motor_Speed[2] = motor_Speed[2] + turn_Speed;
    // motor_Speed[3] = motor_Speed[3] - turn_Speed;

    // run all motors with according speeds
    // if(fixed_base==false)  motor(motor_Speeds);
    // vTaskDelay(100);  // Suspend main loop
  }
}


void followXYProfile(MotionParameters* motionParamsX, MotionParameters* motionParamsY, float targetAngle) {
  static float initialAngle = NAN;
  static float totalRotation = 0;

  int motor_update_interval = 50;
  unsigned long currentTime = millis();
  if ((currentTime - motionParamsX->prev_time) >= motor_update_interval) {
    motionParamsX->prev_time = currentTime;
    motionParamsY->prev_time = currentTime;


    // Initialize target angle tracking
    if (!isnan(targetAngle) && isnan(initialAngle)) {
      initialAngle = MPU_Z_angle();
      totalRotation = 0;
    }


    if (motionParamsX->time_step <= motionParamsX->T || motionParamsY->time_step <= motionParamsY->T) {
      // Calculate X component velocity
      float current_Vx = 0;
      if (motionParamsX->time_step <= motionParamsX->T) {
        if (motionParamsX->time_step < motionParamsX->ta) {
          current_Vx = motionParamsX->acceleration * (motionParamsX->time_step / 1000);
        } else if ((motionParamsX->time_step >= motionParamsX->ta) && (motionParamsX->time_step <= motionParamsX->tcf)) {
          current_Vx = motionParamsX->velocity;
        } else if (motionParamsX->time_step > motionParamsX->tcf) {
          current_Vx = motionParamsX->velocity - ((motionParamsX->time_step / 1000) - (motionParamsX->tcf / 1000)) * motionParamsX->deceleration;
        }
        motionParamsX->time_step += motor_update_interval;
      }

      // Calculate Y component velocity
      float current_Vy = 0;
      if (motionParamsY->time_step <= motionParamsY->T) {
        if (motionParamsY->time_step < motionParamsY->ta) {
          current_Vy = motionParamsY->acceleration * (motionParamsY->time_step / 1000);
        } else if ((motionParamsY->time_step >= motionParamsY->ta) && (motionParamsY->time_step <= motionParamsY->tcf)) {
          current_Vy = motionParamsY->velocity;
        } else if (motionParamsY->time_step > motionParamsY->tcf) {
          current_Vy = motionParamsY->velocity - ((motionParamsY->time_step / 1000) - (motionParamsY->tcf / 1000)) * motionParamsY->deceleration;
        }
        motionParamsY->time_step += motor_update_interval;
      }
      // Calculate rotation correction if target angle is specified
      float rotationSpeed = 0;
      if (!isnan(targetAngle)) {
        float currentAngle = MPU_Z_angle();
        float angleError = targetAngle - currentAngle;

        // 1. Normalize angle error to [-180, 180]
        while (angleError > 180) angleError -= 360;
        while (angleError < -180) angleError += 360;

        // 2. Add deadband to prevent jitter (adjust as needed)
        const float deadband = 2.0f;  // degrees
        if (fabs(angleError) < deadband) {
          angleError = 0;
        }

        // 3. More aggressive control parameters
        const float rotationP = 10.0f;          // Increased from 0.1f
        const float maxRotationSpeed = 400.0f;  // Your max system speed (mm/s)
        const float WHEELBASE_RADIUS = 60.0f;   // Example value (mm) - measure your robot!

        // 4. Calculate raw rotation speed (deg/s to mm/s conversion)
        rotationSpeed = angleError * rotationP;

        // 5. Apply non-linear response for better control
        if (fabs(angleError) > 10.0f) {  // Aggressive correction for large errors
          rotationSpeed *= 1.5f;
        }

        // 6. Final constraints
        rotationSpeed = constrain(rotationSpeed, -maxRotationSpeed, maxRotationSpeed);

        // Debug output
        Serial.print("AngleErr: ");
        Serial.print(angleError);
        Serial.print("° | RotSpeed: ");
        Serial.print(rotationSpeed);
        Serial.println(" mm/s");
      }

      // Combine X, Y and rotation components for each wheel
      // Mecanum wheel equations with rotation:
      // FL: Vx - Vy + Vrot
      // FR: Vx + Vy - Vrot
      // BL: Vx + Vy + Vrot
      // BR: Vx - Vy - Vrot
      // Serial.println(rotationSpeed);
      float fl_speed = current_Vx + current_Vy + rotationSpeed;
      float fr_speed = current_Vx - current_Vy - rotationSpeed;
      float bl_speed = current_Vx - current_Vy + rotationSpeed;
      float br_speed = current_Vx + current_Vy - rotationSpeed;


      // Set speeds for each motor
      frontLeftMotor.setSpeed(fl_speed, 0);  // Note: acceleration might need adjustment
      frontRightMotor.setSpeed(fr_speed, 0);
      backLeftMotor.setSpeed(bl_speed, 0);
      backRightMotor.setSpeed(br_speed, 0);
    } else {
      // Stop all motors when both profiles are complete
      frontLeftMotor.stopMotor();
      frontRightMotor.stopMotor();
      backLeftMotor.stopMotor();
      backRightMotor.stopMotor();
    }
  }
}
