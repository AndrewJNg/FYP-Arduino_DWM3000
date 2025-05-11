
#include "Motor_Subsystem.h"


//{MotorPin1, MotorPin2, EncAnalogPin, EncDir}
MotorControl frontLeftMotor(12, 14, 34, 0);
MotorControl frontRightMotor(4, 2, 39, 1);
MotorControl backLeftMotor(27, 26, 35, 0);
MotorControl backRightMotor(17, 16, 36, 1);

MotionParameters motionParams1;
MotionParameters motionParams2;
MotionParameters motionParams3;
MotionParameters motionParams4;
int Start = 0;
void setup() {
  Serial.begin(115200);
  frontLeftMotor.setupMotorSystem();
  frontRightMotor.setupMotorSystem();
  backLeftMotor.setupMotorSystem();
  backRightMotor.setupMotorSystem();

  // motor_subsystem_setup();
  delay(2000);
  // generateStepResponse();

  motionParams1 = frontLeftMotor.calculateTrapezoidalProfile(-2000, 0, 0);
  motionParams2 = frontRightMotor.calculateTrapezoidalProfile(-2000, 0, 0);
  motionParams3 = backLeftMotor.calculateTrapezoidalProfile(2000, 0, 0);
  motionParams4 = backRightMotor.calculateTrapezoidalProfile(2000, 0, 0, 400);
  frontLeftMotor.resetPID();
  frontRightMotor.resetPID();
  backLeftMotor.resetPID();
  backRightMotor.resetPID();

  // Start = 1;
  // static unsigned long prev_FL = 0;
  // // long start_en = frontRightMotor.updateEncoder();
  while ((motionParams4.time_step < (motionParams4.T))) {
    frontLeftMotor.followProfile(&motionParams1);
    frontRightMotor.followProfile(&motionParams2);
    backLeftMotor.followProfile(&motionParams3);
    backRightMotor.followProfile(&motionParams4);

    // if ((millis() - prev_FL) >= 10) {
    //   prev_FL = millis();
    //   Serial.print(millis());
    //   Serial.print(" , ");
    //   Serial.print(backRightMotor.updateEncoder());
    //   Serial.println();
    // }
  }

  // MotionParameters motionParamsX = frontLeftMotor.calculateTrapezoidalProfile(-1000, 0, 0);   // X movement
  // MotionParameters motionParamsY = frontRightMotor.calculateTrapezoidalProfile(-1000, 0, 0);  // Y movement

  // // Reset all PIDs
  // frontLeftMotor.resetPID();
  // frontRightMotor.resetPID();
  // backLeftMotor.resetPID();
  // backRightMotor.resetPID();

  // // Run the combined profile
  // while ((motionParamsX.time_step < motionParamsX.T) || (motionParamsY.time_step < motionParamsY.T)) {
  //   followXYProfile(&motionParamsX, &motionParamsY);  // 1.0, 1.0 are direction coefficients
  // }
}


void loop() {
  // frontLeftMotor.resetPID();
  // frontRightMotor.resetPID();
  // backLeftMotor.resetPID();
  // backRightMotor.resetPID();

  frontLeftMotor.setMotorPWM(0);
  frontRightMotor.setMotorPWM(0);
  backLeftMotor.setMotorPWM(0);
  backRightMotor.setMotorPWM(0);

  // frontLeftMotor.setSpeed(200, 0);
  // frontRightMotor.setSpeed(200, 0);
  // backLeftMotor.setSpeed(200, 0);
  // backRightMotor.setSpeed(200, 0);
  Start = 0;

  // Serial.println();

  // Serial.print(frontLeftMotor.updateEncoder());
  // Serial.print(" , ");
  // Serial.print(frontLeftMotor.angle2mm());
  // Serial.print(" Speed: ");
  // Serial.print(" , ");
  // Serial.print(frontLeftMotor.setSpeed(0,0));
  // Serial.print("  ");

  // frontLeftMotor.setSpeed(300, 0);
  // frontRightMotor.setSpeed(300, 0);
  // backLeftMotor.setSpeed(300, 0);
  // backRightMotor.setSpeed(300, 0);

  // frontLeftMotor.setSpeed(300,0);

  // Serial.println();
  // frontLeftMotor.setSpeed(500, 0);
  // delay(10);
}
void followXYProfile(MotionParameters* motionParamsX, MotionParameters* motionParamsY) {
  int motor_update_interval = 50;
  unsigned long currentTime = millis();
  if ((currentTime - motionParamsX->prev_time) >= motor_update_interval) {
    motionParamsX->prev_time = currentTime;
    motionParamsY->prev_time = currentTime;

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

      // Combine X and Y components for each wheel
      float fl_speed = current_Vx + current_Vy;
      float fr_speed = current_Vx - current_Vy;
      float bl_speed = current_Vx - current_Vy;
      float br_speed = current_Vx + current_Vy;

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

void generateStepAtPWM(int pwm) {

  frontLeftMotor.setMotorPWM(pwm);
  // rightMotor.setMotorPWM(pwm);

  static unsigned long startMillis = 0;
  static unsigned long prevMillis = 0;
  startMillis = millis();
  unsigned long currentMillis = millis();
  do {
    currentMillis = millis();
    if ((currentMillis - prevMillis) >= 10) {
      Serial.print(" ");
      Serial.print(currentMillis);
      Serial.print(" ");
      Serial.print(frontLeftMotor.angle2mm());
      Serial.print(" ");
      // Serial.print(rightMotor.angle2mm());
      Serial.print(frontLeftMotor.angle2mm());
      Serial.println();

      prevMillis = currentMillis;
    }


    // startMillis = currentMillis ;
  } while ((currentMillis - startMillis) <= 2000);
}
void generateStepResponse() {

  Serial.println("Start");
  frontLeftMotor.resetPID();
  // rightMotor.resetPID();

  generateStepAtPWM(4095);
  generateStepAtPWM(3072);
  generateStepAtPWM(2048);
  generateStepAtPWM(1024);
  generateStepAtPWM(0);

  frontLeftMotor.setMotorPWM(0);

  Serial.println("End");
  // rightMotor.setMotorPWM(0);
}
