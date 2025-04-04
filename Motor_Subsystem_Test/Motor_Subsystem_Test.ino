
#include "Motor_Subsystem.h"


MotorControl frontLeftMotor(
  26,  // motorPin1
  27,  // motorPin2
  25,  // motorPWM
  35,  // analogPin
  1    // Base direction for encoder
);

MotorControl frontRightMotor(
  12,  // motorPin1
  14,  // motorPin2
  13,  // motorPWM
  34,  // analogPin
  -1   // Base direction for encoder
);

MotorControl backLeftMotor(
  17,  // motorPin1
  16,  // motorPin2
  5,   // motorPWM
  36,  // analogPin
  -1   // Base direction for encoder
);

MotorControl backRightMotor(
  2,   // motorPin1
  4,   // motorPin2
  15,  // motorPWM
  39,  // analogPin
  1    // Base direction for encoder
);



void setup() {
  Serial.begin(115200);
  frontLeftMotor.setupMotor();
  frontRightMotor.setupMotor();
  backLeftMotor.setupMotor();
  backRightMotor.setupMotor();

  delay(2000);
  // frontLeftMotor.setMotorPWM(1000);
  // frontLeftMotor.generateStepResponse();
  // frontRightMotor.generateStepResponse();
  // backLeftMotor.generateStepResponse();
  // backRightMotor.generateStepResponse();
}


void loop() {

  // MotionParameters MotionParams;

  // MotionParams = frontLeftMotor.calculateTrapezoidalProfile(1000, 400, 200);

  // while ((MotionParams.time_step < (MotionParams.T))) {
  //   frontLeftMotor.followProfile(&MotionParams);
  //   frontRightMotor.followProfile(&MotionParams);
  //   backLeftMotor.followProfile(&MotionParams);
  //   backRightMotor.followProfile(&MotionParams);
  // }

  MotionParameters frontLeftMotionParams, frontRightMotionParams, backLeftMotionParams, backRightMotionParams;

  // frontLeftMotionParams = frontLeftMotor.calculateTrapezoidalProfile(-1000, -400, -200);
  frontRightMotionParams = frontRightMotor.calculateTrapezoidalProfile(1000, 400, 500);
  // backLeftMotionParams = backLeftMotor.calculateTrapezoidalProfile(1000, 400, 200);
  // backRightMotionParams = backRightMotor.calculateTrapezoidalProfile(-1000, -400, -200);

  while ((frontLeftMotionParams.time_step < (frontLeftMotionParams.T))) {
    // frontLeftMotor.followProfile(&frontLeftMotionParams);
    frontRightMotor.followProfile(&frontRightMotionParams);
    // backLeftMotor.followProfile(&backLeftMotionParams);
    // backRightMotor.followProfile(&backRightMotionParams);
  }

  while (true) {
    // frontLeftMotor.setSpeed(-300, 0);
    // frontRightMotor.setSpeed(300, 0);
    // backLeftMotor.setSpeed(300, 0);
    // backRightMotor.setSpeed(-300, 0);

    frontLeftMotor.stopMotor();
    frontRightMotor.stopMotor();
    backLeftMotor.stopMotor();
    backRightMotor.stopMotor();
  }
}
