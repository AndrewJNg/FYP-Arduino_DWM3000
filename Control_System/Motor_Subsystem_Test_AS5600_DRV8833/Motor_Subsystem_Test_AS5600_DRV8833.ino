
#include "Motor_Subsystem.h"


//{MotorPin1, MotorPin2, EncAnalogPin, EncDir}
MotorControl frontLeftMotor(12, 14, 34, 0);
MotorControl frontRightMotor(4, 2, 39, 1);
MotorControl backLeftMotor(27, 26, 35, 0);
MotorControl backRightMotor(17, 16, 36, 1);

MotionParameters motionParams;
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

  motionParams = frontRightMotor.calculateTrapezoidalProfile(2000, 0, 0, 400, MAX_ACCELERATION, MAX_ACCELERATION);
  frontRightMotor.resetPID();
  Start = 1;
  static unsigned long prev_FL = 0;
  long start_en = frontRightMotor.updateEncoder();
  while ((motionParams.time_step < (motionParams.T)) && Start == 1) {
    frontLeftMotor.followProfile(&motionParams);
    frontRightMotor.followProfile(&motionParams);
    backLeftMotor.followProfile(&motionParams);
    backRightMotor.followProfile(&motionParams);

    // if ((millis() - prev_FL) >= 10) {
    //   prev_FL = millis();
    //   Serial.print(millis());
    //   Serial.print(" , ");
    //   Serial.print(frontRightMotor.updateEncoder());
    //   Serial.println();
    // }
  }
}


void loop() {
  frontLeftMotor.resetPID();
  frontRightMotor.resetPID();
  backLeftMotor.resetPID();
  backRightMotor.resetPID();
  // frontRightMotor.setSpeed(100, 0);
  // frontRightMotor.setMotorPWM(0);
  frontLeftMotor.setSpeed(0, 0);
  frontRightMotor.setSpeed(0, 0);
  backLeftMotor.setSpeed(0, 0);
  backRightMotor.setSpeed(0, 0);
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
