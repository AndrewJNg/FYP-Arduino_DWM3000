
#include "Motor_Subsystem.h"



void setup() {
  Serial.begin(115200);
  frontLeftMotor.setupMotor();
  motor_subsystem_setup();
}


void loop() {
  // Serial.print(analogRead(35)>1048);
  // Serial.print("  ");
  // Serial.print(analogRead(34)>2048);
  // Serial.print("  ");
  // Serial.print(analogRead(36)>2048);
  // Serial.print("  ");
  // Serial.print(analogRead(39)>2048);

  // Serial.print("  ");
  // Serial.print(leftMotor.updateEncoder() );
  // leftMotor.setMotorPWM(2000);
  // delay(1000);
  // leftMotor.setMotorPWM(1000);
  // delay(1000);
  // leftMotor.setMotorPWM(0);
  // delay(1000);


  // Serial.print(leftMotor.angle2mm() );
  // leftMotor.setSpeed(0, 0);
  // frontLeftMotor.setMotorPWM(2000);

  // Serial.print(digitalRead(35));
  // Serial.print("  ");
  // Serial.print(digitalRead(34));
  // Serial.print("  ");
  // Serial.print(digitalRead(36));
  // Serial.print("  ");
  // Serial.println(digitalRead(39));
  // generateStepResponse();
  Serial.println();
  // delay(100);

  while (true) {
  // frontLeftMotor.setMotorPWM(1000);
    // Serial.print(frontLeftMotor.updateEncoder());
    // Serial.println();
    frontLeftMotor.setSpeed(500, 0);
    // delay(10);
  }
}



// MotorControl leftMotor(
//   4,    // motorPin1
//   2,    // motorPin2
//   15,   // motorPWM
//   0,    // ESP32 motorChannel
//   Wire  // I2C bus for encoder
// );

// MotorControl rightMotor(
//   16,    // motorPin1
//   17,    // motorPin2
//   5,     // motorPWM
//   1,     // ESP32 motorChannel
//   Wire1  // I2C bus for encoder
// );

void motor_subsystem_setup() {
  // Wire.begin(SDA_1, SCL_1, i2c_speed);
  // Wire1.begin(SDA_2, SCL_2, i2c_speed);

  // leftMotor.setupEncoder(AS5600_COUNTERCLOCK_WISE);  //set Counter_clockwise rotation
  // leftMotor.setupMotor(1);

  // rightMotor.setupEncoder(AS5600_CLOCK_WISE);  //set Clockwise rotation
  // rightMotor.setupMotor(1);



  double Td = 0.05;
  // Tm = 0.03
  // Km = 3.57
  frontLeftMotor.Tm = 0.035;
  frontLeftMotor.FF_K_offset = -1011;
  frontLeftMotor.FF_K_velocity = 4.77;
  frontLeftMotor.FF_K_accel = 4096 * frontLeftMotor.Tm;  //max voltage over max acceleration, accel = 1/tau

  frontLeftMotor.PID_BIAS = 0;
  frontLeftMotor.PID_Kp = 7.80;  // (leftMotor.Tm * 64.0) / (leftMotor.FF_K_velocity * (double)sqrt(2.0)*sqrt(2.0) * Td * Td);
  // leftMotor.PID_Kp = KpLeft;
  frontLeftMotor.PID_Ki = 0;
  frontLeftMotor.PID_Kd = 0.04;  //(8 * leftMotor.Tm - Td) / (Td * leftMotor.FF_K_velocity);

  /*
  //////////////////////////////////////////////////
  // Tm = 0.03
  // Km = 4.08
  rightMotor.Tm = 0.035;
  rightMotor.FF_K_offset = 784;
  rightMotor.FF_K_velocity = 4.05;
  // rightMotor.FF_K_accel = 4096*leftMotor.Tm;  //max voltage over max acceleration, accel = 1/tau

  rightMotor.PID_BIAS = 0;
  rightMotor.PID_Kp = 7.8;//(rightMotor.Tm*32.0) / (rightMotor.FF_K_velocity * (double) sqrt(2.0) * Td* Td) ;
  // rightMotor.PID_Kp = 0;
  rightMotor.PID_Ki = 0;
  // rightMotor.PID_Kd = 0;
  rightMotor.PID_Kd = 0.04; //(8 * rightMotor.Tm - Td ) / (Td*rightMotor.FF_K_velocity);
*/
}
