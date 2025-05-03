// motor control system for DRV8833 motor driver
// Also includes PS4 to motor control


//  motor control pin layout   [ Left Front, Right Front , Back Left , Back Right]
const byte motorPin1[] = { 12, 4, 27, 17 };
const byte motorPin2[] = { 14, 2, 26, 16 };

int motor_Speed[] = { 0, 0, 0, 0 };
const byte PWM_resolution = 8;             
const int PWM_resolution_max_value = 255;  

void motor(int Speed[]) {
  for (int x = 0; x < sizeof(motorPin1); x++) {
    if (Speed[x] > PWM_resolution_max_value) Speed[x] = PWM_resolution_max_value;
    else if (Speed[x] < -PWM_resolution_max_value) Speed[x] = -PWM_resolution_max_value;

    if (Speed[x] > 0) {  // Forward
      ledcWrite(motorPin1[x], abs(Speed[x]));
      ledcWrite(motorPin2[x], 0);

    } else if (Speed[x] < 0) {  // Reverse
      ledcWrite(motorPin1[x], 0);
      ledcWrite(motorPin2[x], abs(Speed[x]));
    
    } else {  //Stop
      ledcWrite(motorPin1[x], 0);
      ledcWrite(motorPin2[x], 0);
    }

  }
}

void PS4_move(int LX, int LY, int RX, int RY) {
  // angle from 0 to 2*pi
  float angle = PS4_LeftAnalogStickAngle(LX, LY);

  //Speed  (range of 0 to 100)
  float Speed_total_percent = PS4_LeftAnalogStickSpeed(LX, LY);

  // turn speed for rotational movement
  float turn_Speed = 0;

  // filter to ignore inputs below 15 from controller for rotation only
  if (abs(RX) > 15) turn_Speed = map(RX, -128, 127, -PWM_resolution_max_value, PWM_resolution_max_value);



  /*Mecanum wheels programming instructions found on this website
    https://seamonsters-2605.github.io/archive/mecanum
    similar principle is applied, however the direction of motion was changed (clockwise being positive angle)
  */

  motor_Speed[0] = map(sin(angle + (1 * PI) / 4) * Speed_total_percent * 100, -10000, 10000, -PWM_resolution_max_value, PWM_resolution_max_value);
  motor_Speed[1] = map(sin(angle + (3 * PI) / 4) * Speed_total_percent * 100, -10000, 10000, -PWM_resolution_max_value, PWM_resolution_max_value);
  motor_Speed[2] = map(sin(angle + (3 * PI) / 4) * Speed_total_percent * 100, -10000, 10000, -PWM_resolution_max_value, PWM_resolution_max_value);
  motor_Speed[3] = map(sin(angle + (1 * PI) / 4) * Speed_total_percent * 100, -10000, 10000, -PWM_resolution_max_value, PWM_resolution_max_value);

  //apply turn Speed to allow control using right analog stick
  motor_Speed[0] = motor_Speed[0] + turn_Speed;
  motor_Speed[1] = motor_Speed[1] - turn_Speed;
  motor_Speed[2] = motor_Speed[2] + turn_Speed;
  motor_Speed[3] = motor_Speed[3] - turn_Speed;

  // run all motors with according speeds
  motor(motor_Speed);
}

void Movement_setup() {
  // set motor pins as output
  for (int x = 0; x < sizeof(motorPin1); x++) {
    ledcAttach( motorPin1[x], 5000, PWM_resolution);  // New ESP32 3.0 API combines Setup + Attach into a single function
    ledcAttach( motorPin2[x], 5000, PWM_resolution);  
  }
  motor(motor_Speed);  // set all wheels to stop
}
