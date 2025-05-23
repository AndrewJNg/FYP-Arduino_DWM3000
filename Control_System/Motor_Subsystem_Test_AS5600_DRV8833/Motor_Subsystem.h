#include <cmath>
#include "lowPass.h"

// Encoder parameters
#define wheelRadius 24.0      //48mm diameter wheel
#define MAX_ACCELERATION 500  // mm/s/s
#define MAX_VELOCITY 600      // mm/s

// Motor & encoder parameter
#define PWMResolution 12
#define PWMResolutionMaxValue 4095
#define encoder_tick_per_rev 4096

// System speed
#define motor_update_freq 50  // 50Hz -> update time -> 20ms

typedef struct MotionParameters {
  float ta;
  float tc;
  float tcf;
  float T;
  float Vm;
  float s_req;
  float velocity;
  float acceleration;
  float deceleration;
  float time_step;
  int32_t prev_time;

} MotionParameters;

class MotorControl {
private:
  // Classes used
  LowPass<1> lowPassFilter;

  // Motor properties
  uint motorPin1;
  uint motorPin2;
  uint encoderAnalogPin;
  bool enc_dir_base = 0;

  // System setup
  int motor_update_interval = 0;  // matches motor_update_freq in time interval (updated in setup)

  double integral_error = 0;
  double prev_error = 0;
  double previous_measurement = 0;

  unsigned long prevMillis = 0;
  double prev_distance = 0;

  int encoder_counter = 0;

  unsigned long enc_last_update_time, enc_current_time = 0;
  
  float current_timestep_velocity = 0;
  float current_timestep_acceleration = 0;
  

  double err = 0;
  double measured_velocity = 0;
  
  int64_t _lastPosition, _position = 0;




  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
public:
  ///////////////////////////
  double Tm = 0;

  // Feedforward constants
  double FF_K_offset = 100;
  double FF_K_velocity = 8;
  double FF_K_accel = 0;

  ///////////////////////////
  // PID constants
  double PID_BIAS = 0;
  double PID_Kp = 0;
  double PID_Ki = 0;
  double PID_Kd = 0;

  ////////////////////////////////////////////// Setup ////////////////////////////////////////////////////////////////////
  MotorControl(uint pin1, uint pin2, uint analogPin, bool encDirection)
    : motorPin1(pin1),
      motorPin2(pin2),

      encoderAnalogPin(analogPin),
      enc_dir_base(encDirection),
      lowPassFilter(1, motor_update_freq, true) {}


  // //////////////////////////////////////////////// Motor control //////////////////////////////////////////////////////////////////
  void setupMotorSystem() {
    ledcAttach(motorPin1, 5000, PWMResolution);  // New ESP32 3.0 API combines Setup + Attach into a single function
    ledcAttach(motorPin2, 5000, PWMResolution);
    stopMotor();
    motor_update_interval = 1000 / motor_update_freq;  //in ms
    //   PID_Kd = PID_Kd * motor_update_freq;
    /////////////////////////////////// Motor specific tuning //////////////////////////////////////////////////////////

    Tm = 0.035;
    FF_K_offset = 913;
    FF_K_velocity = 4.5;
    FF_K_accel = 3;//4096 * Tm;  //max voltage over max acceleration, accel = 1/tau

    PID_BIAS = 0;
    PID_Kp = 20;  // (leftMotor.Tm * 64.0) / (leftMotor.FF_K_velocity * (double)sqrt(2.0)*sqrt(2.0) * Td * Td);
    PID_Ki = 0.1;
    PID_Kd = 0.005;  //(8 * leftMotor.Tm - Td) / (Td * leftMotor.FF_K_velocity);
  }
  // // Write PWM speed to motor
  void setMotorPWM(int speed) {
    speed = constrain(speed, -PWMResolutionMaxValue, PWMResolutionMaxValue);
    if (speed > 0) {
      ledcWrite(motorPin1, abs(speed));
      ledcWrite(motorPin2, 0);
    } else if (speed < 0) {
      ledcWrite(motorPin1, 0);
      ledcWrite(motorPin2, abs(speed));
    } else stopMotor();
  }

  void stopMotor() {
    ledcWrite(motorPin1, 0);
    ledcWrite(motorPin2, 0);
  }

  //////////////////////////////////////////////// Encoder feedback //////////////////////////////////////////////////////////////////
  // Encoder distance
  int64_t updateEncoder() {

    // enc_current_time = millis();
    // if (enc_current_time - enc_last_update_time >= motor_update_interval) {
    enc_last_update_time = enc_current_time;
    int16_t value = analogRead(encoderAnalogPin);

    if ((_lastPosition > 2048) && (value < (_lastPosition - 2048)))  // Rotation CW?
      _position = _position + 4096 - _lastPosition + value;
    else if ((value > 2048) && (_lastPosition < (value - 2048)))  // Rotation CCW?
      _position = _position - 4096 - _lastPosition + value;
    else
      _position = _position - _lastPosition + value;

    _lastPosition = value;
    // }
    return enc_dir_base ? _position : -_position;
  }

  double angle2mm() {
    // return ((double)updateEncoder()) / encoder_tick_per_rev;
    return (2 * M_PI * wheelRadius * (double)updateEncoder()) / encoder_tick_per_rev;
  }


  /////////////////////// Motor RPM control //////////////////
  // Feedforward - https://youtu.be/qKoPRacXk9Q?si=ahXkdiADK6ndN237
  double feedForward_Control(double velocity, double acceleration) {
    double pwm_FF = (velocity > 0) ? FF_K_offset : -FF_K_offset;
    // double pwm_FF = FF_K_offset;
    pwm_FF += FF_K_velocity * velocity + FF_K_accel * acceleration;
    return pwm_FF;
  }

  // PID feedback
  double PID_Control(double target_velocity = 0, double measured_velocity = 0) {
    err = (target_velocity - measured_velocity);

    // Note (prev_error - err) has a large difference with changing setpoint, resulting in large Kd spikes
    // while (previos_measurement - current_measured_velocity) means it is difference on measurement, and would eliminate the Kd spike issue
    double pwm_PID = PID_Kp * err + PID_Ki * integral_error + PID_Kd * (previous_measurement - measured_velocity) + PID_BIAS;

    integral_error += err;
    prev_error = err;
    previous_measurement = measured_velocity;  // differentiate on derivative

    return pwm_PID;
  }

  // Combined speed signal
  double setSpeed(double target_velocity = 0, double acceleration = 0) {
    // obtain speed from encoder

    unsigned long currentMillis = millis();

    if (currentMillis - prevMillis >= motor_update_interval) {
      double time_elapsed = (currentMillis - prevMillis) / 1000.0;  // Convert ms to seconds
      prevMillis = currentMillis;

      ////////////////////////// calculate speed (mm) /////////////////////////////////////
      double curr_distance = angle2mm();
      // measured_velocity = (curr_distance - prev_distance) / time_elapsed;  // mm/s
      measured_velocity = lowPassFilter.filt((curr_distance - prev_distance) / time_elapsed);  // mm/s
      prev_distance = curr_distance;

      ////////////////////////// Apply feedforward and PID signal /////////////////////////////////////
      double PWM_signal = 0;
      PWM_signal += feedForward_Control(target_velocity, acceleration);
      PWM_signal += PID_Control(target_velocity, measured_velocity);

      // Serial.print(acceleration);
      // Serial.print(" , ");
      // Serial.print(target_velocity);
      // Serial.print(" , ");
      // Serial.print(measured_velocity);
      // Serial.print(" , ");
      // Serial.print(PWM_signal);
      // Serial.println();

      setMotorPWM(PWM_signal);
    }
    return measured_velocity;
  }


  void resetPID() {
    integral_error = 0;
    prev_error = 0;
    previous_measurement = 0;
  }

  //////////////////////////////////////////////// Velocity profile //////////////////////////////////////////////////////////////////

  MotionParameters calculateTrapezoidalProfile(float Total_distance, float Initialvelocity = 0, float Finalvelocity = 0,float Maxvelocity=MAX_VELOCITY,  float Acceleration = MAX_ACCELERATION, float Deceleration = MAX_ACCELERATION) {
    /*
          This function sets up the Trapezoidal velocity profile by calculating the shape of the graph by using the distance, max_velocity and max_acceleration
          assumption:
          1) acceleration and deceleration is equal (time for accel and decel are the same)
          2) 3 zones are available, Accel Zone, Steady State Zone, Decel Zone
          3) when possible, it will form trapezoidal profile, else it will reach as high velocity as it can before decel (triangle profile)
          4) Start and End velocities are 0

          case 1 (triangle): if distance is less than area needed for both accel and decel
          case 2 (trapezoidal): when enough area for accel and decel, add max velocity period in middle, as steady state zone
      */

    MotionParameters motionParams;
    
    // Determine direction (1 for forward, -1 for backward)
    float direction = (Total_distance >= 0) ? 1.0f : -1.0f;
    float abs_distance = fabs(Total_distance);
    float abs_max_vel = fabs(Maxvelocity) * direction; // Signed max velocity
    
    // Step 1: Acceleration phase
    float Acceleration_time = (abs_max_vel - Initialvelocity) / Acceleration;
    Acceleration_time = fmax(Acceleration_time, 0); // Ensure non-negative
    float Distance_duringacceleration = Initialvelocity * Acceleration_time + 0.5f * Acceleration * Acceleration_time * Acceleration_time;
    
    // Step 2: Deceleration phase
    float Deceleration_time = (abs_max_vel - Finalvelocity) / Deceleration;
    Deceleration_time = fmax(Deceleration_time, 0); // Ensure non-negative
    float Distance_duringdeceleration = abs_max_vel * Deceleration_time - 0.5f * Deceleration * Deceleration_time * Deceleration_time;
    
    // Step 3: Constant velocity phase
    float Remaining_Distance = abs_distance - (Distance_duringacceleration + Distance_duringdeceleration);
    float Constant_Accelerationtime = (Remaining_Distance > 0) ? (Remaining_Distance / fabs(abs_max_vel)) : 0;
    
    float t_start_constant = Acceleration_time;
    float t_end_constant = t_start_constant + Constant_Accelerationtime;
    float t_end_deceleration = t_end_constant + Deceleration_time;
    
    // Save parameters in struct (convert to ms)
    motionParams.ta = Acceleration_time * 1000.0f;
    motionParams.tc = Constant_Accelerationtime * 1000.0f;
    motionParams.tcf = t_end_constant * 1000.0f;
    motionParams.T = t_end_deceleration * 1000.0f;
    
    // Store signed values
    motionParams.Vm = abs_max_vel;
    motionParams.s_req = Total_distance; // Signed distance
    motionParams.velocity = abs_max_vel;
    motionParams.acceleration = Acceleration * direction; // Signed acceleration
    motionParams.deceleration = Deceleration * direction; // Signed deceleration
    
    motionParams.prev_time = millis();
    motionParams.time_step = 0;
    

    // Debug output
    // Serial.print("ta: ");
    // Serial.print(motionParams.ta);
    // Serial.print("  tc: ");
    // Serial.print(motionParams.tc);
    // Serial.print("  tcf: ");
    // Serial.print(motionParams.tcf);
    // Serial.print("  T: ");
    // Serial.print(motionParams.T);
    // Serial.print("  Vm: ");
    // Serial.println(motionParams.Vm);

    return motionParams;
  }


  void followProfile(MotionParameters *motionParams) {
    /*
        This function implements the motion profile, and calculates the velocity and acceleration
        required at each timestep

        The first if statement is there so that we do a control action every fixed amount of seconds
        it enters the if statement every 2ms or watever we set.

        Once the duration of the control is over ie reached the end there is an additional leyway for
        pid to settle any remaining error, but as of now there is no need for such.

        In addition, the motion profile takes into account the maximum the motor can do, so the PID
        wont be requested a "harsh step", rather a gentler slope to follow.
    */
    unsigned long currentTime = millis();
    if ((currentTime - motionParams->prev_time) >= motor_update_interval) {
      motionParams->prev_time = currentTime;

      // Serial.print(motionParams->time_step);
      // Serial.print("  ");
      // Serial.print(motionParams->T*1000);
      // Serial.println("  ");

      if (motionParams->time_step <= motionParams->T) {

        // Acceleration Zone
        if (motionParams->time_step < motionParams->ta) {
          current_timestep_velocity = motionParams->acceleration * (motionParams->time_step / 1000);
          current_timestep_acceleration = motionParams->acceleration;

          // // Overshooting condition
          // } else if (motionParams->Vm <= motionParams->velocity){
          //     current_timestep_velocity = motionParams->Vm - motionParams->acceleration * ((motionParams->time_step/1000) - (motionParams->ta/1000));
          //     current_timestep_acceleration = -motionParams->acceleration;

          // Steady State Zone
        } else if ((motionParams->time_step >= motionParams->ta) && (motionParams->time_step <= motionParams->tcf)) {
          current_timestep_velocity = motionParams->velocity;
          current_timestep_acceleration = 0;

          // Deceleration Zone
        } else if (motionParams->time_step > motionParams->tcf) {
          current_timestep_velocity = motionParams->velocity - ((motionParams->time_step / 1000) - (motionParams->tcf / 1000)) * motionParams->deceleration;
          current_timestep_acceleration = -motionParams->deceleration;
        }

        // Update the control blocks with new control signals.
        motionParams->time_step += motor_update_interval;
        // Serial.print(current_timestep_velocity);
        // Serial.print("  ");
        // Serial.print(current_timestep_acceleration);
        // Serial.println("  ");
        setSpeed(current_timestep_velocity, current_timestep_acceleration);

      } else {
        // To reach here we have passed the full duration of the feed forward action, we can stop the motor now.
        stopMotor();
      }
    }
  }
};
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////



