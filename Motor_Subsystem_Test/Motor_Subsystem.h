// #include <AS5600.h>  // https://github.com/RobTillaart/AS5600
#include <cmath>
#include <driver/adc.h>
#include <esp_adc_cal.h>

#include "lowPass.h"

// Encoder parameters
#define wheelRadius 24.0
#define MAX_ACCELERATION 500  // mm/s/s
#define MAX_VELOCITY 500     // mm/s

#define encoder_tick_per_rev 4096

// Motor parameter
#define PWMResolution 12
#define PWMResolutionMaxValue 4095

// System speed
#define motor_update_freq 100  // 200Hz -> update time -> 5ms


typedef struct MotionParameters {
  float ta;
  float tc;
  float tcf;
  float T;
  float Vm;
  float s_req;
  float velocity;
  float acceleration;
  float time_step;
  int32_t prev_time;

} MotionParameters;


class MotorControl {
private:
  // Classes used
  LowPass<1> lowPassFilter;

  // Motor & encoder pins
  uint motorPin1;
  uint motorPin2;
  uint motorPWM;
  uint encoderAnalogPin;

  // Direction variables
  int enc_dir_base = 0;  // either 1 or -1

  // System setup
  int motor_update_interval = 0;  // matches motor_update_freq in time interval (updated in setup)

  double integral_error = 0;
  double prev_error = 0;
  double previous_measurement = 0;

  unsigned long prevMillis = 0;
  double prev_distance = 0;

  int encoder_counter = 0;

  long _position = 0;
  long _lastPosition = 0;

  double last_vel = 0;



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

  double err = 0;
  double measured_velocity = 0;

  float current_timestep_velocity = 0;
  float current_timestep_acceleration = 0;

  ////////////////////////////////////////////// Setup ////////////////////////////////////////////////////////////////////
  MotorControl(uint pin1, uint pin2, uint pwm, uint analogPin, int encDirection)
    : motorPin1(pin1),
      motorPin2(pin2),
      motorPWM(pwm),
      encoderAnalogPin(analogPin),

      enc_dir_base(encDirection),
      lowPassFilter(20, motor_update_freq, true) {}

  void setupMotor() {
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    ledcAttach(motorPWM, 5000, PWMResolution);
    stopMotor();
    
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);

    ////////////////////////////////////////////////////////////////////////
    // Feedforward system
    Tm = 0.22 ;
    FF_K_offset = 298;
    FF_K_velocity = 6.72;
    FF_K_accel = FF_K_velocity * (1 - exp(-1 / (Tm* 1000.0)));

    // Feedback system
    PID_BIAS = 0;
    PID_Kp = 10*(16 * Tm) / (FF_K_velocity * (0.7071 * 0.7071) * (5 * Tm) * (5 * Tm));
    PID_Ki = 0;
    PID_Kd = 0.1 * (8 * Tm - (5 * Tm)) / ((5 * Tm) * FF_K_velocity);

    ////////////////////////////////////////////////////////////////////////
    motor_update_interval = 1000 / motor_update_freq;  //in ms
    PID_Kd = PID_Kd * motor_update_freq;  // TODO need to see if this is needed or not
  }
  //////////////////////////////////////////////// Encoder feedback //////////////////////////////////////////////////////////////////
  // Encoder distance
  /////////////////// Library cummulative calculation ///////////////
  double updateEncoder() {

    // int16_t value = analogRead(encoderAnalogPin);
    int16_t value = adc1_get_raw(ADC1_CHANNEL_6);
    
    if (enc_dir_base == -1) value = PWMResolutionMaxValue - value;  //invert encoder reading if needed
    //  whole rotation CW?
    //  less than half a circle
    if ((_lastPosition > 2048) && (value < (_lastPosition - 2048))) {
      _position = _position + 4096 - _lastPosition + value;
    }
    //  whole rotation CCW?
    //  less than half a circle
    else if ((value > 2048) && (_lastPosition < (value - 2048))) {
      _position = _position - 4096 - _lastPosition + value;
    } else {
      _position = _position - _lastPosition + value;
    }
    _lastPosition = value;

    return _position;
  }

  double angle2mm() {
    //////// return in RPM ////////
    // return 60*((double)updateEncoder()) / encoder_tick_per_rev;

    //////// return in mm  ////////
    return (2 * M_PI * wheelRadius * (double)updateEncoder()) / encoder_tick_per_rev;
  }

  // //////////////////////////////////////////////// Motor control //////////////////////////////////////////////////////////////////
  // // Write PWM speed to motor
  void setMotorPWM(int speed) {
    speed = constrain(speed, -PWMResolutionMaxValue, PWMResolutionMaxValue);
    if (speed > 0) {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
      ledcWrite(motorPWM, abs(speed));
    } else if (speed < 0) {
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
      ledcWrite(motorPWM, abs(speed));
    } else stopMotor();
  }

  void stopMotor() {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, HIGH);
    ledcWrite(motorPWM, 0);
  }

  /////////////////////// Motor RPM control /////////////////////////////////////////////////////////////////
  // Feedforward - https://youtu.be/qKoPRacXk9Q?si=ahXkdiADK6ndN237
  double feedForward_Control(double velocity, double acceleration) {
    double K_off =0;
    if(abs(velocity) < 0) K_off = -FF_K_offset;
    else K_off = FF_K_offset;
    
    double pwm_FF = K_off + FF_K_velocity * velocity + FF_K_accel * acceleration;
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
  void setSpeed(double target_velocity = 0, double acceleration = 0) {
    unsigned long currentMillis = millis();

    if (currentMillis - prevMillis >= motor_update_interval) {

      ////////////////////////// distance (mm) /////////////////////////////////////
      double curr_distance = angle2mm();
      double time_elapsed = (currentMillis - prevMillis) / 1000.0;  // Convert ms to seconds

      measured_velocity = (curr_distance - prev_distance) / time_elapsed;  // mm/s
      // measured_velocity = lowPassFilter.filt((curr_distance - prev_distance) / time_elapsed);  // mm/s

      // filter if encoder hits noisy data (too low speed or too high speeds)
      // if (abs(measured_velocity) < 5) measured_velocity = last_vel;
      // else if (abs(measured_velocity) > abs((MAX_VELOCITY * 1.2))) measured_velocity = last_vel;
      // else last_vel = measured_velocity;

      //////////////////////////////////////////////////////////////////////////////
      // Serial.print("time: ");
      // Serial.print(currentMillis);
      // Serial.print("prev: ");
      // Serial.print(prevMillis);

      // Serial.print("curr: ");
      // Serial.print(curr_distance);
      // Serial.print("prev: ");
      // Serial.print(prev_distance);

      // Serial.print("  Speed: ");
      // Serial.println(measured_velocity);

      prevMillis = currentMillis;
      prev_distance = curr_distance;



      // Apply feedforward and PID signal
      double PWM_signal = 0;
      PWM_signal += feedForward_Control(target_velocity, acceleration);
      PWM_signal += PID_Control(target_velocity, measured_velocity);

      Serial.print(" ");
      Serial.print(currentMillis);
      // Serial.print(" ");
      // Serial.print(angle2mm());
      // Serial.print(" ");
      // Serial.print(angle2mm());
      // Serial.println();

      Serial.print(" ");
      Serial.print(target_velocity);
      Serial.print(" ");
      Serial.print(measured_velocity);

      // Serial.print(" ");
      // Serial.print(PID_Kp);
      // Serial.print(" ");
      // Serial.print(PID_Ki);
      // Serial.print(" ");
      // Serial.print(PID_Kd);

      // Serial.print(" ");
      // Serial.print(feedForward_Control(target_velocity, acceleration));
      // Serial.print(" ");
      // Serial.print(PWM_signal);
      Serial.println();

      setMotorPWM(PWM_signal);
      // setMotorPWM(1000);
    }
  }


  void resetPID() {
    integral_error = 0;
    prev_error = 0;
    previous_measurement = 0;
  }

  //////////////////////////////////////////////// Velocity profile //////////////////////////////////////////////////////////////////

  MotionParameters calculateTrapezoidalProfile(float distance, float max_velocity, float max_acceleration) {
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
    motionParams.s_req = distance;
    if (max_acceleration > MAX_ACCELERATION)
      max_acceleration = MAX_ACCELERATION;
    if (max_velocity > MAX_VELOCITY)
      max_velocity = MAX_VELOCITY;

    motionParams.velocity = max_velocity;
    motionParams.acceleration = max_acceleration;

    motionParams.Vm = sqrt(motionParams.s_req * max_acceleration);
    if (motionParams.Vm <= max_velocity) {
      motionParams.ta = motionParams.Vm / max_acceleration;
      motionParams.T = 2 * motionParams.ta;
      motionParams.tc = 0;
      motionParams.tcf = 0;
    } else {
      motionParams.ta = max_velocity / max_acceleration;
      motionParams.tc = motionParams.s_req / max_velocity - motionParams.ta;
      motionParams.T = 2 * motionParams.ta + motionParams.tc;
      motionParams.tcf = motionParams.T - motionParams.ta;
    }

    // Convert to ms, since we use millis() function to compare time
    motionParams.T *= 1000;
    motionParams.ta *= 1000;
    motionParams.tc *= 1000;
    motionParams.tcf *= 1000;

    // Serial.print(motionParams.velocity);
    // Serial.print("  ");
    // Serial.print(motionParams.acceleration);
    // Serial.print("  ");
    // Serial.print(motionParams.T);
    // Serial.print("  ");
    // Serial.print(motionParams.ta);
    // Serial.print("  ");
    // Serial.print(motionParams.tc);
    // Serial.print("  ");
    // Serial.print(motionParams.Vm);
    // Serial.println("  ");

    // motionParams.velocity = max_velocity;
    // motionParams.acceleration = max_acceleration;

    motionParams.prev_time = millis();
    motionParams.time_step = 0;
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

          // Steady State Zone
        } else if ((motionParams->time_step >= motionParams->ta) && (motionParams->time_step <= motionParams->tcf)) {
          current_timestep_velocity = motionParams->velocity;
          current_timestep_acceleration = 0;

          // Deceleration Zone
        } else if (motionParams->time_step > motionParams->tcf) {
          current_timestep_velocity = motionParams->velocity - ((motionParams->time_step / 1000) - (motionParams->tcf / 1000)) * motionParams->acceleration;
          current_timestep_acceleration = -motionParams->acceleration;
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


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Testing functions to generate step response and Kv value
  void generateStepAtPWM(int pwm) {
    setMotorPWM(pwm);

    static unsigned long startMillis = 0;
    static unsigned long prevMillis = 0;
    startMillis = millis();
    unsigned long currentMillis = millis();
    do {
      currentMillis = millis();
      if ((currentMillis - prevMillis) >= 2) {
        Serial.print(" ");
        Serial.print(currentMillis);
        Serial.print(" ");
        Serial.print(angle2mm());
        Serial.print(" ");
        Serial.print(angle2mm());
        Serial.println();

        prevMillis = currentMillis;
      }

    } while ((currentMillis - startMillis) <= 2000);
  }
  void generateStepResponse() {
    Serial.println("Start");
    resetPID();

    generateStepAtPWM(0);
    generateStepAtPWM(4095);
    generateStepAtPWM(0);
    generateStepAtPWM(4095);
    generateStepAtPWM(0);
    // generateStepAtPWM(4095);
    // generateStepAtPWM(0);
    // generateStepAtPWM(4095);
    // generateStepAtPWM(0);

    // generateStepAtPWM(4095);
    // generateStepAtPWM(3072);
    // generateStepAtPWM(2048);
    // generateStepAtPWM(1024);
    // generateStepAtPWM(0);

    setMotorPWM(0);
    Serial.println("End");
  }
};

