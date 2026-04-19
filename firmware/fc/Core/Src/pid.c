#include "pid.h"

void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float integral_limit, float output_limit) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->integral_limit = integral_limit;
  pid->output_limit = output_limit;
}

float PID_Update(PID_Controller *pid, float setpoint, float measurement,
                 float gyro_rate, float dt) {
  float error = setpoint - measurement;

  /* Proportional */
  float p = pid->kp * error;

  /* Integral with anti-windup clamping */
  pid->integral += error * dt;
  if (pid->integral > pid->integral_limit)
    pid->integral = pid->integral_limit;
  if (pid->integral < -pid->integral_limit)
    pid->integral = -pid->integral_limit;
  float i = pid->ki * pid->integral;

  /* Derivative on measurement (gyro rate) - avoids derivative kick */
  float d = -pid->kd * gyro_rate;

  float output = p + i + d;

  /* Output clamping */
  if (output > pid->output_limit)
    output = pid->output_limit;
  if (output < -pid->output_limit)
    output = -pid->output_limit;

  pid->prev_error = error;

  return output;
}

void PID_Reset(PID_Controller *pid) {
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
}