#ifndef PID_H
#define PID_H

typedef struct {
  float kp;
  float ki;
  float kd;

  float integral;
  float prev_error;

  float integral_limit;
  float output_limit;
} PID_Controller;

void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float integral_limit, float output_limit);

float PID_Update(PID_Controller *pid, float setpoint, float measurement,
                 float gyro_rate, float dt);

void PID_Reset(PID_Controller *pid);

#endif /* PID_H */