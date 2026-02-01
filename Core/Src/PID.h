#ifndef PID_H
#define PID_H
#include "mpu6050.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Kff;
} PIDParam;

typedef struct {
    float prevError;
    float integral;
    float prevDerivative;
} PIDError;

float PID_calculate(PIDParam *k, PIDError *err, float setpoint, float current, float dt);
float PID_motor_calculate(PIDParam *k, PIDError *err, float setpoint, float current, float dt);
float PID_angle_calculate(PIDParam *k, PIDError *err, float setpoint, float current, MPUData *mpudata, float dt);
#endif
