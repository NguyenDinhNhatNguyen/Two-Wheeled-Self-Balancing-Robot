#include "pid.h"
#define D_ALPHA  0.2f

#include <math.h> // Đảm bảo đã include thư viện này

// Biến lưu giá trị lọc cũ (Nên để trong struct nếu muốn chuyên nghiệp, nhưng để đây cho nhanh)
static float last_gyro_filtered = 0.0f;

float PID_angle_calculate(PIDParam *k, PIDError *err, float setpoint, float current, MPUData *mpudata, float dt)
{
    float P, D;
    float error = setpoint - current;
    err->prevError = error;

    // --- 1. SỬA LỖI TRỄ PHA ---
    // Bỏ thành phần phi tuyến (error * abs(error)).
    // P tuyến tính giúp robot phản ứng tức thì ngay khi vừa chớm nghiêng.
    P = k->Kp * error;

    // --- 2. SỬA LỖI RUNG KHI TĂNG KD ---
    // Lọc nhiễu Gyro trước khi đưa vào tính D.
    // Alpha = 0.3 nghĩa là lấy 30% giá trị mới, 70% giá trị cũ (Lọc khá mạnh).
    // Nếu vẫn rung thì giảm xuống 0.2 hoặc 0.1. Nếu bị trễ thì tăng lên 0.5.
    float alpha = 0.3f;
    float current_gyro = mpudata->GYRO_X;

    // Low Pass Filter
    float gyro_filtered = alpha * current_gyro + (1.0f - alpha) * last_gyro_filtered;
    last_gyro_filtered = gyro_filtered;

    // Tính D dựa trên Gyro SẠCH
    D = -k->Kd * gyro_filtered;
    err->prevDerivative = D;

    // Tổng hợp
    float output = P + D;
    return output;
}

float PID_motor_calculate(PIDParam *k, PIDError *err, float setpoint, float current, float dt)
{
    float error = setpoint - current;
    float ff = k->Kff * setpoint;
    // Proportional
    float P = k->Kp * error;

    // Integral
    err->integral += error * dt;

    // Anti wind-up
    if (err->integral > 100) err->integral = 100;
    if (err->integral < -100) err->integral = -100;

    float I = k->Ki * err->integral;

    // Derivative
    if (dt < 0.0001f) dt = 0.0001f;

    float derivative = (error - err->prevError) / dt;
    float D = k->Kd * derivative;

    err->prevError = error;

    // Raw output
    float output = P + I + D + ff;

    // Clamp to ±100%
    if (output > 100.0f) output = 100.0f;
    if (output < -100.0f) output = -100.0f;

    return output;
}
