/*
 * DCmotor.c
 *
 *  Created on: Dec 5, 2025
 *      Author: LENOVO
 */

#include "DCmotor.h"


// ------------------------------------------------------
// Set PWM duty (0–100%)
// ------------------------------------------------------
void DCmotor_setSpeed(DCmotor *m, float percent)
{
    if (percent > 100.0f) percent = 100.0f;
    if (percent < 0.0f)   percent = 0.0f;

    // Giả sử động cơ cần ít nhất 15% PWM để bắt đầu quay (cần thực nghiệm)
    float min_duty =3.0f;

    uint32_t arr = m->pwm_tim->Init.Period;
    uint32_t pulse = 0;

    if (percent > 0.0f)
    {
        // Map từ 0-100% thành min_duty-100%
//        float output_curve = min_duty + (percent * (100.0f - min_duty) / 100.0f);
        pulse = (uint32_t)((percent * arr) / 100.0f);
    }
    else
    {
        pulse = 0;
    }

    __HAL_TIM_SET_COMPARE(m->pwm_tim, m->pwm_channel, pulse);
}

// ------------------------------------------------------
// Set direction pins
// ------------------------------------------------------
void DCmotor_setDir(DCmotor *m, uint8_t dir)
{
    if (dir == DC_FORWARD)
    {
        HAL_GPIO_WritePin(m->PORT_M1, m->PIN_M1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->PORT_M2, m->PIN_M2, GPIO_PIN_RESET);
    }
    else // DC_BACKWARD
    {
        HAL_GPIO_WritePin(m->PORT_M1, m->PIN_M1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->PORT_M2, m->PIN_M2, GPIO_PIN_SET);
    }
}


// ------------------------------------------------------
// Drive motor with signed speed (PID output)
// speed >= 0 → forward
// speed < 0 → backward
// ------------------------------------------------------
void DCmotor_Drive(DCmotor *m, float pwm_duty)
{
    // Kẹp giá trị trong khoảng -100 đến 100
    if (pwm_duty > 100.0f) pwm_duty = 100.0f;
    if (pwm_duty < -100.0f) pwm_duty = -100.0f;

    if (pwm_duty >= 0.0f)
    {
        DCmotor_setDir(m, DC_FORWARD);
        DCmotor_setSpeed(m, pwm_duty);
    }
    else
    {
        DCmotor_setDir(m, DC_BACKWARD);
        DCmotor_setSpeed(m, -pwm_duty); // Lấy giá trị dương cho PWM
    }
}

// ------------------------------------------------------
// Read encoder counter (TIMx CNT register)
// ------------------------------------------------------
int16_t DCmotor_getEncoder(DCmotor *m)
{
	int16_t current_cnt = (int16_t)__HAL_TIM_GET_COUNTER(m->encoder_tim);
    int16_t delta = current_cnt - m->last_counter_value;
    m->last_counter_value = current_cnt;
    // Xử lý tràn số (Overflow/Underflow) tự động nhờ tính chất số nguyên int16_t
    // Ví dụ: curr = -32000, last = 32000 -> diff vẫn tính đúng khoảng cách

    return delta;
}
