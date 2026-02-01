#ifndef SRC_DCMOTOR_H_
#define SRC_DCMOTOR_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define DC_FORWARD 		1
#define DC_BACKWARD 	0
#define	GEAR_RATIO		45.0f
#define TICK_P_ROUND	11.0f
#define	DIAMETER		0.004
#define TICK_P_REV		1980

typedef struct {
    GPIO_TypeDef *PORT_M1;
    uint16_t PIN_M1;

    GPIO_TypeDef *PORT_M2;
    uint16_t PIN_M2;

    TIM_HandleTypeDef *pwm_tim;
    uint32_t pwm_channel;

    TIM_HandleTypeDef *encoder_tim;
    int16_t last_counter_value;
} DCmotor;

void DCmotor_setSpeed(DCmotor *m, float percent);
void DCmotor_setDir(DCmotor *m, uint8_t dir);
void DCmotor_Drive(DCmotor *m, float speed); // Helper function
int16_t DCmotor_getEncoder(DCmotor *m);

//void DCmotor_UpdateSpeed(DCmotor *m, float samplingTime_s)
//{
//    // 1. Lấy số xung đếm được trong khoảng thời gian qua
//    // Hàm này của bạn đã tự reset counter về 0 nên giá trị trả về là delta pulse luôn
//    int16_t delta_pulse = DCmotor_getEncoder(m);
//
//    // 2. Tính tổng số xung cho 1 vòng quay trục động cơ (sau hộp giảm tốc)
//    // Chế độ x4 của Timer Encoder
//    float pulses_per_rev = (float)(m->PPR * 4 * m->GearRatio);
//
//    // 3. Tính số vòng quay (Rotations)
//    float rotations = (float)delta_pulse / pulses_per_rev;
//
//    // 4. Tính vận tốc RPM (Rotations Per Minute)
//    // RPM = (Rotations / Time_seconds) * 60
//    float rpm_raw = (rotations / samplingTime_s) * 60.0f;
//
//    // 5. (Tùy chọn) Lọc nhiễu - Low Pass Filter
//    // Vận tốc thực tế thường rất giật, nên dùng lọc để mượt hơn
//    // Hệ số alpha (0 < alpha < 1). Alpha càng nhỏ càng mượt nhưng đáp ứng chậm.
//    float alpha = 0.3f;
//    m->SpeedRPM = (alpha * rpm_raw) + ((1.0f - alpha) * m->SpeedRPM);
//}

#endif
