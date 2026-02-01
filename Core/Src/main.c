/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "pid.h"
#include "DCmotor.h"
#include "stdio.h"
#include "usbd_cdc_if.h"
#include "send_data.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAD_TO_DEG 			57.2957795
#define	DELTA				5.0f
#define MPU_DELTA			0.5f
#define MPU_DT				((float)MPU_DELTA / 1000.0)
#define DT_SECONDS 			0.005f
#define MAX_SPEED			150U
// ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

osThreadId defaultTaskHandle;
osThreadId ControlTaskHandle;
osThreadId MPUTaskHandle;
/* USER CODE BEGIN PV */

DCmotor motor1 = {
    .PORT_M1 = A1_GPIO_Port, .PIN_M1 = A1_Pin,
    .PORT_M2 = A2_GPIO_Port, .PIN_M2 = A2_Pin,
    .pwm_tim = &htim1, .pwm_channel = TIM_CHANNEL_1,
    .encoder_tim = &htim3
};
DCmotor motor2 = {
    .PORT_M1 = B1_GPIO_Port, .PIN_M1 = B1_Pin,
    .PORT_M2 = B2_GPIO_Port, .PIN_M2 = B2_Pin,
    .pwm_tim = &htim1, .pwm_channel = TIM_CHANNEL_2,
    .encoder_tim = &htim4
};
//extern USBD_HandleTypeDef hUsbDeviceFS;
volatile int speed = 0, delta = 0;
volatile uint_fast8_t update_log = 0;
MPUData mpudata;
static float_t pre_angle = 0.0f, setpoint = 0.0f, setpoint_physical = 4.8f;
float alpha[2] = {0.998f, 0.25f};

static float filt_rpm1 	= 0.0f,
			 filt_rpm2 	= 0.0f,
			 filt_angle = 0.0f;

PIDParam Ks = {
    .Kp = 0.02f,
    .Ki = 0.00f,
    .Kd = 0.0f,
	.Kff = 0.0f
};
PIDParam Ka = {15.68, 0.0, 0.4015};

PIDError angle_error1 = {0};
PIDError speed_error1 = {0};
PIDError speed_error2 = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void StartControlTask(void const * argument);
void StartMPUTask(void const * argument);

/* USER CODE BEGIN PFP */
void Robo_Init();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOC,LED_BLUE_Pin, GPIO_PIN_SET);
  MPU6050_Init(&hi2c1);
  MX_USB_DEVICE_Init();
  // Initialize angle
  MPU6050_Read_Data(&hi2c1, &mpudata);
  Robo_Init();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ControlTask */
  osThreadDef(ControlTask, StartControlTask, osPriorityNormal, 0, 512);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* definition and creation of MPUTask */
  osThreadDef(MPUTask, StartMPUTask, osPriorityRealtime, 0, 512);
  MPUTaskHandle = osThreadCreate(osThread(MPUTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
//	  if (update_log) {
//		  update_log = 0;
//		  MPU6050_Read_Data(&hi2c1, &mpudata);
//
////		  uint32_t previousWakeTime = osKernelSysTick();
////		  PID_motor_calculate(&Ka, &angle_error1, setpoint, filt_angle, dt_seconds);
////		  speed = 10; // 50 RPM
//		  // 3. Tính toán tốc độ (RPM)
//		  int32_t delta_tick1 = DCmotor_getEncoder(&motor1);
//		  int32_t delta_tick2 = DCmotor_getEncoder(&motor2);
//		  //////	      delta = delta_tick1 ;
//		  ////	      // Kiểm tra TICK_P_REV đã define chưa
//		  float current_rpm1 = ((float)delta_tick1 / TICK_P_REV) * (60000.0f / DELTA);
//		  float current_rpm2 = ((float)delta_tick2 / TICK_P_REV) * (60000.0f / DELTA);
//		  //	      speed = -1;
//		  ////	      // Lọc thông thấp cho RPM
//		  filt_rpm1 = (alpha[1] * current_rpm1) + ((1.0f - alpha[1]) * filt_rpm1);
//		  filt_rpm2 = (alpha[1] * current_rpm2) + ((1.0f - alpha[1]) * filt_rpm2);
//		  ////	      // 4. Tính PID điều khiển động cơ
//		  float pwm_req1 = PID_motor_calculate(&Ks, &speed_error1, (float)speed, filt_rpm1, dt_seconds);
//		  float pwm_req2 = PID_motor_calculate(&Ks, &speed_error2, (float)speed, filt_rpm2, dt_seconds);
//		  //	      pwm_req1 = 50; pwm_req2 = 20;
//		  DCmotor_Drive(&motor1, pwm_req1);
//		  DCmotor_Drive(&motor2, pwm_req2);
//		  static char tx_buf[CDC_DATA_FS_MAX_PACKET_SIZE];
////		   In ra: Target, Raw1, Filtered1
//		  sprintf(tx_buf, "%d,%.2f,%.2f\r\n", speed, filt_rpm2, filt_rpm1);
//		  CDC_Transmit_FS((uint8_t*)tx_buf, strlen(tx_buf));
//	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 41999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR_B_Pin|DIR_BA4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, B2_Pin|B1_Pin|A1_Pin|A2_Pin
                          |DIR_A_Pin|DIR_AB4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BLUE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_B_Pin DIR_BA4_Pin */
  GPIO_InitStruct.Pin = DIR_B_Pin|DIR_BA4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : B2_Pin B1_Pin A1_Pin A2_Pin
                           DIR_A_Pin DIR_AB4_Pin */
  GPIO_InitStruct.Pin = B2_Pin|B1_Pin|A1_Pin|A2_Pin
                          |DIR_A_Pin|DIR_AB4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Robo_Init()
{
	HAL_GPIO_WritePin(GPIOC, LED_BLUE_Pin, GPIO_PIN_RESET); // Bật đèn báo đang calib
	HAL_GPIO_WritePin(motor1.PORT_M1, motor1.PIN_M1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor1.PORT_M2, motor1.PIN_M2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor2.PORT_M1, motor2.PIN_M1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor2.PORT_M2, motor2.PIN_M2, GPIO_PIN_SET);

	mpudata.Accel_Y_Offset = 0;
	mpudata.Accel_Y_RAW    = 0;
	mpudata.Accel_Z_Offset = 0;
	mpudata.Accel_Z_RAW	 = 0;
	mpudata.GYRO_X		 = 0;
//	setpoint_physical = MPU6050_Calib(&hi2c1, &mpudata); // Robot phải đứng yên thẳng đứng lúc này

	pre_angle = setpoint_physical;
//	setpoint_physical = 4.0f;
	angle_error1.prevDerivative =0;
	angle_error1.integral = 0;
	angle_error1.prevError = 0;
	speed_error1.integral = 0;
	speed_error2.prevDerivative = 0;
	speed_error2.prevError = 0;
	speed_error2.integral = 0;
	motor1.last_counter_value = (int16_t)__HAL_TIM_GET_COUNTER(motor1.encoder_tim);
	motor2.last_counter_value = (int16_t)__HAL_TIM_GET_COUNTER(motor2.encoder_tim);
	HAL_GPIO_WritePin(GPIOC, LED_BLUE_Pin, GPIO_PIN_SET); // Tắt đèn báo xong
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  HAL_TIM_Base_Start_IT(&htim2);
  /* Infinite loop */
  for(;;)
  {
	  osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlTask */
void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */
	 float speed_target = 0.0f; // 0
	 float speed_output = 0.0f;
  /* Infinite loop */
  for(;;)
  {
	  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//	  MPU6050_Read_Data(&hi2c1, &mpudata);

	  //		  uint32_t previousWakeTime = osKernelSysTick();
	  //		  PID_motor_calculate(&Ka, &angle_error1, setpoint, filt_angle, dt_seconds);
	  //		  speed = 10; // 50 RPM
	  		  // 3. Tính toán tốc độ (RPM)
	  int32_t delta_tick1 = DCmotor_getEncoder(&motor1);
	  int32_t delta_tick2 = DCmotor_getEncoder(&motor2);
	  		  //////	      delta = delta_tick1 ;
	  		  ////	      // Kiểm tra TICK_P_REV đã define chưa
	  float current_rpm1 = ((float)delta_tick1 / TICK_P_REV) * (60000.0f / DELTA);
	  float current_rpm2 = ((float)delta_tick2 / TICK_P_REV) * (60000.0f / DELTA);
	  		  //	      speed = -1;
	  		  ////	      // Lọc thông thấp cho RPM
	  filt_rpm1 = (alpha[1] * current_rpm1) + ((1.0f - alpha[1]) * filt_rpm1);
	  filt_rpm2 = (alpha[1] * current_rpm2) + ((1.0f - alpha[1]) * filt_rpm2);
// 4. Tính PID điều khiển động cơ

	  float avg_speed = (filt_rpm1 + filt_rpm2) / 2.0f;
//	  float pwm_req1 = PID_motor_calculate(&Ks, &speed_error1, (float)speed, filt_rpm1, dt_seconds);
//	  float pwm_req2 = PID_motor_calculate(&Ks, &speed_error2, (float)speed, filt_rpm2, dt_seconds);
	  speed_output = PID_motor_calculate(&Ks, &speed_error1, speed_target, avg_speed, DT_SECONDS);

	  if (speed_output > 5.0f) speed_output = 5.0f;
	  if (speed_output < -5.0f) speed_output = -5.0f;

	  setpoint =  setpoint_physical + speed_output;
//	  pwm_req1 = 50; pwm_req2 = 20;
//	  DCmotor_Drive(&motor1, pwm_req1);
//	  DCmotor_Drive(&motor1, speed);
//	  DCmotor_Drive(&motor2, speed);
//	  DCmotor_Drive(&motor2, pwm_req2);
//	  static char tx_buf[CDC_DATA_FS_MAX_PACKET_SIZE];
//	  //		   In ra: Target, Raw1, Filtered1
//	  sprintf(tx_buf, "%.2f,%.2f\r\n",setpoint, filt_angle);
//	  CDC_Transmit_FS((uint8_t*)tx_buf, strlen(tx_buf));
  }
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartMPUTask */
/**
* @brief Function implementing the MPUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMPUTask */
void StartMPUTask(void const * argument)
{
  /* USER CODE BEGIN StartMPUTask */
  float accel_angle;
  float pwm;
  /* Infinite loop */
  for(;;)
  {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      MPU6050_Read_Data(&hi2c1, &mpudata);
      accel_angle = atan2(mpudata.Accel_Y_RAW, mpudata.Accel_Z_RAW) * RAD_TO_DEG;
	  filt_angle = alpha[0] * (filt_angle + mpudata.GYRO_X * MPU_DT) + (1.0f - alpha[0]) * accel_angle;

	  pwm = PID_angle_calculate(&Ka,
	                                 &angle_error1,
	                                 setpoint,
	                                 filt_angle,
	                                 &mpudata,
	                                 MPU_DT);
	  if (filt_angle > 45.0f || filt_angle < -45.0f) {
		  DCmotor_Drive(&motor1, 0);
		  DCmotor_Drive(&motor2, 0);
	  }
	  if (pwm > 100) pwm = 100;
	  else if (pwm < -100) pwm = -100;

	  DCmotor_Drive(&motor1, -pwm);
//	  DCmotor_Drive(&motor1, speed);
//	  DCmotor_Drive(&motor2, speed);
	  DCmotor_Drive(&motor2, -pwm);
  }
  /* USER CODE END StartMPUTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM9 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
		if (htim->Instance == TIM2)
		{
			static uint8_t tick_div = 0;
			tick_div++;

			BaseType_t hpw = pdFALSE;

			// Notify MPU Task (1kHz)
			vTaskNotifyGiveFromISR(MPUTaskHandle, &hpw);

			// Notify Control Task (10Hz - Every 10 ticks)
			if (tick_div >= 10)
			{
				tick_div = 0;
				vTaskNotifyGiveFromISR(ControlTaskHandle, &hpw);
			}

			portYIELD_FROM_ISR(hpw);
		}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM9)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
