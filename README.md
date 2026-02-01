# STM32 Self-Balancing Robot 🤖
![Status](https://img.shields.io/badge/Status-Completed-success)
![MCU](https://img.shields.io/badge/MCU-STM32F401CCU6-blue)
![Hardware](https://img.shields.io/badge/Hardware-MPU6050_%7C_TB6612FNG-orange)
![Language](https://img.shields.io/badge/Language-C-8A2BE2)
![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-red)

**Course:** Embedded System Design (CE224.Q12)

**Institution:** University of Information Technology (UIT) - VNU-HCM

**Instructor:** Mr. Tran Ngoc Duc

## 📖 Overview

This project implements a **Two-Wheeled Self-Balancing Robot** based on the Inverted Pendulum mathematical model. The system is powered by the **STM32F401CCU6** microcontroller and utilizes **FreeRTOS** for real-time multitasking.

The control system features a **Cascade PID algorithm** (Velocity & Angle loops) and a **Complementary Filter** for sensor fusion, ensuring stability and smooth operation. This project was developed for the **Embedded System Design (CE224.Q12)** course at the **University of Information Technology - VNU-HCM**.

🔗 **Full Project Report:** [https://drive.google.com/file/d/1FqUx3Trcz4oJ_H03-fAWJpXKTJU6ttZQ/view?usp=drive_link]

## 🛠 Tech Stack

### Hardware
* **Microcontroller:** STM32F401CCU6 (Blackpill - Cortex M4).
* **IMU Sensor:** MPU6050 (Accelerometer + Gyroscope).
* **Motor Driver:** TB6612FNG.
* **Actuators:** 2x GA25-370 DC Gear Motors (with Quadrature Encoders).
* **Power:** 3S Li-ion Battery (11.1V - 12.6V) + LM2596 Buck Converter (5V/3.3V).

### Software & Tools
* **IDE:** STM32CubeIDE.
* **Config Tool:** STM32CubeMX.
* **OS:** FreeRTOS (Real-time Operating System).
* **Language:** C, ARM Assembly (Startup code).
* **Debugging:** USB Virtual COM Port (CDC).

## ✨ Key Features

* **Real-Time OS:** Utilizes FreeRTOS to manage tasks (Sensor Reading, PID Calculation, Debugging) with hardware timer interrupts for precise sampling (1ms).
* **Cascade PID Control:**
    * **Inner Loop (Angle):** Maintains upright stability using "Derivative on Measurement" to eliminate "Derivative Kick."
    * **Outer Loop (Speed):** Controls position and velocity.
* **Sensor Fusion:** Implements a Complementary Filter ($\alpha=0.97$) to combine Accelerometer stability with Gyroscope responsiveness.
* **Anti-Windup:** PID integral term saturation to prevent overshoot.
* **Hardware Abstraction:** Optimized use of STM32 HAL and Low-Level Direct Register Access (Macros) for PWM generation.

## 🔌 Pin Configuration

| Component | Signal | STM32 Pin | Timer/Function |
| :--- | :--- | :--- | :--- |
| **MPU6050** | SCL | `PB6` | I2C1 |
| | SDA | `PB7` | I2C1 |
| **Left Motor** | PWM | `PA8` | TIM1_CH1 |
| | DIR | `PB3` | GPIO |
| | ENC A | `PB14` | TIM3_CH1 |
| | ENC B | `PB15` | TIM3_CH2 |
| **Right Motor** | PWM | `PA9` | TIM1_CH2 |
| | DIR | `PA3` | GPIO |
| | ENC A | `PB13` | TIM4_CH1 |
| | ENC B | `PB12` | TIM4_CH2 |
| **System** | LED | `PC13` | Status Indicator |

## 🚀 Installation & Setup

1.  **Clone the Repository:**
    ```bash
    git clone [https://github.com/NguyenDinhNhatNguyen/Two-Wheeled-Self-Balancing-Robot]
    ```
2.  **Hardware Setup:**
    * Connect the components according to the Pin Configuration table above.
    * Ensure the MPU6050 is mounted flat and secure.
3.  **Open in STM32CubeIDE:**
    * File -> Open Projects from File System -> Select the cloned folder.
4.  **Build & Flash:**
    * Connect the STM32F4 via ST-Link.
    * Click **Run** (Green Play Button).
5.  **Calibration:**
    * Hold the robot upright (static) for the first 3 seconds after boot to allow Gyroscope Offset calibration.

## 📊 Control Logic (PID)

The system uses a custom PID implementation:
$$Output = K_p \times e(t) + K_i \times \int e(t)dt - K_d \times Gyro_{filtered}$$

* **Proportional (P):** Corrects error immediately.
* **Integral (I):** Corrects steady-state error (Speed loop).
* **Derivative (D):** Uses raw Gyroscope data instead of Error Derivative to dampen oscillations and prevent spikes when the setpoint changes.

## 👥 Contributors

* **Nguyen Viet Thien Nhan** (23521086) - PID Algorithm, Motor Driver, Hardware Design.
* **Nguyen Dinh Nhat Nguyen** (23521043) - MPU6050 Driver, System Config (I2C, Timer), Hardware Assembly.

---
*Ho Chi Minh City, December 2025*
