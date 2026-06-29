/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F103C8T6 Line Tracking Car - Final Optimized
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PD */

/* 1. ???? */
#define LEFT_MOTOR_DIR        1
#define RIGHT_MOTOR_DIR       1

/* 2. PWM ???? (????) */
#define PWM_MAX               800
#define PWM_MIN_LIMIT         75    // ?? ?????????????

#define SPEED_BASE            180 //???)
#define SPEED_CORRECT_FAST    250
#define SPEED_CORRECT_SLOW    -50//(???????)
#define SPEED_BLIND_WALK      100   // ????????

/* USER CODE END PD */

int left_pwm_cmd = 0;
int right_pwm_cmd = 0;
int left_pwm_output = 0;
int right_pwm_output = 0;
uint8_t sensor_left_val = 0;
uint8_t sensor_right_val = 0;

/* Function Prototypes */
void Motor_Set(int left_pwm, int right_pwm);
int Limit_PWM(int value, int max);
int Ramp_PWM(int target_pwm, int *current_pwm);
void Line_Tracking_Control(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_I2C1_Init(); 

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    Motor_Set(0, 0);
    HAL_Delay(1000); 

    while (1)
    {
        Line_Tracking_Control();
        
        /* ??????? */
        if (left_pwm_cmd > 0 && left_pwm_cmd < PWM_MIN_LIMIT) left_pwm_cmd = PWM_MIN_LIMIT;
        if (right_pwm_cmd > 0 && right_pwm_cmd < PWM_MIN_LIMIT) right_pwm_cmd = PWM_MIN_LIMIT;

        left_pwm_cmd = Ramp_PWM(left_pwm_cmd, &left_pwm_output);
        right_pwm_cmd = Ramp_PWM(right_pwm_cmd, &right_pwm_output);

        Motor_Set(left_pwm_cmd, right_pwm_cmd);
        HAL_Delay(5);
    }
}

void Line_Tracking_Control(void)
{
    /* PB0 = ?, PB1 = ? */
    sensor_left_val  = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
    sensor_right_val = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);

    if (sensor_left_val == 0 && sensor_right_val == 0)
    {
        left_pwm_cmd  = SPEED_BASE;
        right_pwm_cmd = SPEED_BASE;
    }
    else if (sensor_left_val == 1 && sensor_right_val == 0)
    {
        left_pwm_cmd  = SPEED_CORRECT_FAST;
        right_pwm_cmd = SPEED_CORRECT_SLOW;
    }
    else if (sensor_left_val == 0 && sensor_right_val == 1)
    {
        left_pwm_cmd  = SPEED_CORRECT_SLOW;
        right_pwm_cmd = SPEED_CORRECT_FAST;
    }
    else
    {
        left_pwm_cmd  = SPEED_BLIND_WALK;
        right_pwm_cmd = SPEED_BLIND_WALK;
    }
}

void Motor_Set(int left_pwm, int right_pwm)
{
    left_pwm = Limit_PWM(left_pwm * LEFT_MOTOR_DIR, PWM_MAX);
    right_pwm = Limit_PWM(right_pwm * RIGHT_MOTOR_DIR, PWM_MAX);

    /* ?? PB13/PB12 */
    if (left_pwm > 0) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); }
    else if (left_pwm < 0) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); }
    else { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); }
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, abs(left_pwm));

    /* ?? PB14/PB15 */
    if (right_pwm > 0) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); }
    else if (right_pwm < 0) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); }
    else { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); }
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, abs(right_pwm));
}

int Limit_PWM(int value, int max) { return (value > max) ? max : (value < -max ? -max : value); }

int Ramp_PWM(int target_pwm, int *current_pwm)
{
    int step = 4; // ?????,????
    if (target_pwm > *current_pwm + step) *current_pwm += step;
    else if (target_pwm < *current_pwm - step) *current_pwm -= step;
    else *current_pwm = target_pwm;
    return *current_pwm;
}

void SystemClock_Config(void) { /* ...????... */ }
void Error_Handler(void) { while(1); }