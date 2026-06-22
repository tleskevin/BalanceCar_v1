/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F103C8T6 Balance Car - PID Angle Version
  *
  * 功能：
  * 1. MPU6050 讀取角度
  * 2. 開機校正直立角
  * 3. 開機校正 GyroX 零點
  * 4. 修正 SENSOR_DIR
  * 5. 角度 PID：KP + KI + KD
  * 6. KI 積分限制，避免暴衝
  * 7. 小角度立即做動
  * 8. 防過衝輸出限制
  * 9. TB6612 馬達控制
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

#include <math.h>
#include <stdlib.h>

void SystemClock_Config(void);

/* USER CODE BEGIN PD */

/* =========================================================
   方向設定
   ========================================================= */

#define LEFT_MOTOR_DIR        1
#define RIGHT_MOTOR_DIR       1

/*
   感測方向。
   你目前已經把重心方向調對，所以保持 1。
*/
#define SENSOR_DIR            1

/*
   平衡輸出方向。
   重心往前倒，輪子要往前。
   重心往後倒，輪子要往後。
*/
#define BALANCE_DIR           1

/*
   Gyro 阻尼方向。
   如果輪子方向正確，但車子越救越甩，再改成 1。
*/
#define GYRO_DIR              -1

/*
   重要：
   你前面說會往前衝，所以這裡不要用 -0.2f。
   先歸零。
*/
#define BALANCE_OFFSET_ANGLE  -0.2f

/* =========================================================
   PWM 參數
   ========================================================= */

#define PWM_MAX               460
#define PWM_MIN_START         48
#define PWM_DEAD_ZONE         1

#define FALL_ANGLE            34.0f
#define ANGLE_LIMIT           12.0f

#define MIN_START_ANGLE       0.25f
#define PWM_RAMP_STEP         26

/* =========================================================
   PID 參數
   ========================================================= */

/*
   P：角度偏多少，馬達補多少。
*/
#define UPRIGHT_KP_LOW        13.0f
#define UPRIGHT_KP_MID        18.0f
#define UPRIGHT_KP_HIGH       28.0f

/*
   I：長時間偏同一邊時，慢慢修正。
   不要太大，否則會暴衝。
*/
#define UPRIGHT_KI            8.0f

/*
   I 積分最大值。
   angle_integral 單位大約是 degree * second。
*/
#define INTEGRAL_LIMIT        8.0f

/*
   I 輸出最大限制。
   避免積分項突然推太大。
*/
#define I_OUTPUT_LIMIT        55.0f

/*
   只有小角度附近才允許積分。
   大角度倒下時不積分，避免越積越大。
*/
#define INTEGRAL_ENABLE_ANGLE 5.0f

/*
   D：阻尼，抑制倒下速度與過衝。
*/
#define UPRIGHT_KD_LOW        0.90f
#define UPRIGHT_KD_MID        1.05f
#define UPRIGHT_KD_HIGH       1.20f

#define D_OUTPUT_LIMIT        140.0f

#define OUTPUT_FILTER_ALPHA   0.16f
#define ZERO_CROSS_DAMPING    0.35f

#define CONTROL_PERIOD_MS     5

/* USER CODE END PD */

/* USER CODE BEGIN PV */

/* MPU6050 */
uint8_t mpu_check = 0;
uint8_t mpu_data = 0;
uint8_t imu_buf[14];

int16_t AccX = 0;
int16_t AccY = 0;
int16_t AccZ = 0;
int16_t GyroX = 0;
int16_t GyroY = 0;
int16_t GyroZ = 0;

HAL_StatusTypeDef mpu_who_status;
HAL_StatusTypeDef mpu_wakeup_status;
HAL_StatusTypeDef mpu_read_status;

/* Angle */
float acc_angle = 0.0f;
float gyro_rate = 0.0f;
float gyro_x_bias = 0.0f;

float angle = 0.0f;
float target_angle = 0.0f;
float angle_error = 0.0f;
float last_angle_error = 0.0f;
float angle_error_limited = 0.0f;

/* PID */
float angle_integral = 0.0f;

/* PWM */
int balance_pwm = 0;
float balance_pwm_filter = 0.0f;

int left_pwm_cmd = 0;
int right_pwm_cmd = 0;

int left_pwm_output = 0;
int right_pwm_output = 0;

/* Time */
uint32_t last_time = 0;
uint32_t now_time = 0;
uint32_t last_control_tick = 0;
float dt = 0.005f;

/* USER CODE END PV */

/* USER CODE BEGIN PFP */

uint8_t MPU6050_Init(void);
uint8_t MPU6050_Read_All(void);
void Balance_Calibrate(void);

int Upright_PID(float error_now, float gyro_now);

void Motor_Set(int left_pwm, int right_pwm);

int Limit_PWM(int value, int max);
float Limit_Float(float value, float max);
float Abs_Float(float value);
int Ramp_PWM(int target_pwm, int *current_pwm);
void Reset_Control_State(void);

/* USER CODE END PFP */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();

    /*
       PWM 接線：
       TIM2_CH3 / PA2 -> 右輪 PWM
       TIM2_CH4 / PA3 -> 左輪 PWM
    */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    Motor_Set(0, 0);
    HAL_Delay(500);

    HAL_I2C_DeInit(&hi2c1);
    HAL_Delay(10);
    MX_I2C1_Init();
    HAL_Delay(50);

    if (MPU6050_Init() != 0)
    {
        Motor_Set(0, 0);

        while (1)
        {
            HAL_Delay(100);
        }
    }

    /*
       開機時請扶正車身，不要晃動。
       這裡會校正直立角與 GyroX 零點。
    */
    Balance_Calibrate();

    Reset_Control_State();

    last_time = HAL_GetTick();
    last_control_tick = HAL_GetTick();

    while (1)
    {
        if (HAL_GetTick() - last_control_tick < CONTROL_PERIOD_MS)
        {
            continue;
        }

        last_control_tick = HAL_GetTick();

        if (MPU6050_Read_All() == 0)
        {
            now_time = HAL_GetTick();
            dt = (now_time - last_time) / 1000.0f;
            last_time = now_time;

            if (dt <= 0.0f || dt > 0.05f)
            {
                dt = 0.005f;
            }

            /*
               MPU6050 擺法：
               VCC 朝車尾
               前後傾斜使用 AccY
               前後角速度使用 GyroX
            */
            acc_angle = SENSOR_DIR * atan2f((float)AccY, (float)AccZ) * 57.2958f;

            gyro_rate = SENSOR_DIR * (((float)GyroX / 131.0f) - gyro_x_bias);

            /*
               互補濾波。
            */
            angle = 0.985f * (angle + gyro_rate * dt) + 0.015f * acc_angle;

            /*
               角度誤差。
            */
            angle_error = angle - (target_angle + BALANCE_OFFSET_ANGLE);

            /*
               倒下保護。
               倒下時一定要清掉 KI，不然下次會突然暴衝。
            */
            if (angle_error > FALL_ANGLE || angle_error < -FALL_ANGLE)
            {
                Reset_Control_State();
                Motor_Set(0, 0);
                HAL_Delay(5);
                continue;
            }

            /*
               角度過零抑制。
               車被救過直立點時，降低舊輸出，避免繼續推到另一邊。
            */
            if ((last_angle_error > 0.0f && angle_error < 0.0f) ||
                (last_angle_error < 0.0f && angle_error > 0.0f))
            {
                balance_pwm_filter *= ZERO_CROSS_DAMPING;
                left_pwm_output = (int)((float)left_pwm_output * ZERO_CROSS_DAMPING);
                right_pwm_output = (int)((float)right_pwm_output * ZERO_CROSS_DAMPING);

                /*
                   過零時也削弱積分，避免 KI 繼續往舊方向推。
                */
                angle_integral *= 0.35f;
            }

            last_angle_error = angle_error;

            /*
               角度 PID。
            */
            balance_pwm = Upright_PID(angle_error, gyro_rate);

            /*
               輸出濾波。
            */
            balance_pwm_filter = OUTPUT_FILTER_ALPHA * balance_pwm_filter +
                                 (1.0f - OUTPUT_FILTER_ALPHA) * (float)balance_pwm;

            left_pwm_cmd = (int)balance_pwm_filter;
            right_pwm_cmd = (int)balance_pwm_filter;

            left_pwm_cmd = Limit_PWM(left_pwm_cmd, PWM_MAX);
            right_pwm_cmd = Limit_PWM(right_pwm_cmd, PWM_MAX);

            /*
               死區。
            */
            if (left_pwm_cmd > -PWM_DEAD_ZONE && left_pwm_cmd < PWM_DEAD_ZONE)
            {
                left_pwm_cmd = 0;
            }

            if (right_pwm_cmd > -PWM_DEAD_ZONE && right_pwm_cmd < PWM_DEAD_ZONE)
            {
                right_pwm_cmd = 0;
            }

            /*
               小角度啟動 PWM。
            */
            if (Abs_Float(angle_error) > MIN_START_ANGLE)
            {
                int min_pwm_dynamic = 0;
                float abs_err = Abs_Float(angle_error);

                min_pwm_dynamic = (int)(18.0f +
                                   (PWM_MIN_START - 18.0f) *
                                   (abs_err - MIN_START_ANGLE) / 2.5f);

                if (min_pwm_dynamic > PWM_MIN_START)
                {
                    min_pwm_dynamic = PWM_MIN_START;
                }

                if (min_pwm_dynamic < 18)
                {
                    min_pwm_dynamic = 18;
                }

                if (left_pwm_cmd > 0 && left_pwm_cmd < min_pwm_dynamic)
                {
                    left_pwm_cmd = min_pwm_dynamic;
                }
                else if (left_pwm_cmd < 0 && left_pwm_cmd > -min_pwm_dynamic)
                {
                    left_pwm_cmd = -min_pwm_dynamic;
                }

                if (right_pwm_cmd > 0 && right_pwm_cmd < min_pwm_dynamic)
                {
                    right_pwm_cmd = min_pwm_dynamic;
                }
                else if (right_pwm_cmd < 0 && right_pwm_cmd > -min_pwm_dynamic)
                {
                    right_pwm_cmd = -min_pwm_dynamic;
                }
            }

            left_pwm_cmd = Ramp_PWM(left_pwm_cmd, &left_pwm_output);
            right_pwm_cmd = Ramp_PWM(right_pwm_cmd, &right_pwm_output);

            Motor_Set(left_pwm_cmd, right_pwm_cmd);
        }
        else
        {
            Reset_Control_State();
            Motor_Set(0, 0);
        }
    }
}

/* USER CODE BEGIN 4 */

/*
   角度 PID 控制。
   P：救角度
   I：修正長時間偏一邊
   D：阻尼，抑制過衝
*/
int Upright_PID(float error_now, float gyro_now)
{
    int output = 0;

    float p_out = 0.0f;
    float i_out = 0.0f;
    float d_out = 0.0f;

    float abs_error = 0.0f;

    float kp_now = UPRIGHT_KP_LOW;
    float kd_now = UPRIGHT_KD_LOW;

    angle_error_limited = Limit_Float(error_now, ANGLE_LIMIT);
    abs_error = Abs_Float(angle_error_limited);

    /*
       小角度柔和，中角度補強，大角度救車。
    */
    if (abs_error > 6.0f)
    {
        kp_now = UPRIGHT_KP_HIGH;
        kd_now = UPRIGHT_KD_HIGH;
    }
    else if (abs_error > 2.5f)
    {
        kp_now = UPRIGHT_KP_MID;
        kd_now = UPRIGHT_KD_MID;
    }
    else
    {
        kp_now = UPRIGHT_KP_LOW;
        kd_now = UPRIGHT_KD_LOW;
    }

    /*
       如果角度正在回正，降低 P 項，避免救過頭。
    */
    if ((error_now > 0.0f && gyro_now < 0.0f) ||
        (error_now < 0.0f && gyro_now > 0.0f))
    {
        kp_now *= 0.55f;
    }

    /*
       P 項。
    */
    p_out = kp_now * angle_error_limited;

    /*
       I 項。
       只有角度不太大時才積分。
       大角度時積分慢慢釋放，避免倒下前越積越多。
    */
    if (abs_error < INTEGRAL_ENABLE_ANGLE)
    {
        angle_integral += angle_error_limited * dt;
    }
    else
    {
        angle_integral *= 0.92f;
    }

    if (angle_integral > INTEGRAL_LIMIT)
    {
        angle_integral = INTEGRAL_LIMIT;
    }
    else if (angle_integral < -INTEGRAL_LIMIT)
    {
        angle_integral = -INTEGRAL_LIMIT;
    }

    i_out = UPRIGHT_KI * angle_integral;

    if (i_out > I_OUTPUT_LIMIT)
    {
        i_out = I_OUTPUT_LIMIT;
    }
    else if (i_out < -I_OUTPUT_LIMIT)
    {
        i_out = -I_OUTPUT_LIMIT;
    }

    /*
       D 項。
    */
    d_out = GYRO_DIR * kd_now * gyro_now;

    if (d_out > D_OUTPUT_LIMIT)
    {
        d_out = D_OUTPUT_LIMIT;
    }
    else if (d_out < -D_OUTPUT_LIMIT)
    {
        d_out = -D_OUTPUT_LIMIT;
    }

    output = (int)(p_out + i_out + d_out);

    /*
       小角度限制輸出，避免 KI + P 一起造成過衝。
    */
    if (abs_error < 0.8f)
    {
        output = Limit_PWM(output, 60);
    }
    else if (abs_error < 1.8f)
    {
        output = Limit_PWM(output, 120);
    }
    else if (abs_error < 3.5f)
    {
        output = Limit_PWM(output, 220);
    }
    else if (abs_error < 6.0f)
    {
        output = Limit_PWM(output, 340);
    }
    else
    {
        output = Limit_PWM(output, PWM_MAX);
    }

    output = BALANCE_DIR * output;
    output = Limit_PWM(output, PWM_MAX);

    return output;
}

/*
   清除控制狀態。
*/
void Reset_Control_State(void)
{
    left_pwm_output = 0;
    right_pwm_output = 0;
    balance_pwm_filter = 0.0f;
    last_angle_error = 0.0f;
    angle_integral = 0.0f;
}

/*
   開機平衡角 + GyroX 零點校正。
*/
void Balance_Calibrate(void)
{
    float angle_sum = 0.0f;
    float gyro_sum = 0.0f;
    int count = 0;
    int i = 0;

    Motor_Set(0, 0);
    HAL_Delay(800);

    for (i = 0; i < 300; i++)
    {
        if (MPU6050_Read_All() == 0)
        {
            acc_angle = SENSOR_DIR * atan2f((float)AccY, (float)AccZ) * 57.2958f;

            angle_sum += acc_angle;
            gyro_sum += ((float)GyroX / 131.0f);

            count++;
        }

        HAL_Delay(3);
    }

    if (count > 0)
    {
        target_angle = angle_sum / count;
        gyro_x_bias = gyro_sum / count;
        angle = target_angle;
    }
    else
    {
        target_angle = 0.0f;
        gyro_x_bias = 0.0f;
        angle = 0.0f;
    }

    Reset_Control_State();
}

/*
   MPU6050 初始化。
*/
uint8_t MPU6050_Init(void)
{
    mpu_who_status = HAL_I2C_Mem_Read(
        &hi2c1,
        0x68 << 1,
        0x75,
        1,
        &mpu_check,
        1,
        100
    );

    if (mpu_who_status != HAL_OK)
    {
        return 1;
    }

    if (mpu_check != 0x68)
    {
        return 2;
    }

    /*
       Wake up MPU6050
    */
    mpu_data = 0x00;

    mpu_wakeup_status = HAL_I2C_Mem_Write(
        &hi2c1,
        0x68 << 1,
        0x6B,
        1,
        &mpu_data,
        1,
        100
    );

    if (mpu_wakeup_status != HAL_OK)
    {
        return 3;
    }

    /*
       Gyro full scale ±250 deg/s
    */
    mpu_data = 0x00;
    HAL_I2C_Mem_Write(
        &hi2c1,
        0x68 << 1,
        0x1B,
        1,
        &mpu_data,
        1,
        100
    );

    /*
       Acc full scale ±2g
    */
    mpu_data = 0x00;
    HAL_I2C_Mem_Write(
        &hi2c1,
        0x68 << 1,
        0x1C,
        1,
        &mpu_data,
        1,
        100
    );

    /*
       低通濾波。
       0x03 反應比較快。
    */
    mpu_data = 0x03;
    HAL_I2C_Mem_Write(
        &hi2c1,
        0x68 << 1,
        0x1A,
        1,
        &mpu_data,
        1,
        100
    );

    HAL_Delay(50);

    return 0;
}

/*
   讀取 MPU6050。
*/
uint8_t MPU6050_Read_All(void)
{
    mpu_read_status = HAL_I2C_Mem_Read(
        &hi2c1,
        0x68 << 1,
        0x3B,
        1,
        imu_buf,
        14,
        100
    );

    if (mpu_read_status != HAL_OK)
    {
        return 1;
    }

    AccX = (int16_t)((imu_buf[0] << 8) | imu_buf[1]);
    AccY = (int16_t)((imu_buf[2] << 8) | imu_buf[3]);
    AccZ = (int16_t)((imu_buf[4] << 8) | imu_buf[5]);

    GyroX = (int16_t)((imu_buf[8] << 8) | imu_buf[9]);
    GyroY = (int16_t)((imu_buf[10] << 8) | imu_buf[11]);
    GyroZ = (int16_t)((imu_buf[12] << 8) | imu_buf[13]);

    return 0;
}

/*
   馬達輸出。
*/
void Motor_Set(int left_pwm, int right_pwm)
{
    left_pwm = Limit_PWM(left_pwm * LEFT_MOTOR_DIR, 999);
    right_pwm = Limit_PWM(right_pwm * RIGHT_MOTOR_DIR, 999);

    /*
       左馬達：
       PWM：TIM2_CH4 / PA3
       DIR：PB12、PB13
    */
    if (left_pwm > 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, left_pwm);
    }
    else if (left_pwm < 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, -left_pwm);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    }

    /*
       右馬達：
       PWM：TIM2_CH3 / PA2
       DIR：PB14、PB15
    */
    if (right_pwm > 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, right_pwm);
    }
    else if (right_pwm < 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, -right_pwm);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    }
}

/*
   PWM 限制。
*/
int Limit_PWM(int value, int max)
{
    if (value > max)
    {
        return max;
    }

    if (value < -max)
    {
        return -max;
    }

    return value;
}

/*
   float 限制。
*/
float Limit_Float(float value, float max)
{
    if (value > max)
    {
        return max;
    }

    if (value < -max)
    {
        return -max;
    }

    return value;
}

/*
   float 絕對值。
*/
float Abs_Float(float value)
{
    if (value >= 0.0f)
    {
        return value;
    }

    return -value;
}

/*
   PWM 斜率限制。
*/
int Ramp_PWM(int target_pwm, int *current_pwm)
{
    if (target_pwm > *current_pwm + PWM_RAMP_STEP)
    {
        *current_pwm += PWM_RAMP_STEP;
    }
    else if (target_pwm < *current_pwm - PWM_RAMP_STEP)
    {
        *current_pwm -= PWM_RAMP_STEP;
    }
    else
    {
        *current_pwm = target_pwm;
    }

    return *current_pwm;
}

/* USER CODE END 4 */

/*
   72MHz 系統時鐘
   HSE 8MHz -> PLL x9 -> 72MHz
*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;

    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    Motor_Set(0, 0);
    __disable_irq();

    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif