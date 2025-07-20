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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "xunji.h"
#include "inv_mpu.h"
#include "mpu6050.h"
#include "control.h"
#include "inv_mpu_dmp_motion_driver.h"
#include <math.h>
#include "bluetooth.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t grey_watch = 0;

float pitch, roll, yaw;
short gyrox, gyroy, gyroz;
short aacx, aacy, aacz;
int g_speed_L, g_speed_R;
long g_total_encoder_cnt;

extern TIM_HandleTypeDef htim3, htim4;
extern DMA_HandleTypeDef hdma_usart3_rx;

extern _Bool is_turning;
extern int target_speed_L, target_speed_R;
extern int target_speed;

extern char received_data[110];
extern char send_data[50];

extern PID_Parameters para_temp;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// uart transmit
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 50);
  return ch;
}

// calibrate_gyro
uint32_t GyroTick = 0;
#define GYRO_TOLERANCE 0.2f // 容差 0.2°之内
#define STABLE_COUNT 110
_Bool Gyro_Calibration_Done = 0;
float YAW_MIDDLE;
void Calibrate_Gyro()
{
  if (!Gyro_Calibration_Done)
  {
    if (uwTick - GyroTick < 100)
      return; // 每100ms调用一次
    GyroTick = uwTick;

    float last_yaw = 0.0f;
    int same_count = 0;

    if (fabs(yaw - last_yaw) < GYRO_TOLERANCE)
      same_count++;
    else
      same_count = 0; // 清零

    last_yaw = yaw;

    if (same_count >= STABLE_COUNT)
    {
      YAW_MIDDLE = yaw;
      Gyro_Calibration_Done = 1;
    }
  }
}

// read speed & angle
uint32_t DataTick = 0;
void Get_Data(void)
{

  g_speed_L = -Read_Speed(&htim4);
  g_speed_R = Read_Speed(&htim3);
  g_total_encoder_cnt += ((g_speed_L + g_speed_R) / 2);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim1)
  {
    Get_Data();
  }
}
// control per 20ms
uint32_t CtrlTick = 0;
extern PID_Para PID_L, PID_R;
int motor0, motor1;
void Control(void)
{
  if (uwTick - CtrlTick < 20)
    return;
  CtrlTick = uwTick;

  mpu_dmp_get_data(&pitch, &roll, &yaw);
  MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
  MPU_Get_Accelerometer(&aacx, &aacy, &aacz);
  // control

  // Control_test0();

  PID_L.Kp = para_temp.kp0;
  PID_L.Ki = para_temp.ki0;
  PID_L.Kd = para_temp.kd0;
  PID_R.Kp = para_temp.kp1;
  PID_R.Ki = para_temp.ki1;
  PID_R.Kd = para_temp.kd1;
  // Bluetooth parameter tuning
  target_speed = (int)para_temp.target0;

  // Transmit the speed values via UART

  // HAL_UART_Transmit(&huart3, (uint8_t*)send_data, strlen(send_data), 100);
  pid_set_target(&PID_L, target_speed * 100.0f / 296);
  pid_set_target(&PID_R, target_speed * 100.0f / 274);
  motor0 = pid_control(g_speed_R * 100.0f / 274, &PID_R);

  motor1 = pid_control(g_speed_L * 100.0f / 296, &PID_L);

  Motor_Load(-motor1, motor0);
  sprintf(send_data, "{Left_Speed:%d,Right_Speed:%d,target:%d}|", g_speed_L, g_speed_R, target_speed); // Print the speed values
  printf("%s", send_data);
}

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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  MPU_Init();
  mpu_dmp_init();
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim1);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)received_data, sizeof(received_data));
  // 关闭dma传输过半中断
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
  // HAL_UART_Receive_DMA(&huart3, (uint8_t *)received_data, sizeof(received_data));
  PID_xunji_Init();
  PID_mpu6050_Init();

  // Motor_Load(20,20);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    Control();

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
