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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>
#include "mpu6050.h"
#include <stdio.h>  
#include <string.h> 
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
MPU6050_t MPU6050;

float AngleY;
float Input, Output, I, Input_last, Output_L, Output_R, Motor_L, Motor_R;


float Kp = 7;
float Ki = 0.5;
float Kd = 0.9;

float Vgo = 0;           
float Turn_Control = 0;  
float Target_Speed = 2.5;

uint8_t rx_byte;         

float Offset = -0.1;

int8_t Dir_M_L, Dir_M_R;

volatile int Count_timer_L, Count_timer_R;
volatile uint16_t Period_L, Period_R;

uint32_t loop_timer;
char tx_buffer[64];
uint32_t uart_timer = 0; 


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void Speed_L(int16_t x);
void Speed_R(int16_t x);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    switch (rx_byte)
    {
      case 'F': // Tiến
        Vgo = Target_Speed; 
        Turn_Control = 0;
        break;

      case 'B': // Lùi
        Vgo = -Target_Speed; 
        Turn_Control = 0;
        break;

      case 'L': // Xoay Trái tại chỗ
        Vgo = 0; 
        Turn_Control = 150; // Cộng thêm xung vào một bên để xoay
        break;

      case 'R': // Xoay Phải tại chỗ
        Vgo = 0; 
        Turn_Control = -150;
        break;
      
      case 'G': // Tiến + Trái (Forward Left)
        Vgo = Target_Speed;
        Turn_Control = 80;
        break;

      case 'I': // Tiến + Phải (Forward Right)
        Vgo = Target_Speed;
        Turn_Control = -80;
        break;

      case 'S': // Dừng (Stop)
        Vgo = 0;
        Turn_Control = 0;
        break;
        
      // Chỉnh tốc độ (độ nghiêng)
      case '1': Target_Speed = 1.0; break;
      case '5': Target_Speed = 2.5; break;
      case '9': Target_Speed = 4.5; break;
    }
    // Kích hoạt lại ngắt để nhận ký tự tiếp theo
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    MPU6050_Init(&hi2c1);
        // Cai dat che do vi buoc 1/4 (LOW-HIGH-LOW)
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
        
        // Bat driver dong co (kich hoat o muc LOW)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);

        // Bat dau ngat timer de tao xung STEP
        HAL_TIM_Base_Start_IT(&htim2);
        // Lay thoi gian bat dau cho vong lap chinh
        loop_timer = HAL_GetTick();
				
				HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
        // 1. Dam bao vong lap chinh chay voi chu ky ~4ms
    if (HAL_GetTick() - loop_timer < 4)
    {
      continue;
    }
    loop_timer = HAL_GetTick();

    // 2. Doc goc nghieng tu cam bien MPU6050
        MPU6050_Read_All(&hi2c1, &MPU6050);
        AngleY = MPU6050.KalmanAngleY;
        
    // 3. Thuat toan PID de tinh toan tin hieu dieu khien
    Input = AngleY + Offset - Vgo;
    if (Output > 10.0f || Output < -10.0f) Input += Output * 0.025f;
    I += Input * Ki;
    if(I > 400.0f) I = 400.0f;
    if(I < -400.0f) I = -400.0f;
    Output = Kp * Input + I + Kd * (Input - Input_last);
    Input_last = Input;

    // 4. Gioi han Output va tao vung chet (deadband)
    if (Output > -5.0f && Output < 5.0f) Output = 0.0f;
    if(Output > 400.0f) Output = 400.0f;
    if(Output < -400.0f) Output = -400.0f;
        
    //Output_L = Output;
    //Output_R = Output;
    Output_L = Output + Turn_Control;
    Output_R = Output - Turn_Control;
    if(Output_L > 400.0f) Output_L = 400.0f;
    if(Output_L < -400.0f) Output_L = -400.0f;
    if(Output_R > 400.0f) Output_R = 400.0f;
    if(Output_R < -400.0f) Output_R = -400.0f;

    // 5. Bu tru phi tuyen cho dong co (giu nguyen tu code goc)
    if (Output_L > 0) Output_L = 405.0f - (1.0f / (Output_L + 9.0f)) * 5500.0f;
    else if (Output_L < 0) Output_L = -405.0f - (1.0f / (Output_L - 9.0f)) * 5500.0f;

    if (Output_R > 0) Output_R = 405.0f - (1.0f / (Output_R + 9.0f)) * 5500.0f;
    else if (Output_R < 0) Output_R = -405.0f - (1.0f / (Output_R - 9.0f)) * 5500.0f;

    // 6. Anh xa nguoc gia tri cho ham Speed (giu nguyen)
    if (Output_L > 0) Motor_L = 400.0f - Output_L;
    else if (Output_L < 0) Motor_L = -400.0f - Output_L;
    else Motor_L = 0;

    if (Output_R > 0) Motor_R = 400.0f - Output_R;
    else if (Output_R < 0) Motor_R = -400.0f - Output_R;
    else Motor_R = 0;

    // 7. Ra lenh cho dong co chay
    Speed_L(-Motor_L);
    Speed_R(Motor_R);
		
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
  hi2c1.Init.ClockSpeed = 100000;
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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STEP_R_Pin|DIR_L_Pin|DIR_R_Pin|STEP_L_Pin
                          |MOTOR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MS1_Pin|MS2_Pin|MS3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STEP_R_Pin DIR_L_Pin DIR_R_Pin STEP_L_Pin
                           MOTOR_EN_Pin */
  GPIO_InitStruct.Pin = STEP_R_Pin|DIR_L_Pin|DIR_R_Pin|STEP_L_Pin
                          |MOTOR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MS1_Pin MS2_Pin MS3_Pin */
  GPIO_InitStruct.Pin = MS1_Pin|MS2_Pin|MS3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if (htim->Instance == TIM2) // Chi xu ly ngat tu TIM2
  {
    // --- Tao xung STEP cho Motor Trai (L) ---
    if (Dir_M_L != 0) {
      Count_timer_L++;
      if (Count_timer_L <= (Period_L / 2)) {
        // Toi uu toc do: Ghi truc tiep vao thanh ghi BSRR de SET chan STEP_L
        STEP_L_GPIO_Port->BSRR = STEP_L_Pin;
      } else {
        // Ghi vao phan 16 bit cao cua BSRR de RESET chan STEP_L
        STEP_L_GPIO_Port->BSRR = (uint32_t)STEP_L_Pin << 16U;
      }
      if (Count_timer_L > Period_L) {
        Count_timer_L = 0;
      }
    }

    // --- Tao xung STEP cho Motor Phai (R) ---
    if (Dir_M_R != 0) {
      Count_timer_R++;
      if (Count_timer_R <= (Period_R / 2)) {
        STEP_R_GPIO_Port->BSRR = STEP_R_Pin; // SET
      } else {
        STEP_R_GPIO_Port->BSRR = (uint32_t)STEP_R_Pin << 16U; // RESET
      }
      if (Count_timer_R > Period_R) {
        Count_timer_R = 0;
      }
    }
  }
}

/**
  * @brief  Cai dat toc do va huong cho dong co trai.
  * @param  x: Tin hieu dieu khien (dau xac dinh huong, do lon xac dinh toc do).
  * @retval None
  */
void Speed_L(int16_t x)
{
  if (x < 0) {
    Dir_M_L = -1;
    HAL_GPIO_WritePin(DIR_L_GPIO_Port, DIR_L_Pin, GPIO_PIN_RESET); // Chieu nguoc
  } else if (x > 0) {
    Dir_M_L = 1;
    HAL_GPIO_WritePin(DIR_L_GPIO_Port, DIR_L_Pin, GPIO_PIN_SET);   // Chieu thuan
  } else {
    Dir_M_L = 0; // Dung
  }

  // Cap nhat chu ky xung, gia tri 0 nghia la khong tao xung
  Period_L = abs(x);
}

void Speed_R(int16_t x)
{
  if (x < 0) {
    Dir_M_R = -1;
    HAL_GPIO_WritePin(DIR_R_GPIO_Port, DIR_R_Pin, GPIO_PIN_RESET);
  } else if (x > 0) {
    Dir_M_R = 1;
    HAL_GPIO_WritePin(DIR_R_GPIO_Port, DIR_R_Pin, GPIO_PIN_SET);
  } else {
    Dir_M_R = 0;
  }
  
  Period_R = abs(x);
}

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
