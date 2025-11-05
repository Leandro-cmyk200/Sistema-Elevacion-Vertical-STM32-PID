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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <i2c_lcd.h>
#include <string.h>
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
float Kp = 1.0, Ki = 0.5, Kd = 0.1;
float error, previous_error = 0, integral = 0;
float pid_output;
float setpoint = 13.0;

// Solo usaremos el sensor ultrasónico
void update_motor_pid(float setpoint, float measurement, float distancia);
void set_motor_direction(uint8_t forward);
void setPWM(int16_t duty_percent);
void send_motor_status_i2c(int16_t pwm_value, float distancia);
// Control de dirección (GPIO)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef SLAVE_ADDRESS_LCD
#define SLAVE_ADDRESS_LCD (0x27 << 1) // o (0x3F << 1) según tu módulo
#endif
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart2;
osThreadId defaultTaskHandle;
osThreadId Task_PIDHandle;
osThreadId Task_LCDHandle;

/* USER CODE BEGIN PV */

uint32_t echo_start = 0, echo_end = 0;
uint8_t echo_captured = 0;
float distancia = 0.0f;
int pwm_value = 0;
float ultima_distancia_valida = 0.0f;

// Failover monitoring
uint32_t failover_start = 0; // HAL_GetTick timestamp when saturation started
uint8_t in_failover = 0;     // 0 = normal, 1 = using backup motor (PB5)
uint32_t recovery_start = 0; // timestamp when recovery (below saturation) began
uint32_t last_switch_time = 0; // timestamp of last failover switch
const uint32_t min_failover_dwell_ms = 30000; // 30s minimum between switches
uint32_t aux_fail_start = 0; // when backup motor is saturated
uint8_t system_fault = 0; // 0=ok, 1=fault/shutdown
uint8_t motor1_disabled = 0; // 0=allowed, 1=permanently disabled after failover

/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void const * argument);
void StartTaskPID(void const * argument);
void StartTaskLCD(void const * argument);

/* USER CODE BEGIN PFP */

float read_setpoint(void);
void loop_ultrasonic_pid(void);
void update_motor_pid(float setpoint, float measurement, float distancia);
void set_motor_direction(uint8_t forward);
void setPWM(int16_t duty_percent);
void send_motor_status_i2c(int16_t pwm_value, float distancia);
void lcd_print_diagnostico(float distancia_cm, int16_t pwm_percent);

void lcd_print_diagnostico(float distancia_cm, int16_t pwm_percent) {
  char linea1[17];
  char linea2[17];
  static char prev_linea1[17] = "";
  static char prev_linea2[17] = "";

  // Línea 1: Distancia actual y setpoint
  snprintf(linea1, sizeof(linea1), "Dis:%2.1f  SP:%2.1f", distancia_cm, setpoint);

  // Línea 2: PWM y setpoint
  snprintf(linea2, sizeof(linea2), "PWM:%+4d%%", pwm_percent);

  // Actualización selectiva
  if (strcmp(linea1, prev_linea1) != 0) {
    lcd_put_cursor(0, 0);
    lcd_send_string(linea1);
    strcpy(prev_linea1, linea1);
  }
  if (strcmp(linea2, prev_linea2) != 0) {
    lcd_put_cursor(1, 0);
    lcd_send_string(linea2);
    strcpy(prev_linea2, linea2);
  }
}

/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
void send_trigger_pulse(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
}
float measure_distance_cm(void) {
    echo_captured = 0;
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    send_trigger_pulse();
    HAL_Delay(50);

    if (echo_captured == 2) {
        uint32_t duration = (echo_end > echo_start) ? (echo_end - echo_start) : (0xFFFF - echo_start + echo_end);
        return duration / 58.0;
    }
    return -1;
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
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  
  // Inicializar LCD
  lcd_init();
  lcd_clear();
  
  // Leer setpoint inicial
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  setpoint = read_setpoint();


  // Iniciar PWM para motor principal
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);          // Motor principal
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);          // Motor principal
  
  // Iniciar PWM para motor backup
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);          // PB3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);          // PB1
  
  printf("PWM Channels iniciados. Principal: TIM2_CH3/TIM3_CH1, Backup: PB1/PB3\r\n");
  printf("PWM Channels iniciados. Principal: TIM2_CH3/TIM3_CH1, Backup: TIM3_CH4(PB1), TIM2_CH2(PB3)\r\n");
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  float measurement = measure_distance_cm();
  distancia = measurement; // si querés usar el mismo valor


  update_motor_pid(setpoint, measurement, distancia);



  //printf("Contador TIM4: %lu\r\n", __HAL_TIM_GET_COUNTER(&htim4));
  HAL_Delay(500);
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

  /* definition and creation of Task_PID */
  osThreadDef(Task_PID, StartTaskPID, osPriorityNormal, 0, 256);
  Task_PIDHandle = osThreadCreate(osThread(Task_PID), NULL);

  /* definition and creation of Task_LCD */
  osThreadDef(Task_LCD, StartTaskLCD, osPriorityNormal, 0, 256);
  Task_LCDHandle = osThreadCreate(osThread(Task_LCD), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


      /* Infinite loop */
      while (1) 
      {
          // El planificador de FreeRTOS maneja todo
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;  // Mismo prescaler que TIM2/3
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;    // Mismo período que TIM2/3
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;    // Mismo prescaler que TIM2
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;       // Mismo periodo que TIM2
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 63;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* PB4 is configured as TIM3_CH1 in CubeMX (.ioc). Do not configure it here as GPIO—
    TIM post-init will set the pin to the timer alternate function for PWM. */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

float read_setpoint(void) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint16_t raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    
    // Escala el valor de 0-4095 a 5-20cm para tener un rango útil
    float setpoint = 5.0f + (raw / 4095.0f) * 15.0f; // Rango de 5 a 20 cm
    return setpoint;
}


void update_motor_pid(float setpoint, float measurement, float distancia) {
    float error = setpoint - measurement;
    static float integral = 0;
    static float previous_error = 0;
    integral += error;
    float derivative = error - previous_error;
    float pid_output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;

    if(integral >= 200){
        integral = 200.00;
    } else if(integral <= -200.00) {
    	integral = -200.00;
    }

    //uint8_t pwm_valu= (uint8_t)pid_output;
    // Clamp PID output to -100..100 and apply signed PWM
    if (pid_output > 100.0f) pid_output = 100.0f;
    if (pid_output < -100.0f) pid_output = -100.0f;
    pwm_value = (int)pid_output;
    setPWM((int16_t)pwm_value);

    // 🖨️ Diagnóstico por UART
    printf("Error: %.2f | PID: %.2f | PWM: %+d%%\r\n",
      error, pid_output, pwm_value);
    printf("integral: %.2f\n", integral);

    lcd_print_diagnostico(distancia, pwm_value);
}

void set_motor_direction(uint8_t forward) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void setPWM(int16_t duty_percent) {
  // duty_percent: -100 .. 100
  if (duty_percent > 100) duty_percent = 100;
  if (duty_percent < -100) duty_percent = -100;
  
  // Calcular PWM exacto
  uint16_t abs_duty = (uint16_t)(duty_percent < 0 ? -duty_percent : duty_percent);
  uint32_t pulse = (abs_duty * (99 + 1)) / 100;

  if (system_fault) {
    // En caso de falla, apagar ambos motores
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    return;
  }

  // Establecer dirección para ambos motores
  set_motor_direction(duty_percent >= 0);

  if (in_failover) {
    // Apagar motor principal primero
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    // Motor backup usando PB1 (TIM3_CH4) y PB3 (TIM2_CH2)
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse); // PB3
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse); // PB1
    printf("Backup motor activo - PWM:%d%%, Pulse:%lu\r\n", duty_percent, pulse);
  } else {
    // Apagar motor backup primero
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);    // PB3
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);    // PB1
    
    // Motor principal
    if (duty_percent < 0) {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
    } else {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse);
    }
    printf("Main motor - PWM:%d%%, Pulse:%lu\r\n", duty_percent, pulse);
  }
}

void system_shutdown(void) {
  system_fault = 1;
  
  // Stop all PWMs and force 0 duty
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);  // Motor principal
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);  // Motor principal
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);  // Motor backup (PB1)
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);  // Motor backup (PB3)
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);

  // Indicate error on LCD and serial
  lcd_clear();
  lcd_put_cursor(0,0);
  lcd_send_string("ERROR: MOTORES");
  lcd_put_cursor(1,0);
  lcd_send_string("SATURADOS >20s");

  printf("FATAL: Ambos motores saturados por >20s, apagando sistema\r\n");
}

void send_motor_status_i2c(int16_t pwm_value, float distancia) {
  uint8_t buffer[3];
  uint16_t distancia_int = (uint16_t)(distancia * 100);

  // Enviar PWM como int8_t en buffer[0] (valor -128..127)
  buffer[0] = (uint8_t)((int8_t)pwm_value);
  buffer[1] = (distancia_int >> 8) & 0xFF;
  buffer[2] = distancia_int & 0xFF;

  HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, buffer, 3, HAL_MAX_DELAY);

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        if (echo_captured == 0) {
            echo_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
            echo_captured = 1;
        } else if (echo_captured == 1) {
            echo_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            echo_captured = 2;
        }
        //printf("Callback activo\r\n");
    }
}

void loop_ultrasonic_pid(void) {
    float medida_ultra = measure_distance_cm();

    if (medida_ultra > 0 && medida_ultra <= 100.00f) {
        distancia = medida_ultra;
        ultima_distancia_valida = distancia;
        printf("Distancia: %.2f cm\r\n", distancia);
        update_motor_pid(setpoint, distancia, distancia);
    } else {
        distancia = ultima_distancia_valida;
        printf("Usando última distancia válida: %.2f cm\r\n", distancia);
        update_motor_pid(setpoint, distancia, distancia);
    }
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
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskPID */
/**
* @brief Function implementing the Task_PID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskPID */
void StartTaskPID(void const * argument)
{
  /* USER CODE BEGIN StartTaskPID */
  /* Infinite loop */
  for(;;)
  {
      // Actualizar setpoint desde el potenciómetro
      setpoint = read_setpoint();
      
      // Ejecutar el control PID
      loop_ultrasonic_pid();
      
      // Failover detection and permanent disable behaviour
      // - If motor1 (principal) reaches 100% and stays 20s -> switch to backup permanently and set motor1_disabled=1
      // - While in backup (in_failover==1 or motor1_disabled==1): if backup reaches 100% and stays 20s -> fatal
      int abs_pwm = pwm_value < 0 ? -pwm_value : pwm_value;
      uint32_t now = HAL_GetTick();

      printf("Debug - PWM:%d abs:%d in_failover:%d motor1_disabled:%d\r\n", pwm_value, abs_pwm, in_failover, motor1_disabled);

      if (!in_failover && !motor1_disabled) {
        // Monitor main motor saturation
        if (abs_pwm == 100) {
          if (failover_start == 0) {
            failover_start = now;
            printf("Debug - Motor principal saturado, iniciando conteo: %lu\r\n", now);
          }
          if ((now - failover_start) >= 20000) {
            // Switch permanently to backup
            in_failover = 1;
            motor1_disabled = 1;
            failover_start = 0;
            aux_fail_start = 0;
            last_switch_time = now;
            printf("FAILOVER: motor principal 100% x20s -> cambiando a backup, motor1 deshabilitado\r\n");
            setPWM((int16_t)pwm_value);
          }
        } else {
          // reset if not saturated
          failover_start = 0;
        }
      } else {
        // We're in backup (either temporary or permanent). Monitor backup saturation for fatal condition.
        if (abs_pwm == 100) {
          if (aux_fail_start == 0) {
            aux_fail_start = now;
            printf("Debug - Motor backup saturado, iniciando conteo: %lu\r\n", now);
          }
          if ((now - aux_fail_start) >= 20000) {
            printf("FATAL: Motor backup 100% x20s -> apagando ambos motores\r\n");
            system_shutdown();
          }
        } else {
          aux_fail_start = 0;
        }
      }

      osDelay(50); // cada 50 ms
  }
  /* USER CODE END StartTaskPID */
}

/* USER CODE BEGIN Header_StartTaskLCD */
/**
* @brief Function implementing the Task_LCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLCD */
void StartTaskLCD(void const * argument)
{
  /* USER CODE BEGIN StartTaskLCD */
  /* Infinite loop */
  for(;;)
  {
      lcd_print_diagnostico(distancia, (int16_t)pwm_value);
      osDelay(1000); // cada 1000 ms

  }
  /* USER CODE END StartTaskLCD */
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
