/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body

  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DS18B20_PORT GPIOA
#define DS18B20_PIN GPIO_PIN_1

/* *************** define application level MACROS ********************* */
#define SYSTEM_FREQ 0.1F			// sampling and actuation frequency 0.1 Hz
#define SYSTEM_LOOP_INTERVAL 10		// sampling and actuation period 10 s

#define MAX_CONTROL_SIGNAL 100.0F
#define MIN_CONTROL_SIGNAL 0.0F

#define MAX_DUTY_CYCLE 20.0F
#define MIN_DUTY_CYCLE 0.0F

/* to be used in final version with UI */
#define MIN_TEMPERATURE 20
#define MAX_TEMPERATURE 30

#define SENSOR_CALIBRATION_OFFSET -0.4F

typedef enum { DS18B20_ERROR = -1, DS18B20_OK = 1} DS18B20_error_codes_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* *************** define global variables ********************* */
volatile uint32_t time = 0;

float temperature = 0.0;

float off_cycle = 0.0;
float duty_cycle = 100.0;
float setpoint = 0.0;					// setpoint in ºC

volatile bool trigger_system_flag = false;	// flag to trigger the digital control system

//volatile bool stop_flag = false;						// flag to stop the system

static uint8_t uart_buff[200];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

/* *************** declare application level functions ********************* */

/* function to set the duty cycle*/
void set_duty_cycle(TIM_HandleTypeDef* const, float uint16_t);

/* function to implement the PI controller */
float pi_controller(float setpoint, float curr_input, float h);

void set_pin_output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void set_pin_input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

float read_temperature(void);

/* blocking microseconds delay function using Timer 6 */
void delay_us(uint16_t delay_us);

/* */
uint8_t DS18B20_start(void);
void DS18B20_write(uint8_t data);
uint8_t DS18B20_read(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay_us(uint16_t delay_us)
{
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while( __HAL_TIM_GET_COUNTER(&htim6) < delay_us);
}

void set_pin_output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void set_pin_input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

uint8_t DS18B20_start(void)
{
	uint8_t retcode = 0;

	set_pin_output(DS18B20_PORT, DS18B20_PIN);   // set the pin as output
	HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin low
	delay_us(480);   // delay according to datasheet

	set_pin_input(DS18B20_PORT, DS18B20_PIN);    // set the pin as input
	delay_us(80);    // delay according to datasheet

	if (!HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN) )
	{
		retcode = DS18B20_OK;    // if the pin is low i.e the presence pulse is detected
	}
	else
	{
		retcode = DS18B20_ERROR;
	}

	delay_us(400); // 480 us delay totally.

	return retcode;
}

void DS18B20_write(uint8_t data)
{
	set_pin_output(DS18B20_PORT, DS18B20_PIN);  // set as output

	for (int i = 0; i < 8; i++)
	{

		if ( (data & (1 << i) ) !=0)  // if the bit is high
		{
			// write 1
			set_pin_output(DS18B20_PORT, DS18B20_PIN);  // set as output
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			delay_us(1);  // wait for 1 us

			set_pin_input(DS18B20_PORT, DS18B20_PIN);  // set as input
			delay_us(50);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0cycle
			set_pin_output(DS18B20_PORT, DS18B20_PIN);
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			delay_us(50);  // wait for 60 us

			set_pin_input(DS18B20_PORT, DS18B20_PIN);
		}
	}
}


uint8_t DS18B20_read(void)
{
	uint8_t value = 0;

	set_pin_input(DS18B20_PORT, DS18B20_PIN);

	for (int i = 0; i < 8; i++)
	{
		set_pin_output(DS18B20_PORT, DS18B20_PIN);   // set the pin as output

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);  // pull the data pin LOW
		delay_us(2);  // wait for 2 us

		set_pin_input(DS18B20_PORT, DS18B20_PIN);   // set the pin as output

		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))  // if the pin is HIGH
		{
			value |= 1 << i;  // read = 1
		}
		delay_us(60);  // wait for 60 us
	}
	return value;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	DS18B20_error_codes_t DS18B20_retcode = DS18B20_OK;
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
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  HAL_StatusTypeDef ret = HAL_OK;

  /* start timer used for micro second delay function */
  HAL_TIM_Base_Start(&htim6);
  if(ret != HAL_OK)
  {
  	  sprintf((char *) uart_buff, "Error starting TIM6!\n\r");
  	  HAL_UART_Transmit(&huart2, uart_buff, strlen( (const char *) uart_buff), HAL_MAX_DELAY);
  }

  /* start timer used for triggering the system @ 0.1 Hz (acquire, process, actuate) */
  HAL_TIM_Base_Start_IT(&htim7);
  if(ret != HAL_OK)
  {
  	  sprintf((char *) uart_buff, "Error starting TIM7!\n\r");
  	  HAL_UART_Transmit(&huart2, uart_buff, strlen( (const char *) uart_buff), HAL_MAX_DELAY);
  }

  /* start timer used for PWM generation */
  ret = HAL_TIM_Base_Start(&htim8);
  if(ret != HAL_OK)
  {
  	  sprintf((char *) uart_buff, "Error starting TIM8!\n\r");
  	  HAL_UART_Transmit(&huart2, uart_buff, strlen( (const char *) uart_buff), HAL_MAX_DELAY);
  }

  ret = HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  if(ret != HAL_OK)
  {
	  sprintf((char *) uart_buff, "Error starting PWM!\n\r");
  	  HAL_UART_Transmit(&huart2, uart_buff, strlen( (const char *) uart_buff), HAL_MAX_DELAY);
  }

  set_duty_cycle(&htim8, duty_cycle);

  /* */
  sprintf((char *) uart_buff, "##### Init app! #####\n\r");
  HAL_UART_Transmit(&huart2, uart_buff, strlen( (const char *) uart_buff), HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  DS18B20_retcode = DS18B20_start();

  if(DS18B20_retcode == DS18B20_ERROR)
  {
	  sprintf((char *) uart_buff, "Error starting sensor!\n\r");
	  HAL_UART_Transmit(&huart2, uart_buff, strlen( (const char *) uart_buff), HAL_MAX_DELAY);
  }

  off_cycle = 0.0;
  duty_cycle = 100.0 - off_cycle;
  set_duty_cycle(&htim8, duty_cycle);

  setpoint = 20.0;

  while(true)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(trigger_system_flag)
	  {
		  //HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

		  /* 1) sample system: measure load temperature and ambient temperature */
		  temperature = read_temperature();
		  temperature = temperature + SENSOR_CALIBRATION_OFFSET;
		  /* round using 2 decimal places */
		  temperature = floor(temperature * 100) / 100;

		  /* 2) compute PI response */
		  float u = pi_controller(setpoint, temperature, SYSTEM_FREQ);   // testing
		  float duty_cycle = 100.0 - u;

		  /* 3) actuation */
		  set_duty_cycle(&htim8, duty_cycle);

		  /* specify time instant at which the system starts  */
		  if(time >= 30)
		  {
//			  off_cycle = 35.0;
//			  duty_cycle = 100 - off_cycle;

			  setpoint = 28.0;	// test closed loop system
		  }

		  //sprintf((char *) uart_buff, "%lu,%.1f,%.1f\r\n", time, temperature, off_cycle);
		  //HAL_UART_Transmit(&huart2, uart_buff, strlen( (const char *) uart_buff), HAL_MAX_DELAY);

		  time += SYSTEM_LOOP_INTERVAL;

		  /* clear trigger flag to prevent entering the loop several times between system cycles */
		  trigger_system_flag = false;

		  //HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);	// GPIO PA5 = D13
	  }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 90-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 13734-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65531-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 199;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 44999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	trigger_system_flag = true;
}

float read_temperature(void)
{
	uint8_t temp_msb_byte = 0x00;
	uint8_t temp_lsb_byte = 0x00;

	/* start conversion */
	DS18B20_start();
	HAL_Delay(1);
	DS18B20_write(0xCC);  // skip ROM
	DS18B20_write(0x44);  // convert t
	HAL_Delay(750);

	DS18B20_start();
	HAL_Delay(1);
	DS18B20_write(0xCC);  // skip ROM
	DS18B20_write(0xBE);  // Read Scratch-pad

	temp_lsb_byte = DS18B20_read();
	temp_msb_byte = DS18B20_read();
	/* end conversion */

	int16_t temperature = (temp_msb_byte << 8) | temp_lsb_byte;
	float temperature_fp = (float) temperature / 16;

	return temperature_fp;
}

void set_duty_cycle(TIM_HandleTypeDef* const handle, const float duty_cycle)
{
	assert(handle != NULL);
	assert( (duty_cycle >= 0.0) && (duty_cycle <= 100.0) );

	uint32_t new_OCCR = (uint32_t)((float)(handle->Instance->ARR) * duty_cycle * 0.01);

	/* In case the new duty cycle is bigger than the reload register, saturate it to the reload register */
	if(new_OCCR > handle->Instance->ARR)
	{
		new_OCCR = handle->Instance->ARR;
	}

	handle->Instance->CCR1 = new_OCCR;
}

/* function to implement the discrete PI controller */
float pi_controller(float setpoint, float curr_input, float h)
{
    // works great with h = 10 s

	/* Controller gains */
    static const float K = 47.50;
    static const float Ki = 0.003;

//    float Ti = K / Ki;
    //static const Td = 0;                 // no D component

    float error = 0.0;
    static float integral = 0.0;

    //static float error_prev = 0.0;
    //static float u_prev = 0.0;

    /* closed loop discrete implementation -> controller has pole in the origin and a zero in -s1/s0 */
//    float s0 = K * (1 + h / Ti);
//    float s1 = -K;
    /* compute control signal */
//    float u = u_prev + s0*error + s1*error_prev;    // discrete model

    /* compute error signal */
    error = setpoint - curr_input;

    /* update error integral */
    integral += error*h;

    /* compute control signal */
    float u = (K*error) + (Ki*integral);    // continuous model

    /* implement anti windup (specially important if the setpoint is manually controlled )*/

    /* implement saturation block */
    if(u > MAX_CONTROL_SIGNAL)
    {
        u = MAX_CONTROL_SIGNAL;
    }
    if(u < MIN_CONTROL_SIGNAL)
    {
        u = MIN_CONTROL_SIGNAL;
    }

    // DEBUG and data extraction
	sprintf((char *) uart_buff, "t:%lu\ttemp:%.2f\terror:%.2f\t prop:%.2f\tintegral:%.2f u:%.2f \t ref:%.1f\r\n", time, temperature, error, K*error, Ki*integral, u , setpoint);
	HAL_UART_Transmit(&huart2, uart_buff, strlen( (const char *) uart_buff), HAL_MAX_DELAY);

    /* update previous error value (discrete implementation) */
    //error_prev = error;
    //u_prev = u;

    return u;
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
