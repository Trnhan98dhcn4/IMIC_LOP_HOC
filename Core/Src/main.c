/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


typedef enum
{
	LED_OFF,
	LED_ON
}State_t;

uint32_t dem = 1;

void GPIO_Init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	// thanh ghi moder
	uint32_t* GPIOA_MODER = (uint32_t*)(0x40020000);
	*GPIOA_MODER &= ~(0b11 << 0);
	uint32_t* GPIOD_MODER = (uint32_t*)(0x40020c00);
	*GPIOD_MODER &= ~(0xff << 0);//  su dung 4 led
	*GPIOD_MODER |= (0b01 << 24) | (0b01 << 26) | (0b01 << 28) | (0b01 << 30);

	//thanh ghi pupdr
	uint32_t* GPIOA_PUPDR = (uint32_t*)(0x4002000c);
	*GPIOA_PUPDR &= ~(0b1 << 0);

	//thanh ghi  otyper
	uint32_t* GPIOD_OTYPER = (uint32_t*)(0x40020c04);
	*GPIOD_OTYPER &= ~(0xf << 12);
}

// khoi tao interrput

void EXTI0_Init()
{
	// khoi tao trong EXTI0
	uint32_t* EXTICR1 = (uint32_t*)(0x40013808);
	*EXTICR1 &= ~(0xf << 0);
	uint32_t* FTSR = (uint32_t*)(0x40013c0c);
	*FTSR |= 0b1 << 0;
	uint32_t* IMR = (uint32_t*)(0x40013c00);
	*IMR |= 0b1 << 0;

	// khoi tao trong NVIC trong ARM cotrex
	uint32_t* NVIC_ISER0 = (uint32_t*)(0xe000e100);
	*NVIC_ISER0 |= (0b1 << 6);
}
unsigned int  tick;
uint32_t* SYST_CR = (uint32_t*)(0xE000E010);
uint32_t* SYST_RV = (uint32_t*)(0xE000E014);
void delay_init()
{
	*SYST_RV = 16000 / 8 - 1;
	*SYST_CR |= 1 | (1 << 1); // enable timer 1 |  enable interrupt timer
}
void delay(unsigned int time)
{
	tick = 0;
	while(tick < time);
}


void Systick_custom_Handler()
{
	tick++;
	*SYST_CR &= ~(1 << 16);
}

void led_control(int led_num, State_t state)
{
	uint32_t* GPIOD_ODR = (uint32_t*)(0x40020c14);
	if(state == LED_ON)
	{
		*GPIOD_ODR |= (0b1 << (12 + led_num));
	}
	else
	{
		*GPIOD_ODR &= ~(0b1 << (12 + led_num));
	}
}

char Get_button()
{
	uint32_t* GPIOD_IDR = (uint32_t*)(0x40020010);
	return (*GPIOD_IDR & 1);
}
void EXTI0_IRQHandler()
{
	uint32_t* PR = (uint32_t*)(0x40013c14);
	*PR |= (0b1 << 0);
}

void EXTI0_Custom_Handler()
{
	dem++;
	if(dem % 2 == 0)
	{
		led_control(0, LED_ON);
		led_control(1, LED_ON);
		led_control(2, LED_ON);
		led_control(3, LED_ON);
	}
	else
	{
		led_control(0, LED_OFF);
		led_control(1, LED_OFF);
	    led_control(2, LED_OFF);
		led_control(3, LED_OFF);
	}
	uint32_t* PR = (uint32_t*)(0x40013c14);
	*PR |= (0b1 << 0);
}
void Flash_to_Ram()
{
	memcpy(0x20000000, 0, 0x198);// copy vector table size 0 -> 0x198
	uint32_t* VTOR = (uint32_t*)0xe000ed08; // enable vector table
	*VTOR = 0x20000000; //in Ram

	// custom exti0
	uint32_t* EXTI0_Address_Custom = (uint32_t*) 0x20000058;
	*EXTI0_Address_Custom = (uint32_t)EXTI0_Custom_Handler | 1;

	//custom Systick
	uint32_t* Systick_address_Custom = (uint32_t*) 0x2000003C;
	*Systick_address_Custom = (uint32_t)Systick_custom_Handler | 1;
}

void UART_Init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();

	uint32_t* GPIOA_MODER = (uint32_t*)0x40020000;
	*GPIOA_MODER &= ~(0b1111 << 4);
	*GPIOA_MODER |= (0b10 << 4) | (0b10 << 6);
	uint32_t* GPIOA_AFRL = (uint32_t*)0x40020020;
	*GPIOA_AFRL |= (7 << 8) | (7 << 12);

	// set baut rate
	uint32_t* UART2_BRR = (uint32_t*)0x40004408;
	*UART2_BRR = (104 << 4) | 3;

	//set size and check chan le
	uint32_t* UART2_CR1 = (uint32_t*)0x4000440c;
	*UART2_CR1 |= (0b1 << 13) | (0b1 << 2) | (0b1 << 3);
}

void UART_Send_byte(char data)
{
	uint32_t* UART2_SR = (uint32_t*)0x40004400;
	uint32_t* UART2_DR = (uint32_t*)0x40004404;

	while(((*UART2_SR >> 7) & 1) != 1);
	*UART2_DR = data;
	while(((*UART2_SR >> 6) & 1) != 0);
	//*UART2_SR &= ~(1 << 6);
}

void UART_Send_ARR(char* arr, int num)
{
	for(int i = 0; i < num; i++)
	{
		UART_Send_byte(arr[i]);
	}
}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

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
  GPIO_Init();
  EXTI0_Init();
  delay_init();
  Flash_to_Ram();
  UART_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  char msg[] = "Hello PC! we  are the STM32 \r\n";

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  UART_Send_ARR(msg, sizeof(msg));
		 led_control(0, LED_ON);
		 delay(1000);
		 led_control(0, LED_OFF);
		 delay(1000);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
