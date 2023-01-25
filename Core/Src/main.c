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
#include "string.h"
#include "spi_lib.h"

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

typedef enum
{
	sector_0,
	sector_1,
	sector_2,
	sector_3,
	sector_4,
	sector_5,
	sector_6,
	sector_7
}Sector_t;

uint32_t dem = 1;
char rx_buf[5176];
int rx_index;
short data_sensor[3] = {0};
short x = 0;
short y = 0;
short z = 0;

char ESP32_AT[] = "AT\r\n";
char ESP32_MODE[] = "AT+CWMODE=1\r\n";
char ESP32_CWJAP[] = "AT+CWJAP=\"FPT-Software-2G\",""\"Admin@2022\"\r\n";
char ESP32_CIPSTART[] = "AT+CIPSTART=\"TCP\",""\"192.168.2.9\",""1234\r\n";
char ESP32_CIPSEND[] = "AT+CIPSEND=3\r\n";


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


void UART_Send_byte(char data)
{
	uint32_t* UART2_SR = (uint32_t*)0x40004400;
	uint32_t* UART2_DR = (uint32_t*)0x40004404;

	//set TXE if TXE set 1 to return;
	while(((*UART2_SR >> 7) & 1) != 1);
	*UART2_DR = data;
	// set TC if TC set 0 to return
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

void EXTI0_Custom_Handler()
{

	dem++;
	if(dem % 2 == 0)
	{
		led_control(0, LED_ON);
		led_control(1, LED_ON);
		led_control(2, LED_ON);
		led_control(3, LED_ON);
		UART_Send_ARR(ESP32_AT, sizeof(ESP32_AT));
		UART_Send_ARR(ESP32_MODE, sizeof(ESP32_MODE));
		UART_Send_ARR(ESP32_CWJAP, sizeof(ESP32_CWJAP));
;

	}
	else
	{
		UART_Send_ARR(ESP32_CIPSTART, sizeof(ESP32_CIPSTART));
		UART_Send_ARR(ESP32_CIPSEND, sizeof(ESP32_CIPSEND));
		//memset(rx_buf, 0, 255);
		led_control(0, LED_OFF);
		led_control(1, LED_OFF);
	    led_control(2, LED_OFF);
		led_control(3, LED_OFF);
	}
	uint32_t* PR = (uint32_t*)(0x40013c14);
	*PR |= (0b1 << 0);
}

void UART2_custom_Handler()
{
	uint32_t* UART2_DR = (uint32_t*)0x40004404;
	rx_buf[rx_index] = *UART2_DR;
	rx_index++;

	if(strstr(rx_buf, "on") != 0)
	{
		led_control(0, LED_ON);
		led_control(1, LED_ON);
		led_control(2, LED_ON);
		led_control(3, LED_ON);
		memset(rx_buf, 0, sizeof(rx_buf));
		rx_index = 0;
	}
	else if(strstr(rx_buf, "off") != 0)
	{
		led_control(0, LED_OFF);
		led_control(1, LED_OFF);
		led_control(2, LED_OFF);
		led_control(3, LED_OFF);
		memset(rx_buf, 0, sizeof(rx_buf));
		rx_index = 0;
	}
}
void Flash_to_Ram()
{
	memcpy((void*)0x20000000, 0, 0x198);// copy vector table size 0 -> 0x198
	uint32_t* VTOR = (uint32_t*)0xe000ed08; // enable vector table
	*VTOR = 0x20000000; //in Ram

	// custom exti0
	uint32_t* EXTI0_Address_Custom = (uint32_t*) 0x20000058;
	*EXTI0_Address_Custom = (uint32_t)EXTI0_Custom_Handler | 1;

	//custom Systick
	uint32_t* Systick_address_Custom = (uint32_t*) 0x2000003C;
	*Systick_address_Custom = (uint32_t)Systick_custom_Handler | 1;

	// custom UART2
	uint32_t* UART2_address_Custom = (uint32_t*) 0x200000D8;
	*UART2_address_Custom = (uint32_t)UART2_custom_Handler | 1;
}

void UART_Init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();

	uint32_t* GPIOA_MODER = (uint32_t*)0x40020000;
	*GPIOA_MODER &= ~(0b1111 << 4); // set PIN2,3
	*GPIOA_MODER |= (0b10 << 4) | (0b10 << 6);// Pin2,3 Analog
	uint32_t* GPIOA_AFRL = (uint32_t*)0x40020020;
	*GPIOA_AFRL |= (7 << 8) | (7 << 12);

	// set baud rate 9600
	uint32_t* UART2_BRR = (uint32_t*)0x40004408;
	*UART2_BRR = (8 << 4) | 11;

	//set 13 enable UART, set 2 r/w enable TX, set 3 r/w enable RX
	// set 5 enable Interrupt of UART
	//size 8 byte and check chan le
	uint32_t* UART2_CR1 = (uint32_t*)0x4000440c;
	*UART2_CR1 |= (0b1 << 13) | (0b1 << 2) | (0b1 << 3);// | (0b1 << 5);

	//set enable DMA of UART
	uint32_t* UART2_CR3 = (uint32_t*)0x40004414;
	*UART2_CR3 |= (0b1 << 6);
}

void DMA_Init()
{
	__HAL_RCC_DMA1_CLK_ENABLE();
	// stream 5 enable
	uint32_t* DMA1_S5PAR = (uint32_t*)0x40026090;
	*DMA1_S5PAR = 0x40004404;// read data of UART

	uint32_t* DMA1_S5NDTR = (uint32_t*)0x4002608c;
	*DMA1_S5NDTR = sizeof(rx_buf); // read size of data

	uint32_t* DMA1_S5M0AR = (uint32_t*)0x40026094;
	*DMA1_S5M0AR = (uint32_t)rx_buf;// read on RAM

	// set 25 enable channel 4
	// set 10 increment on 1 ex: var[0], var[1], var[2] ...
	// set 8 Circular mode
	// set 4 read data full the interrupt enable complete
	// set 0 enable DMA
	uint32_t* DMA1_S5CR = (uint32_t*)0x40026088;
	*DMA1_S5CR &= ~(0b111 << 25);
	*DMA1_S5CR |= (4 << 25) | (0b1 << 10)| (0b1 << 8) | (0b1 << 4) | (0b1 << 0);

	// set NVIC interrupt Cortex_M4 DMA
	uint32_t* NVIC_ISER0 = (uint32_t*)0xe000e100;
	*NVIC_ISER0 |= (0b1 << 16);
}

/*
void SPI_Init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();

	uint32_t* GPIOA_MODER = (uint32_t*)0x40020000;
	*GPIOA_MODER &= ~(0b111111 << 10);
	*GPIOA_MODER |= (0b10 << 10) | (0b10 << 12) | (0b10 << 14); // set analog
	uint32_t* GPIOA_AFRL = (uint32_t*)0x40020020;
	*GPIOA_AFRL &= ~(0xfff << 20);
	*GPIOA_AFRL |= (5 << 20) | (5 << 24) | (5 << 28);//set function SPI

	__HAL_RCC_GPIOE_CLK_ENABLE();

	uint32_t* GPIOE_MODER = (uint32_t*)0x40021000;
	*GPIOE_MODER &= ~(0b11 << 6);
	*GPIOE_MODER |= (0b01 << 6); //set output get SS SPI;

	SENSOR_INACTIVE;

	__HAL_RCC_SPI1_CLK_ENABLE();

	uint32_t* SPI1_CR1 = (uint32_t*)0x40013000;
	*SPI1_CR1 |= (0b100 << 3); // set CLK 32
	// set bit 2 MSTR chon MASTER
	// set bit 6 SPE:  enable SPI
	// set bit 8 SSI: Internal slave select
	// set bit 9 SSM: Software slave management
	*SPI1_CR1 |= (0b1 << 2) | (0b1 << 6) | (0b1 << 8) | (0b1 << 9);

}

uint32_t SPI_Sensor_Read(uint32_t cmd)
{
	uint32_t* SPI1_SR = (uint32_t*)0x40013008;
	uint32_t* SPI1_DR = (uint32_t*)0x4001300c;

	SENSOR_ACTIVE;
	while(((*SPI1_SR >> 1) & 1) != 1); // check data
	uint32_t  data_send = cmd | (0b1 << 7);
	*SPI1_DR = data_send;

	while(((*SPI1_SR >> 1) & 1) == 1); // check data empty
	while(((*SPI1_SR >> 0) & 1) != 1);
	while(((*SPI1_SR >> 7) & 1) == 1);

	uint32_t temp = *SPI1_DR;

	while(((*SPI1_SR >> 1) & 1) != 1);
	*SPI1_DR = 0x00; // set CLK
	while(((*SPI1_SR >> 1) & 1) == 1); // check data empty
	while(((*SPI1_SR >> 0) & 1) != 1);
	while(((*SPI1_SR >> 7) & 1) == 1);

	temp = *SPI1_DR;

	SENSOR_INACTIVE;
	return temp;
}
*/

__attribute__((section(".function_in_ram"))) void Erase_Sector(Sector_t sector)
{
	uint32_t* FLASH_SR = (uint32_t*)0x40023c0c;
	uint32_t* FLASH_CR = (uint32_t*)0x40023c10;

	while(((*FLASH_SR >> 16) & 1) == 1);// check Flash memory operation bit 16 BSY
	if((*FLASH_CR >> 31) == 1) //check block
	{
		uint32_t* FLASH_KEYR = (uint32_t*)0x40023c04;
		*FLASH_KEYR = 0x45670123;
		*FLASH_KEYR = 0xCDEF89AB;
	}
	*FLASH_CR &= ~(0xf << 3);
	*FLASH_CR |= (sector << 3) | (0b1 << 1); // set erase bit 3
	*FLASH_CR |= (0b1 << 16); // bit 16 on STRT of check
	while(((*FLASH_SR >> 16) & 1) == 1);
	*FLASH_CR &= ~(0b1 << 16);//bit 16 off STRT of check
}

__attribute__((section(".function_in_ram"))) void Programming(void* addr, char* data, int size)
{
	uint32_t* FLASH_SR = (uint32_t*)0x40023c0c;
	uint32_t* FLASH_CR = (uint32_t*)0x40023c10;

	while(((*FLASH_SR >> 16) & 1) == 1);// check Flash memory operation bit 16 BSY
	if((*FLASH_CR >> 31) == 1) //check block
	{
		uint32_t* FLASH_KEYR = (uint32_t*)0x40023c04;
		*FLASH_KEYR = 0x45670123;
		*FLASH_KEYR = 0xCDEF89AB;
	}

	*FLASH_CR |= (0b1 << 0);// set bit 1 PG

	char* temp_address = addr; //read address on  ram
	for(int i = 0; i < size; i++)
	{
		*temp_address = data[i]; //  address on data
		while(((*FLASH_SR >> 16) & 1) == 1); // check bit 16 BSY
		temp_address++;
	}

	*FLASH_CR &= ~(0b1 << 0);
}
int recv_done = 0;
void DMA1_Stream5_IRQHandler()
{
	recv_done = 1;
	__asm("NOP");

	uint32_t* HIFCR = (uint32_t*)0x4002600C;
	*HIFCR |= (0b1 << 11);
}

__attribute__((section(".function_in_ram"))) void update_sector()
{
	*SYST_CR = 0;
	Erase_Sector(sector_0);
	Programming((void*)0x08000000, rx_buf, sizeof(rx_buf));
	uint32_t* AIRCR = (uint32_t*)0xE000ED0C;// enable reset address
	*AIRCR = (0x5FA << 16) | (0b1 << 2);// on key bit 16 , enable bit 2 reset
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
  DMA_Init();
  SPI_Init();


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
/*
  Erase_Sector(sector_1);
  Programming((void*)0x08004000, msg, sizeof(msg));
*/
  uint32_t sensor_id = SPI_Sensor_Read(WHO_I_AM);
  (void)sensor_id;
  SPI_Sensor_Write(CTRL_REG1, 0b00001111);
  uint32_t sensor_ctrl = SPI_Sensor_Read(CTRL_REG1);
  (void)sensor_ctrl;
  delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
/*	  UART_Send_ARR(msg, sizeof(msg));
	  delay(1000);*/
	  if(recv_done == 1)
	  {
		  update_sector();
	  }
	/*  led_control(0, LED_ON);
	  delay(1000);
	  led_control(0, LED_OFF);
	  delay(1000);*/
	  SPI_Multi_Read(OUT_X_L, data_sensor, 6);
	  x = data_sensor[0];
	  y = data_sensor[1];
	  z = data_sensor[2];
	  delay(1000);
	  //"+IPD,3:on_\r\n"
/*		if(strstr(rx_buf, "+IPD,2:on\r\n") != 0)
		{
			led_control(0, LED_ON);
			led_control(1, LED_ON);
			led_control(2, LED_ON);
			led_control(3, LED_ON);
			//memset(rx_buf, 0, 255);
			UART_Send_ARR(ESP32_CIPSEND, sizeof(ESP32_CIPSEND));

		}
		else if(strstr(rx_buf, "+IPD,2:of\r\n") != 0)
		{

			led_control(0, LED_OFF);
			led_control(1, LED_OFF);
			led_control(2, LED_OFF);
			led_control(3, LED_OFF);
			UART_Send_ARR(ESP32_CIPSEND, sizeof(ESP32_CIPSEND));
			//memset(rx_buf, 0, 255);
		}*/
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
