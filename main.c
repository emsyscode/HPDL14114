/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
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
 SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// The following #define statements have been taken directly from the datasheet

  /****************/
 /* CONTROL BYTE */
/****************/
// Slave Address      XXXX      (number beneath X's)
#define SLAVE_ADDR  0b01000000

// Wired Logic Address    XXX   (number beneath X's)
#define LOGIC_ADDR  0b00000000

// Read/Write Command        X  (number beneath X's)
#define CTL_WRITE  0b00000000
#define CTL_READ  0b00000001

/**********************/
/* REGISTER ADDRESSES */
/**********************/
uint8_t REG_IODIRA = 0x00;
uint8_t REG_IODIRB = 0x01;
uint8_t REG_GPIOA = 0x12;
uint8_t REG_GPIOB = 0x13;
uint8_t REG_IOCON_A = 0x0A;
uint8_t REG_IOCON_B = 0x0B;

uint8_t i = 0x00;
uint8_t n = 0x00;
uint8_t seq = 0x00;
uint8_t dsp = 0x00;
uint8_t symb = 0x00;
uint8_t cnt = 0x00;
uint8_t row = 0x00;
uint8_t addr = 0x00;

 /*************/
/* GPIO MASK */
/*************/
// This bitmask will set all 8 I/O pins of one bank to output (0)
uint8_t MASK_ALL_OUTPUT = 0x00;

// This operation takes all three elements and merges them together (i.e. 01000000)
uint8_t control_byte = (SLAVE_ADDR | LOGIC_ADDR | CTL_WRITE);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  //Set of both pins of /WR of HPDL1414 1 & 2
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

  	          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
  	          HAL_Delay(100);
  	          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
  	          HAL_Delay(100);
  	          //
  	          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
  	          HAL_Delay(100);
  	          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
  	          HAL_Delay(100);
  	          //
  	         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
  	         HAL_Delay(100);
  	         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
  	         HAL_Delay(100);
  	         //
  	  	initOfHPDL1414();
  	  	clearHPDL1414();
  	  	testOfHPDL1414();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint8_t myRxbuffer[8] ={0};
	  uint8_t Rxbuffer = 0x00;

	  //writeDSP(uint8_t digit, uint8_t letter, uint8_t group)
	  //The marked point of 1 is the left side of word
	  writeDSP(0x03, 0x48, 0x01); // Digit, Letter, first or second HPDL1414
	  writeDSP(0x02, 0x49, 0x01);
	  writeDSP(0x01, 0x20, 0x01);
	  writeDSP(0x00, 0x46, 0x01);
	  writeDSP(0x03, 0x4F, 0x00);
	  writeDSP(0x02, 0x4C, 0x00);
	  writeDSP(0x01, 0x4B, 0x00);
	  writeDSP(0x00, 0x53, 0x00);
	  //
	  HAL_Delay(500);
	  writeDSP(0x03, 0x20, 0x01);
	  writeDSP(0x02, 0x20, 0x01);
	  writeDSP(0x01, 0x20, 0x01);
	  writeDSP(0x00, 0x20, 0x01);
	  writeDSP(0x03, 0x20, 0x00);
	  writeDSP(0x02, 0x20, 0x00);
	  writeDSP(0x01, 0x20, 0x00);
	  writeDSP(0x00, 0x20, 0x00);
	  //
	  HAL_Delay(500);
	  writeDSP(0x03, 0x48, 0x01); // Digit, Letter, first or second HPDL1414
	  writeDSP(0x02, 0x49, 0x01);
	  writeDSP(0x01, 0x20, 0x01);
	  writeDSP(0x00, 0x46, 0x01);
	  writeDSP(0x03, 0x4F, 0x00);
	  writeDSP(0x02, 0x4C, 0x00);
	  writeDSP(0x01, 0x4B, 0x00);
	  writeDSP(0x00, 0x53, 0x00);
	  //
	  HAL_Delay(500);
	  writeDSP(0x03, 0x20, 0x01);
	  writeDSP(0x02, 0x20, 0x01);
	  writeDSP(0x01, 0x20, 0x01);
	  writeDSP(0x00, 0x20, 0x01);
	  writeDSP(0x03, 0x20, 0x00);
	  writeDSP(0x02, 0x20, 0x00);
	  writeDSP(0x01, 0x20, 0x00);
	  writeDSP(0x00, 0x20, 0x00);
	  //
	  HAL_Delay(500);
	  writeDSP(0x03, 0x48, 0x01); // Digit, Letter, first or second HPDL1414
	  writeDSP(0x02, 0x49, 0x01);
	  writeDSP(0x01, 0x20, 0x01);
	  writeDSP(0x00, 0x46, 0x01);
	  writeDSP(0x03, 0x4F, 0x00);
	  writeDSP(0x02, 0x4C, 0x00);
	  writeDSP(0x01, 0x4B, 0x00);
	  writeDSP(0x00, 0x53, 0x00);
	  //
	  HAL_Delay(500);
	  writeDSP(0x03, 0x20, 0x01);
	  writeDSP(0x02, 0x20, 0x01);
	  writeDSP(0x01, 0x20, 0x01);
	  writeDSP(0x00, 0x20, 0x01);
	  writeDSP(0x03, 0x20, 0x00);
	  writeDSP(0x02, 0x20, 0x00);
	  writeDSP(0x01, 0x20, 0x00);
	  writeDSP(0x00, 0x20, 0x00);
	  //
	  HAL_Delay(500);
	  writeDSP(0x03, 0x48, 0x01);
	  writeDSP(0x02, 0x50, 0x01);
	  writeDSP(0x01, 0x44, 0x01);
	  writeDSP(0x00, 0x4C, 0x01);
	  writeDSP(0x03, 0x31, 0x00);
	  writeDSP(0x02, 0x34, 0x00);
	  writeDSP(0x01, 0x31, 0x00);
	  writeDSP(0x00, 0x34, 0x00);
	  //
	  HAL_Delay(2000);
	  //StartDefaultTask(0xAA);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void StartDefaultTask(void const * argument)
{
 uint8_t leds[4] = {0xFF, 0x0F, 0xF0, 0x00};
 /* Infinite loop */
 for(;;)
 {
 HAL_SPI_Transmit(&hspi1, leds, 1, 1);
 GPIOB -> ODR &= ~GPIO_PIN_6;
 GPIOB -> ODR |= GPIO_PIN_6;
 HAL_Delay(500);
 HAL_SPI_Transmit(&hspi1, &leds[3], 1, 1);
 GPIOB -> ODR &= ~GPIO_PIN_6;
 GPIOB -> ODR |= GPIO_PIN_6;
 HAL_Delay(500);
 }
}

void initOfHPDL1414(){
	// Wait for 50 ms
	  	  	  HAL_Delay(50);

	         //Note is very important define if the is in sequence mode or byte mode
	         //of working configuration of IOCON_port
	         // Setting of IOCON_A
	         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);  //CS to MCP23S17
	         HAL_Delay(1);

	         HAL_SPI_Transmit(&hspi1, &control_byte, 1, 1);
	         HAL_SPI_Transmit(&hspi1, &REG_IOCON_A, 1, 1); //Define IOCON of port A
	         HAL_SPI_Transmit(&hspi1, &MASK_ALL_OUTPUT, 1, 1); //All as zero of port A

	         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);  //CS to MCP23S17
	         HAL_Delay(1);
	         //END of IOCON setting's
	         // Setting of IOCON_B
	         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);  //CS to MCP23S17
	         HAL_Delay(1);

	         HAL_SPI_Transmit(&hspi1, &control_byte, 1, 1);
	         HAL_SPI_Transmit(&hspi1, &REG_IOCON_B, 1, 1); //Define IOCON of port B
	         HAL_SPI_Transmit(&hspi1, &MASK_ALL_OUTPUT, 1, 1); //All as zero of port B

	         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);  //CS to MCP23S17
	         HAL_Delay(1);
	         //END of IOCON setting's

	         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);  //CS to MCP23S17
	         HAL_Delay(1);

	         HAL_SPI_Transmit(&hspi1, &control_byte, 1, 1);
	         HAL_SPI_Transmit(&hspi1, &REG_IODIRA, 1, 1);  //Define input/output of port A
	         HAL_SPI_Transmit(&hspi1, &MASK_ALL_OUTPUT, 1, 1); //All as output pins of port A

	         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);  //CS to MCP23S17
	         HAL_Delay(1);

	         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);  //CS to MCP23S17
	         HAL_Delay(1);

	         HAL_SPI_Transmit(&hspi1, &control_byte, 1, 1);
	         HAL_SPI_Transmit(&hspi1, &REG_IODIRB, 1, 1); //Define input/output of port B
	         HAL_SPI_Transmit(&hspi1, &MASK_ALL_OUTPUT, 1, 1);  //All as output pins of port B

	         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);  //CS to MCP23S17
	         for(uint32_t i = 0; i < 85; i++)    //delay
	                 {
	        	 i++;
	                 }
}
void testOfHPDL1414(){
	//This is three cycles to control the display HPDL1414
	//The pin 1 is under the chanfre of glass... the pin 6 is VCC
	//The pin number 7 is over the pin 6 is the GND!
	//The datasheet refer the DSP consuming 130mA
	//Is necessary control the second word of data, because skip bit's !!!
	uint8_t io_pin = 0;
	unsigned short bitmask = 0x0000;  // represents two bytes
	unsigned char *spi_byte_array;
	// Point byte array at the bitmask
	spi_byte_array = (unsigned char *)&bitmask;

	for(dsp = 0; dsp < 8; dsp++){
		for(row = 0x00; row < 4; row++){
			for(cnt = 0x00; cnt < 16; cnt++){
				switch (row) {
					case 0: seq = 0x20;
							symb = (seq | cnt); break;
					case 1: seq = 0x30;
							symb = (seq | cnt); break;
					case 2: seq = 0x40;
							symb = (seq | cnt); break;
					case 3: seq = 0x50;
							symb = (seq | cnt); break;
				}

			spi_byte_array[0] =  dsp; //Port A of MCP
		    spi_byte_array[1] = symb; //Port B of MCP

		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);  //CS to MCP23S17
		  //HAL_Delay(0.1);
		  HAL_SPI_Transmit(&hspi1, &control_byte, 1 ,1);
		  HAL_SPI_Transmit(&hspi1, &REG_GPIOA, 1, 1);

		  HAL_SPI_Transmit(&hspi1, &spi_byte_array[0], 1 ,1);
		  HAL_SPI_Transmit(&hspi1, &spi_byte_array[1], 1 ,1);

		  //HAL_SPI_Transmit(&hspi1, 0x80, 1, 1);
		  //HAL_SPI_Transmit(&hspi1, n, 1 ,1);

		  //HAL_Delay(0.1);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);  //CS to MCP23S17

		  //Write to HDPL1414
		  if(dsp >=4){
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		  }
		  else{
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		  }

		  	  // Wait for the human eye to catch up
		  	  HAL_Delay(100);

			}
		}
	}
}

void writeDSP(uint8_t digit, uint8_t letter, uint8_t group){
	uint8_t outputBuffer = 0x20;
	//The first argument is a pointer to the SPI instance.
	//The second argument is a pointer to the data to be sent (uint8_t).
	//The third argument is the size of the data in bytes.
	//Last argument is the timeout, a value of 1 should do the job just fine.
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);  //CS to MCP23S17
	  	//
	  	HAL_SPI_Transmit(&hspi1, &control_byte, 1 ,1);
	  	HAL_SPI_Transmit(&hspi1, &REG_GPIOA, 1, 1);
	  	outputBuffer = digit;
	  	HAL_SPI_Transmit(&hspi1, &outputBuffer, 1, 1);
	  	//
	  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);  //CS to MCP23S17

	  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);  //CS to MCP23S17
	  	//
	  	HAL_SPI_Transmit(&hspi1, &control_byte, 1 ,1);
	  	HAL_SPI_Transmit(&hspi1, &REG_GPIOB, 1, 1);
	  	outputBuffer = letter;
	  	HAL_SPI_Transmit(&hspi1, &outputBuffer, 1, 1);
	  	//
	  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);  //CS to MCP23S17
//Set the /WR of first or second HPDL1414
if(group == 0){
	  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
}
else{
	  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}
}
void clearHPDL1414(){
		  writeDSP(0x03, 0x20, 0x01);
		  writeDSP(0x02, 0x20, 0x01);
		  writeDSP(0x01, 0x20, 0x01);
		  writeDSP(0x00, 0x20, 0x01);
		  writeDSP(0x03, 0x20, 0x00);
		  writeDSP(0x02, 0x20, 0x00);
		  writeDSP(0x01, 0x20, 0x00);
		  writeDSP(0x00, 0x20, 0x00);
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
