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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define usTIM TIM4
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


//#define TRIG_Pin GPIO_PIN_8
//#define TRIG_GPIO_Port GPIOA
//
//#define ECHO_Pin GPIO_PIN_9
//#define ECHO_GPIO_PortT GPIOA

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void usDelay(uint32_t uSec);
float MeasureDistance(int sensorID);
float trigMeasurement(float preX, float curX, float preY, float curY);
float AveVel(float velArray[]);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//////pin layout//////
/*
 SensorID = 1
 GPIO_PIN_8 (PA8) Trig for sensor 1:
 GPIO_PIN_9 (PA9) Echo for sensor 1:

 SensorID =2;
 GPIO_PIN_6 (PA6) Trig for sensor 2:
 GPIO_PIN_7 (PA7) Echo for sensor 2:

 sensor 1 is the one at the door
 sensor 2 is the one k meters from the door


 GPIO_PIN_10 PIR Sensor:
 */

const float speedOfSound = 0.0343/2; // go and back
int const timemultipler = 4200000;
float const k = 1.0; // distance between two sensors



// bool isPresent=0; // this will be set to whetever is returned by the sensors

// use one of the following
const uint32_t interval = 0.010; // this will be changed to match the clock
uint32_t numTicks= 0; // 1 tick = 1 us



// local array for mean vel calculation
// how many entries do we want? how often do we calculate the velocity
// if we calculate the instant displacement every 50000 ticks (0.05 seconds),



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
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  float preX=0;
  float curX=0;

  float preY=0;
  float curY=0;

  float const interval= 0.01*timemultipler;

  float deltaX=0;

  float aveVel=0;

  int velStage=0;
  int lightStage=0;
  int timer = 0;

  int index=0;
  float xArray[10]={}; // 12 => every 0.6 seconds
  float velArray[9]={}; // every 0.6 seconds we calculate velocity


  uint16_t peopleCounter=0;

  // 1 tick = 1 us
  while (1)
  {
	  timer++;
	  ////////////Test 1///////////
//	  HAL_Delay(500);
//
//	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//
//	  testing_d_1 = MeasureDistance(1);
//	  if (testing_d_1 > 15){
//		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//		  HAL_Delay(3000);
//	  }



	  ////////////Testing2/////////
//
//	  HAL_Delay(500);
//
//	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//
//	  testing_d_2 = MeasureDistance(2);
//	  if (testing_d_2 > 15){
//		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//		  HAL_Delay(3000);
//	  }
//




	  ////////////Testing3/////////
	  // stage machine to calculate average velocity
	  switch(velStage) {
	    case 0:
	      preX = MeasureDistance(1);
		  preY = MeasureDistance(2);
		  if (timer>interval){ // every 0.01 seconds;
			  velStage++;
			  timer=0;
		      break;
		  }

	    case 1:
		  curX = MeasureDistance(1);
		  curY = MeasureDistance(2);
		  break;

	    case 2:
	    	deltaX=trigMeasurement(preX, curX, preY, curY); //dX
	    	curX=preX;
	    	curY=preY;
	    	velStage++;
	    	break;

	    case 3: // add instant x to array
			xArray[index] = deltaX; // dX
			index++;
			velStage=0;

			if (index==10){
				velStage=4;
				index=0;
				break;
			}

	    case 4: // add vel to the array
	    	velArray[index]= (xArray[index+1]-xArray[index])/(interval);// calculate dV; derivative
	    	index++;

	    	if (index==9){
	    		velStage++;
	    		index=0;
	    		break;
	    	}


	    case 5:
	    	aveVel = AveVel(velArray);
	    	velStage++;
	    	break;
	  }




	  // light state machine
//	  if (aveVel>0){ // approaching the door
//		  switch(lightStage) {
//		    case 0:
//		    	if (30.0 < Measurement(3)){	 // this should give the third distance
//		    		peopleCounter--;
//		    		lightStage++;
//		    	}
//
//		  }
//	  }






	  //




	  // HAL_Delay(150);


	  // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	  // testing_d_2 = MeasureDistance(2);
	  //if (testing_d_2 > 10){
//		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//		  HAL_Delay(3000);
//	  }




	  /*
	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)==GPIO_PIN_SET){ // when detecting people

	  }
	  */








/*
	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)==GPIO_PIN_SET){
		  MeasureDistance();
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_SET);
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_RESET);
	  }
*/





	 /////////////////////////////////////////


	  // this will be where our main logic sits






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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|ECHO2_Pin|TRIG1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin ECHO2_Pin TRIG1_Pin */
  GPIO_InitStruct.Pin = LED_Pin|ECHO2_Pin|TRIG1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG2_Pin */
  GPIO_InitStruct.Pin = TRIG2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TRIG2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO2C9_Pin */
  GPIO_InitStruct.Pin = ECHO2C9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO2C9_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PIR_Pin */
  GPIO_InitStruct.Pin = PIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PIR_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void usDelay(uint32_t uSec)
{
	if(uSec < 2) uSec = 2;
	usTIM->ARR = uSec - 1; 	/*sets the value in the auto-reload register*/
	usTIM->EGR = 1; 			/*Re-initialises the timer*/
	usTIM->SR &= ~1; 		//Resets the flag
	usTIM->CR1 |= 1; 		//Enables the counter
	while((usTIM->SR&0x0001) != 1);
	usTIM->SR &= ~(0x0001);
}

// SensorID = 1
// GPIO_PIN_8 Trig for sensor 1:
// GPIO_PIN_9 Echo for sensor 1:

// SensorID =2;
// GPIO_PIN_6 Trig for sensor 2
// GPIO_PIN_7 Echo for sensor 2:
float MeasureDistance(int sensorID){
	float localDistance=0;
	if (sensorID == 1){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		usDelay(3);

		//*** START Ultrasonic measure routine ***//
		//1. Output 10 usec TRIG
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		usDelay(10);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

		//2. Wait for ECHO pin rising edge
		while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_RESET);

		//3. Start measuring ECHO pulse width in usec
		numTicks = 0;
		while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET)
		{
			numTicks++;
			usDelay(2); //2.8usec
		};


		localDistance = (numTicks + 0.0f)*2.8*speedOfSound; // Speed of sound is already divided by 2 here.


		return localDistance;
	}

	if (sensorID == 2){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
		usDelay(3);

		//*** START Ultrasonic measure routine ***//
		//1. Output 10 usec TRIG
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
		usDelay(10);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

		//2. Wait for ECHO pin rising edge
		while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET);

		//3. Start measuring ECHO pulse width in usec
		numTicks = 0;
		while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET)
		{
			numTicks++;
			usDelay(2); //2.8usec
		};


		localDistance = (numTicks + 0.0f)*2.8*speedOfSound; // Speed of sound is already divided by 2 here.


		return localDistance;
	}



	// This is the testing code
	/*
	if(localDistance > 20){
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		HAL_Delay(3000);
	}
	*/
	return localDistance;

}


//// Description
// This function will calculate the speed of the object using cosine law
// It takes 4 parameters from MeasureDistance function: preX, curX, preY, curY
// X is the measurement of sensor 1 from the door
// Y is the measurement of sensor 2 on the wall

// check which part is which and double check the algorithm
// Do we need to calculate both at the same time or split into two function calls?
// int counter=0;



float trigMeasurement(float preX, float curX, float preY, float curY){
	// k is the distance between two sensors pair
	//+++++++++++++++++++++++++++++++++++++++++++++
	float local_deltaX=0;
	float preDistance=0;
	float curDistance=0;
	float preData=0;
	float curData=0;
	float delta1=0;
	float delta2=0;


	preData = -((preX*preX)-(preY*preY)-(k*k))/(preY*k);
	delta1 = acos(preData);
	preDistance = preY*cos(delta1);


	curData = -((curX*curX)-(curY*curY)-(k*k))/(curY*k);
	delta2 = acos(curData);
	curDistance = curY*cos(delta2);

	// calculate delta x
	local_deltaX = curDistance - preDistance;

	return local_deltaX;

}





// this function will calculate the average velocity of an object over an interval
float AveVel(float velArray[]){ // size 9

	float sum=0;
	float aveVel=0;
	for (int i=0; i<9;i++){
		sum += velArray[i];
	}

	aveVel = sum/(interval*10); // mean value
	return aveVel;





	//+++++++++++++++++++++++++++++++++++++++++++++
	// Problem is that first time some entries are still at their default values
	// float velArray[i]=(xArray[i+1]-xArray[i])/interval; // this is the idea of derivative where lim(f(x+h)-f(x))/h as h which is our interval approaches 0
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
