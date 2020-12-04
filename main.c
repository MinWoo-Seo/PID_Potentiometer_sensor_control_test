/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t Uart3_RX=0;   
int16_t Motor_Position_ADC_Value[100]={0,};

int16_t PC_TX_to_MCU_HexString_Flag=0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX(a, b) (((a) > (b)) ? (b):(a)) 
#define MIN(a, b) (((a) < (b)) ? (b):(a))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#if 1  // printf 사용 함수
#include <stdio.h>
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)  //for printf

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
#endif
void LED_OnOff(){
    HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin ,  GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin ,  GPIO_PIN_RESET);
    HAL_Delay(1000);
}
void Timer3_CH1_PWM(){
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1 );
//htim3.Instance->CCR1=500;  다 됨
__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,1);  //TIM3->CCR1=0; 
HAL_Delay(250);
__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,250);  // TIM3->CCR1=500;
HAL_Delay(250);
__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,500);  // TIM3->CCR1=500;
HAL_Delay(250);
__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,750);  // TIM3->CCR1=500;
HAL_Delay(250);
__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,1000);  // TIM3->CCR1=1000;
HAL_Delay(250);
}
void Timer3_CH2_PWM(){
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2 );
 //htim3.Instance->CCR2=500;  다 됨
__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,1);  //TIM3->CCR2=0;
HAL_Delay(250);
__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,250);  // TIM3->CCR2=500;
HAL_Delay(250);
__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,500);  // TIM3->CCR2=500;
HAL_Delay(250);
__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,750);  // TIM3->CCR2=500;
HAL_Delay(250);
__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,1000);  // TIM3->CCR2=1000;
HAL_Delay(250);
}
void Motor_PWM_CW_Control(){// 정방향
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1 );
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2 );

__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,1);  //TIM3->CCR1=0; 
__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,400);  // TIM3->CCR2=500;
//HAL_Delay(10);
}
void Motor_PWM_CCW_Control(){// 역방향
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1 );
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2 );
__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,400);  //TIM3->CCR1=0; 
__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,1);  // TIM3->CCR2=500;
//HAL_Delay(10);
}
void Uart3_TxRx(){
    HAL_UART_Receive(&huart3,&Uart3_RX, 1, 10);
    HAL_UART_Transmit(&huart3, &Uart3_RX, 1,  10);
    //__HAL_UART_GET_FLAG
}
void ADC1_polling(){
    // char buffer[255];
     uint16_t ADC_data;
     //HAL_Delay(100);
    // for(int i=1; i<150; i++){    
    HAL_ADC_Start(&hadc1);
    ADC_data = HAL_ADC_GetValue(&hadc1);
   // sprintf(buffer,"%d \r\n", ADC_data);
   // HAL_UART_Transmit(&huart3,(uint8_t *)buffer,strlen(buffer),100);
    printf ( "\n \r AD val == % d", ADC_data);
   // HAL_Delay(100); 
   // }
}
void ADC1_polling_Motor_Position_Value(){
   HAL_ADC_Start(&hadc1);
   /*for(int i=1; i<10; i++){  
   Motor_Position_ADC_Value[i] = HAL_ADC_GetValue(&hadc1);
   }*/
   Motor_Position_ADC_Value[0] = HAL_ADC_GetValue(&hadc1);
  // printf( "\n \r AD val == % d", Motor_Position_ADC_Value[0]);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance == TIM6){
     // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
      ADC1_polling();
  }
}
void Motor_Position_Control(int16_t Position_Value){
  ADC1_polling_Motor_Position_Value();
  if(Position_Value>3800){
  Position_Value=3800;
  }
  else if(Position_Value<300){
  Position_Value=300;
 }
  //Position_Value=Motor_Position_ADC_Value[0];
  /*int Motor_Dir;
  Motor_Dir= Position_Value > Motor_Position_ADC_Value[0] ? 1:2;  // CW:1 ,CCW:2
  if(Motor_Dir==1){
  Motor_PWM_CW_Control();// 정방향
  }
  else if(Motor_Dir==2){
  Motor_PWM_CCW_Control();// 역방향
  }*/
  int16_t data=Motor_Position_ADC_Value[0];
  if( (Position_Value+100)<data ){
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_RESET);
  Motor_PWM_CW_Control();// 정방향
  }
  else if((Position_Value-100)>data ){
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_RESET);
  Motor_PWM_CCW_Control();// 역방향
  }
  else{
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2 );
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_SET);
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
  PC_TX_to_MCU_HexString_Flag=0;
   //printf( "\n \r 도달 도달");
  //HAL_Delay(100);
  }
}

//int16_t PC_TX_to_MCU_HexString_Flag=0;
int16_t PC_TX_to_MCU_HexString(uint8_t *data){
  int16_t result;
  int num[4];
  //char string[4]={0,};
  for(int i=0; i<4; i++){
    if('0'<=data[i] && data[i]<='9') {num[i]=data[i]-'0';}
    else if('A'<=data[i] && data[i]<='F'){num[i]=data[i]-'A'+10;}
    else if('a'<=data[i] && data[i]<='f'){num[i]=data[i]-'a'+10;}
    else {num[i]=0;  }}//printf("\n \r알맞은 데이터를 넣어 주세요.\n \r");
  /*
  result = num[0]*0x1000 + num[1]*0x100 + num[2]*0x10 + num[3]*0x1;
  sprintf(string, "%04X 입니다.\n" , (uint16_t)result);*/ // 16진수 표현
  result = num[0]*1000 + num[1]*100 + num[2]*10 + num[3]*1;
  //sprintf(string, "%04d 입니다.\n" , (uint16_t)result);
  //HAL_UART_Transmit(&huart3, (uint8_t *)string, strlen(string),  10);
  PC_TX_to_MCU_HexString_Flag=1;
  return result;  //result 
}

uint8_t data[10]={0,}; //pc에서 들어온 data들을 저장하기 위한 변수
int8_t buffer_count=0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if(huart->Instance ==USART3){
    data[buffer_count] = Uart3_RX;
    buffer_count=buffer_count+1;  
   // printf( "\n \r 도달 도달");
    if(buffer_count>3){ 
      PC_TX_to_MCU_HexString((uint8_t*)data);
      buffer_count=0;      
    }
    HAL_UART_Receive_IT(&huart3,&Uart3_RX, 1);  // 또 한번 초기화 해줘야 함, 그래야 인터럽트가 끝나고 인터럽트가 오면 실행함  // 플래그 또한 초기화 해줌
  }
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  //HAL_ADC_Start_IT(&hadc1);
  HAL_UART_Receive_IT(&huart3,&Uart3_RX, 1);  // 한번 초기화 해줘야함,  // 플래그 또한 초기화 해줌
  printf( "하이 하이 \n \r");
  HAL_TIM_Base_Start_IT(&htim6);  // 인터럽트용 타이 500ms

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    //LED_OnOff();
    //ADC1_polling();
    //Timer3_CH1_PWM();
    //Motor_PWM_CW_Control();
    //Motor_PWM_CCW_Control();
    
    if(PC_TX_to_MCU_HexString_Flag==1){
      Motor_Position_Control(PC_TX_to_MCU_HexString(data));
   // PC_TX_to_MCU_HexString_Flag=0;
  //  LED_OnOff();
    }
    //Motor_Position_Control(300);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  htim6.Init.Prescaler = 8400-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 5000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED0_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED0_Pin LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
