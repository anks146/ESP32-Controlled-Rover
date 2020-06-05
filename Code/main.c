/* USER CODE BEGIN Header */
/**
  ***************************	***************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
	
char RData[10];
int Count_Rec=0;
int rec_count=0;
char v[10];
uint8_t Rover_Direction;
int PWM_Pulse,PWM_Value_Motor_1,PWM_Value_Motor_2,PWM_Value_Motor_3,PWM_Value_Motor_4;
int PWM_Previous_Value_Motor_1,PWM_Previous_Value_Motor_2,PWM_Previous_Value_Motor_3,PWM_Previous_Value_Motor_4;
TIM_ClockConfigTypeDef sClockSourceConfig = {0};
TIM_MasterConfigTypeDef sMasterConfig = {0};
TIM_OC_InitTypeDef sConfigOC = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
//----------------------------------------------------------------
void Delay(int);	
// Speed Related Functions
void Update_Speed(int Channel,int Speed);
void Update_Speed_All(void);
void PWM_Start(void);
void PWM_Stop(void);
void Rover_Fast(void);
void Rover_Slow(void);
void Rover_Left(void);
void Rover_Right(void);
void Save_Speed(void);
void Load_Speed(void);
//----------------------------------------------------------------
// Direction Related Functions
void Motor_Forward(int Motor);
void Motor_Backward(int Motor);
void Rover_Forward(void);
void Rover_Reverse(void);
void Rover_Stop(void);
void Set_Equal_Speed(int Option);
//----------------------------------------------------------------
// Serial COM Functions
void Receive_Data(int);
int Receive_String(int,char *);
void Uart_Interrupt_Enable(void);
void Tx_data(char c);
void Sendserial(char *s);
void Interrupt_Sync(void);
void UART5_IRQHandler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Delay(int t)
{
	t=t*4;
	__HAL_TIM_SET_COUNTER(&htim3,0);
	while(__HAL_TIM_GET_COUNTER(&htim3)<t);
	return;
}
//==================== SERIAL COM BEGIN ==========================
//----------------------------------------------------------------
void Interrupt_Sync(void)
{
	if(rec_count%2==0)
		rec_count++;
	
	NVIC_EnableIRQ(UART5_IRQn);
	return;
}	
void Uart_Interrupt_Enable(void)
{
	UART5->CR1|=(0X20);
	return;
}
//----------------------------------------------------------------
void Tx_data(char c)
{
	while((UART5->ISR&(0X40))!=0X40);
	UART5->TDR=c;
	
	return;
}
//----------------------------------------------------------------
void Sendserial(char *s)
{
	int k=0;
	while(s[k]!='\0')
	{
		Tx_data(s[k]);
		k++;
	}
	return;
}
//----------------------------------------------------------------
/*
void Receive_data(int number)
{
	int j=0;
	Count_Rec=0;
	for(j=0;j<number;j++)
	{
		while((UART5->ISR&(0X20))!=0X20);
		RData[Count_Rec]=UART5->RDR;
		Count_Rec++;
		if(Count_Rec==10)
		{
		Count_Rec=0;
		}
	}
}
*/
//----------------------------------------------------------------
int Receive_String(int number,char *str)
{
	int j=0,check=0;
	Count_Rec=0;
	while(j<number)
	{
		while((UART5->ISR&(0X20))!=0X20);
		RData[Count_Rec]=UART5->RDR;
		Count_Rec++;
		j++;
		if(Count_Rec==10)
		{
		Count_Rec=0;
		}
	}
	for(j=0;j<number;j++)
	{
		if(RData[j]==str[j])
			check++;
	}
	if(check==number)
	{
		Sendserial("@T");
		return 1;
	}
	else 
	{
	  Sendserial("@F");
		return 0;
	}
}
//----------------------------------------------------------------
//===================== SERIAL COM END ===========================
//----------------------------------------------------------------
void PWM_Start(void)
{
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
		return;
}
//----------------------------------------------------------------
void PWM_Stop(void)
{
		HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
		HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_4);
		return;
}
//----------------------------------------------------------------
void Set_Equal_Speed(int Option)
{
	if(Option==Fastest)
	{
		if(PWM_Value_Motor_1<=PWM_Value_Motor_2)
			PWM_Value_Motor_1=PWM_Value_Motor_2;
		else 
			PWM_Value_Motor_2=PWM_Value_Motor_1;

		if(PWM_Value_Motor_3<=PWM_Value_Motor_4)
			PWM_Value_Motor_3=PWM_Value_Motor_4;
		else 
			PWM_Value_Motor_4=PWM_Value_Motor_3;		
		
		if(PWM_Value_Motor_3<=PWM_Value_Motor_1)
		{
			PWM_Value_Motor_3=PWM_Value_Motor_1;
			PWM_Value_Motor_4=PWM_Value_Motor_1;
			PWM_Pulse=PWM_Value_Motor_1;
		}
		else 
		{
			PWM_Value_Motor_1=PWM_Value_Motor_3;
			PWM_Value_Motor_2=PWM_Value_Motor_3;
			PWM_Pulse=PWM_Value_Motor_3;
		}
	}
	
	else if(Option == Slowest)
	{
		if(PWM_Value_Motor_1<=PWM_Value_Motor_2)
			PWM_Value_Motor_2=PWM_Value_Motor_1;
		else 
			PWM_Value_Motor_1=PWM_Value_Motor_2;

		if(PWM_Value_Motor_3<=PWM_Value_Motor_4)
			PWM_Value_Motor_4=PWM_Value_Motor_3;
		else 
			PWM_Value_Motor_3=PWM_Value_Motor_4;		
		
		if(PWM_Value_Motor_3<=PWM_Value_Motor_1)
		{
			PWM_Value_Motor_1=PWM_Value_Motor_3;
			PWM_Value_Motor_2=PWM_Value_Motor_3;
			PWM_Pulse=PWM_Value_Motor_3;
		}
		else 
		{
			PWM_Value_Motor_3=PWM_Value_Motor_1;
			PWM_Value_Motor_4=PWM_Value_Motor_1;
			PWM_Pulse=PWM_Value_Motor_1;
		}
	}
		return;
}
//----------------------------------------------------------------
void Save_Speed(void)
{
	  PWM_Previous_Value_Motor_1=PWM_Value_Motor_1;
		PWM_Previous_Value_Motor_2=PWM_Value_Motor_2;
		PWM_Previous_Value_Motor_3=PWM_Value_Motor_3;
		PWM_Previous_Value_Motor_4=PWM_Value_Motor_4;
		return;
}
//----------------------------------------------------------------
void Load_Speed(void)
{
		PWM_Value_Motor_1 = PWM_Previous_Value_Motor_1;
		PWM_Value_Motor_2 = PWM_Previous_Value_Motor_2;
		PWM_Value_Motor_3 = PWM_Previous_Value_Motor_3;
		PWM_Value_Motor_4=  PWM_Previous_Value_Motor_4;
		return;
}
//----------------------------------------------------------------
void Update_Speed(int Channel,int Speed)
{
		if(Channel==1)
			{
				if(PWM_Value_Motor_1!=Speed)
					{
						PWM_Value_Motor_1=Speed;
						TIM2->CCR1=Speed;
					}
					else
						TIM2->CCR1=Speed;
						
			}
		else if(Channel==2)
			{
				if(PWM_Value_Motor_2!=Speed)
					{
						PWM_Value_Motor_2=Speed;
						TIM2->CCR2=Speed;
					}
					else
						TIM2->CCR2=Speed;
			}
		
		else if(Channel==3)
			{
				if(PWM_Value_Motor_3!=Speed)
					{
						PWM_Value_Motor_3=Speed;
						TIM2->CCR3=Speed;
					}
					else
						TIM2->CCR3=Speed;
			}
		
		else if(Channel==4)
			{
				if(PWM_Value_Motor_4!=Speed)
					{
						PWM_Value_Motor_4=Speed;
						TIM2->CCR4=Speed;
					}
					else
						TIM2->CCR4=Speed;
			}
		return;
		}
//----------------------------------------------------------------
void Update_Speed_All(void)
{
		Update_Speed(1,PWM_Value_Motor_1);
		Update_Speed(2,PWM_Value_Motor_2);
		Update_Speed(3,PWM_Value_Motor_3);
		Update_Speed(4,PWM_Value_Motor_4);
		return;
}
//----------------------------------------------------------------
void Rover_Fast(void)
{

		if(PWM_Pulse<Max_PWM_Value) // Check if max Speed attained
			{
				Set_Equal_Speed(Fastest); // Set speed of all motor equal to speed of the fastest motor
				
				PWM_Pulse=PWM_Pulse+PWM_Speed_Variation;
				
				PWM_Value_Motor_1=PWM_Pulse;
				PWM_Value_Motor_2=PWM_Pulse;
				PWM_Value_Motor_3=PWM_Pulse;
				PWM_Value_Motor_4=PWM_Pulse;
				
				Update_Speed_All();
				Delay(50);
				TIM2->EGR|=0x7F; // Update Generation Event occurs
				__HAL_TIM_SET_PRESCALER(&htim2,Prescalar);
			}
			else 
				Sendserial("Speed Fastest\r\n");
		return;
}
//----------------------------------------------------------------
void Rover_Slow(void)
{
		if(PWM_Pulse>Min_PWM_Value) // Check if max Speed attained
			{
				PWM_Stop();
				Set_Equal_Speed(Slowest);// Set speed of all motor equal to speed of the slowest motor

				PWM_Pulse-=PWM_Speed_Variation;

				PWM_Value_Motor_1=PWM_Pulse;
				PWM_Value_Motor_2=PWM_Pulse;
				PWM_Value_Motor_3=PWM_Pulse;
				PWM_Value_Motor_4=PWM_Pulse;
				
				Update_Speed_All();	
				Delay(50);
				
				TIM2->EGR|=0x7F; // Update Generation Event occurs
				__HAL_TIM_SET_PRESCALER(&htim2,Prescalar);
				PWM_Start();
			}
			else 
				Sendserial("Speed Lowest\r\n");
			return;
}
//----------------------------------------------------------------
void Rover_Left(void)
	{
		Save_Speed();
		PWM_Pulse=Turning_Speed;
		PWM_Value_Motor_1=Turning_Speed;
		PWM_Value_Motor_2=Turning_Speed;
		PWM_Value_Motor_3=Turning_Speed;
		PWM_Value_Motor_4=Turning_Speed;
		Update_Speed_All();
	
		if(Rover_Direction==Reverse)
		{
			Motor_Backward(1);
			Motor_Backward(2);
			Motor_Forward(3);
			Motor_Forward(4);
		}
		else if(Rover_Direction==Forward)
		{
			Motor_Forward(1);
			Motor_Forward(2);
			Motor_Backward(3);
			Motor_Backward(4);
		}
		else
			return;
		
		Delay(Turning_Time);
		
		if(Rover_Direction==Forward)
			Rover_Forward();
		else
			Rover_Reverse();
		Rover_Stop();
		Load_Speed();
		return;
}
//----------------------------------------------------------------
void Rover_Right(void)
{
		Save_Speed();
		PWM_Pulse=Turning_Speed;
		PWM_Value_Motor_1=Turning_Speed;
		PWM_Value_Motor_2=Turning_Speed;
		PWM_Value_Motor_3=Turning_Speed;
		PWM_Value_Motor_4=Turning_Speed;
		Update_Speed_All();
	
		if(Rover_Direction==Forward)
			{
				Motor_Backward(1);
				Motor_Backward(2);
				Motor_Forward(3);
				Motor_Forward(4);
			}
		else if(Rover_Direction==Reverse)
			{
				Motor_Forward(1);
				Motor_Forward(2);
				Motor_Backward(3);
				Motor_Backward(4);
			}
		else
				return;
		
		Delay(Turning_Time);
		
		if(Rover_Direction==Forward)
			Rover_Forward();
		else
			Rover_Reverse();
		Rover_Stop();
		Load_Speed();
		return;
}

// Direction Control
//----------------------------------------------------------------
void Motor_Forward(int Motor)
{
	if (Motor==1)
		{	
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
		}
	else if(Motor==2)
		{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
		}
	else if(Motor==3)
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
		}
	else if(Motor==4)
		{
			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_7,GPIO_PIN_RESET);

		}
		return;
}
//----------------------------------------------------------------
void Motor_Backward(int Motor)
{
	if (Motor==1)
		{	
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
		}
	else if(Motor==2)
		{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
		}
	else if(Motor==3)
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
		}
	else if(Motor==4)
		{
			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_7,GPIO_PIN_SET);
		}
		return;
}
//----------------------------------------------------------------
void Rover_Forward(void)
{
		Motor_Forward(1);
		Motor_Forward(2);
		Motor_Forward(3);
		Motor_Forward(4);
		Rover_Direction=Forward;
		return;
}
//----------------------------------------------------------------
void Rover_Reverse(void)
{
		Motor_Backward(1);
		Motor_Backward(2);
		Motor_Backward(3);
		Motor_Backward(4);
		Rover_Direction=Reverse;
		return;
}
void Rover_Stop(void)
	{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_7,GPIO_PIN_RESET);
			return;
}
//----------------------------------------------------------------
/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */
				NVIC_DisableIRQ(UART5_IRQn);
		if((UART5->ISR&0x20)==0x20) // Ceck source of interrupt
		{
			v[rec_count]=UART5->RDR;
			
			if(/*(v[rec_count-1]=='#') && */((v[rec_count] =='F')||(v[rec_count] =='f')))
			{
				Sendserial("Forward\r\n");
				Rover_Forward();
				Interrupt_Sync();
			}
			else if(/*(v[rec_count-1]=='#') &&*/ ((v[rec_count] =='B')||(v[rec_count] =='b')))
			{
				Sendserial("Backward\r\n");
				Rover_Reverse();
				Interrupt_Sync();
			}
			else if(/*(v[rec_count-1]=='#') && */((v[rec_count] =='L')||(v[rec_count] =='l')))
			{
				Sendserial("Left\r\n");
				Rover_Left();
				Interrupt_Sync();
			}
			else if(/*(v[rec_count-1]=='#') && */((v[rec_count] =='R')||(v[rec_count] =='r')))
			{
				Sendserial("Right\r\n");
				Rover_Right();
				Interrupt_Sync();
			}
			else if(/*(v[rec_count-1]=='#') && */((v[rec_count] =='U')||(v[rec_count] =='u')))
			{
				Sendserial("Speed up\r\n");
				Rover_Fast();
				Interrupt_Sync();
			}
			else if(/*(v[rec_count-1]=='#') && */((v[rec_count] =='D')||(v[rec_count] =='d')))
			{
				Sendserial("Speed down\r\n");
				Rover_Slow();
				Interrupt_Sync();
			}
			else if(/*(v[rec_count-1]=='#') && */((v[rec_count] =='S')||(v[rec_count] =='s')))
			{
				Sendserial("Stop\r\n");
				Rover_Stop();
				Interrupt_Sync();
			}
			
			rec_count++;
			if(rec_count==10)
				rec_count=0;
			NVIC_EnableIRQ(UART5_IRQn);

		}
  /* USER CODE END UART5_IRQn 1 */
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	PWM_Pulse=550;
	PWM_Value_Motor_1=PWM_Pulse;
	PWM_Value_Motor_2=PWM_Pulse;
	PWM_Value_Motor_3=PWM_Pulse;
	PWM_Value_Motor_4=PWM_Pulse;
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim3); // Start delay timer
	Sendserial("Hello\r\n");
	PWM_Start();
//	Delay(10000);
	Rover_Forward();
//  while(!Receive_String(3,"#F@")); // Hold till function received successfully
//	Sendserial("Character received successfully\r\n");
	Uart_Interrupt_Enable();
	NVIC_EnableIRQ(UART5_IRQn);
	Sendserial("Awaiting Instructions\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable 
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART5;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
	TIM2->CR1|=(1<<7);
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 49999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 57600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_TXINVERT_INIT|UART_ADVFEATURE_RXINVERT_INIT;
  huart5.AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
  huart5.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PF6 PF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
