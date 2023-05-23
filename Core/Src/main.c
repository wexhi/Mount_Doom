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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "niming.h"
#include "pid.h"
#include <stdio.h>
#include <string.h>
#include "cJSON.h"
#include <math.h>
#include "bsp_oled.h"
#include "Header.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_SPEED 4.5
#define MIN_SPEED 0
#define PI 3.141592653589793
#define LED1_ON() HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define LED1_OFF() HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define LED1_TOGGLE() HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

short Encoder1Count = 0;
short Encoder2Count = 0;
uint8_t str[256];

float Motor1Speed = 0.00;
float Motor2Speed = 0.00;
int Motor1Pwm = 0;
int Motor2Pwm = 0;

uint16_t Timer1Count = 0;

extern tPid pidMotor1Speed, pidMotor2Speed, pidMPU6050YawMovement, pidMPU6050PitchMovement;
extern Car wheel1, wheel2;

extern uint8_t usart1_ReadBuf[256];
extern uint8_t usart1_ReadBufCount;

float p, i, d, a;

uint8_t oled_str[] = { "I LOVE CY" };\
uint8_t OledString[30];

float Mileage = 0.00;

extern float Roll, Pitch, Yaw; 

float g_fMPU6050YawMovePidOut = 0, g_fMPU6050YawMovePidOut1 = 0, g_fMPU6050YawMovePidOut2 = 0,
	  g_fMPU6050PitchMovePidOut = 0, g_fMPU6050PitchMovePidOut1 = 0, g_fMPU6050PitchMovePidOut2 = 0;

float MAX_Roll = 0,	Target_Yaw = 0.0001;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Show_Info()
{
		sprintf((char*)str,
		"Motor1Speed : %2f Motor2Speed : %2f\r\n Motor1Pwm : %d Motor2Pwm : %d\r\n", 
		 Motor1Speed,	   Motor2Speed,		 Motor1Pwm,		Motor1Pwm);
	HAL_UART_Transmit(&huart1, str, sizeof(str), 100);
}

void Show_Info_Car(Car *pid)
{
	sprintf((char*)str,
		"target_pos : %2f actual_pos : %2f\r\n err : \r\n", 
		 pid->target_pos,	   pid->actual_pos);
	HAL_UART_Transmit(&huart1, str, sizeof(str), 100);
}


int XianFu_PWM(int pwm)
{
	if (pwm >= 10000)
	{
		pwm = 10000;
	}
	else if (pwm <= -10000)
	{
		pwm = -10000;
	}
	return pwm;
}

void Motor_Left(int pwm)
{
	if (pwm > 0) // forward
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	}
	else if(pwm < 0) //back
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -pwm);
	}
	else // stop
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	}
}

void Motor_Right(int pwm)
{
	pwm = -pwm;	
	if (pwm > 0) // forward
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	}
	else if(pwm < 0) //back
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, -pwm);
	}
	else // stop
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	}
}

void Motor_Control(int pwm1, int pwm2)
{
	// if (pwm1 == pwm2 ) go straight
	// if (pwm1 > pwm2) turn right
	// if (pwm1 < pwm2) turn left
	pwm1 = XianFu_PWM(pwm1);
	pwm2 = XianFu_PWM(pwm2);
	Motor_Left(pwm1);
	Motor_Right(pwm2);
	
}

//MotorPidSetSpeed(3, 4);//turn left
//MotorPidSetSpeed(4, 3);//turn right
//MotorPidSetSpeed(4, 4);//forward
//MotorPidSetSpeed(-3, -3);//backward
void MotorPidSetSpeed(float Motor1SetSpeed, float Motor2SetSpeed)
{
	//Set pid target speed
	pidMotor1Speed.target_val = Motor1SetSpeed;
	pidMotor2Speed.target_val = Motor2SetSpeed;
	
	Motor_Control(PID_realize(&pidMotor1Speed, Motor1Speed), PID_realize(&pidMotor2Speed, Motor2Speed));
}

void MotorPidSpeedUp(void)
{
	static float motor_Speed_Up = 2;
	if (motor_Speed_Up < MAX_SPEED)
	{
		motor_Speed_Up += 0.5;
		MotorPidSetSpeed(motor_Speed_Up, motor_Speed_Up);
	}
}

void MotorPidSpeedDown(void)
{
	static float motor_Speed_Down = 4;
	if (motor_Speed_Down > MIN_SPEED)
	{
		motor_Speed_Down -= 0.5;
		MotorPidSetSpeed(motor_Speed_Down, motor_Speed_Down);
	}
}

void stop(void)
{
	pidMotor1Speed.target_val = 0;
	pidMotor2Speed.target_val = 0;
	Motor_Control(0, 0);
}

void MotorTurnAngle(float angle, float speed)
{
	pidMPU6050YawMovement.target_val = angle;
	g_fMPU6050YawMovePidOut = PID_Anglerealize(&pidMPU6050YawMovement, Yaw);
	  
	g_fMPU6050YawMovePidOut1 = speed - g_fMPU6050YawMovePidOut;
	g_fMPU6050YawMovePidOut2 = speed + g_fMPU6050YawMovePidOut;
	MotorPidSetSpeed(g_fMPU6050YawMovePidOut1, g_fMPU6050YawMovePidOut2);
}

void Climb (float angle, float speed)
{
	pidMPU6050PitchMovement.target_val = -angle;
	g_fMPU6050PitchMovePidOut = PID_Anglerealize(&pidMPU6050PitchMovement, Pitch);
	  
	g_fMPU6050PitchMovePidOut1 = speed - g_fMPU6050PitchMovePidOut;
	g_fMPU6050PitchMovePidOut2 = speed + g_fMPU6050PitchMovePidOut;
	MotorPidSetSpeed(g_fMPU6050PitchMovePidOut1, g_fMPU6050PitchMovePidOut2);
}
//void MoveTo(int des, int act)
//{
//	if (des * 1560 )
//}

void OLED_Show()
{
	memset(OLED_GRAM, 0, 128 * 8*sizeof(uint8_t));
	sprintf((char*)OledString, "v1: %2.2fv2: %2.2f", Motor1Speed, Motor2Speed);
	OLED_ShowString(0, 0, OledString);
	sprintf((char*)OledString, "Mileage: %2.2f", Mileage);
	OLED_ShowString(0, 10, OledString);
	sprintf((char*)OledString, "U: %2.2fV", adcGetBatteryVoltage());
	OLED_ShowString(0, 20, OledString);
	sprintf((char*)OledString, "R: %2.1f", Roll);
	OLED_ShowString(0, 30, OledString);
	sprintf((char*)OledString, "P: %2.1f", Pitch);
	OLED_ShowString(0, 40, OledString);
	sprintf((char*)OledString, "Y: %2.1f", Yaw);
	OLED_ShowString(0, 50, OledString);
	sprintf((char*)OledString, "D: %2.2f", g_fMPU6050YawMovePidOut);
	//sprintf((char*)OledString, "D: %2.2f", g_fMPU6050PitchMovePidOut);
	OLED_ShowString(64, 50, OledString);
	sprintf((char*)OledString, "M: %2.2f", MAX_Roll);
	OLED_ShowString(64, 40, OledString);
	sprintf((char*)OledString, "T: %2.2f", Target_Yaw);
	OLED_ShowString(64, 30, OledString);
	OLED_Refresh_Gram();
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3) // 1000hz 10ms
	{
		static int Moto1 = 0, Moto2 = 0;
		Timer1Count++;
		if (Timer1Count % 1 == 0)
		{
			Encoder1Count = (short)__HAL_TIM_GET_COUNTER(&htim4);
			Encoder2Count = -(short)__HAL_TIM_GET_COUNTER(&htim8);
				
			Motor1Speed = (float)Encoder1Count * 100 / 30 / 13 / 4; 
			Motor2Speed = (float)Encoder2Count * 100 / 1560;
			__HAL_TIM_SET_COUNTER(&htim4, 0);
			__HAL_TIM_SET_COUNTER(&htim8, 0);
			//Show_Info();		
			
		}
		if (Timer1Count % 2 == 0) //20ms
		{
			Mileage += (Motor1Speed + Motor2Speed) / 2 * 0.02 * 0.044; //m
			Moto1 = PID_realize(&pidMotor1Speed, Motor1Speed);
			Moto2 = PID_realize(&pidMotor2Speed, Motor2Speed);
			Motor_Control(Moto1, Moto2);
			Timer1Count = 0;
		}	
	}
}

	

uint8_t Usart_WaitReasFinish(void)
{
	static uint8_t Usart_LastReadCount = 0;
	
	if (usart1_ReadBufCount == 0)
	{
		Usart_LastReadCount = 0;
		return 1;
	}
	else if (Usart_LastReadCount == usart1_ReadBufCount)
	{
		Usart_LastReadCount = 0;
		usart1_ReadBufCount = 0;
		return 0;
	}
	else
	{
		Usart_LastReadCount = usart1_ReadBufCount;
		return 2;
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

	LED1_ON();
	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_2);

	HAL_TIM_Base_Start_IT(&htim3);
	
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	
	PID_init();
	
	cJSON *cJsonData, *cJsonValue;
	
	OLED_Init();
	
	//OLED_ShowString(30, 30, oled_str);
	HAL_Delay(500);
	MPU6050_initialize();
	DMP_Init();

	LED1_OFF();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  OLED_Show();
	  Read_DMP();

	  if (-Roll > MAX_Roll)
	  {
		  MAX_Roll = -Roll;
		  Target_Yaw = Yaw;
	  }

	  MotorTurnAngle(Target_Yaw, 2);
	  
	  //Climb(Pitch, 2);
	  //MotorTurnAngle(-0.001, 2);

	  delay_ms(50);

	  //ANO_DT_Send_F2(wheel1.actual_pos / 1560 * 100, 3.0 * 100, wheel2.actual_pos / 1560 * 100, 3.0 * 100);
	  //ANO_DT_Send_F2(Motor1Speed * 100, 3.0 * 100, Motor2Speed * 100, 3.0 * 100);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
