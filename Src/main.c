/**
  ******************************************************************************
  * @file    TIM/TIM_PWMOutput/Src/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-June-2014
  * @brief   This sample code shows how to use STM32F3xx TIM HAL API to generate
  *          4 signals in PWM.
  ******************************************************************************
  * @attention
  * 
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  * 
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/********************************************************************************
	Pin connection and configuration for STM32F303CCT6:s
	IN2_1 		:: 10 :: PA0
	IN2_2 		:: 11 :: PA1
	IN2_3 		:: 12 :: PA2
	EN2_2 		:: 13 :: PA3
	EN2_3 		:: 14 :: PA4
	EN2_1 		:: 15 :: PA5
	IN1_3 		:: 16 :: PA6
	IN1_1 		:: 17 :: PA7
	ENC1_A 		:: 29 :: PA8
	ENC1_B 		:: 30 :: PA9
	ENC1_Z 		:: 31 :: PA10
	HALL_B2		:: 32 :: PA11
	HALL_Z2		:: 33 :: PA12
	SWDIO	 		:: 34 :: PA13/SWDIO
	SWCLK 		:: 37 :: PA14/SWCLK
	TIM8	 		:: 38 :: PA15
	OSC_IN 		:: 05 :: PD0
	OSC_OUT		:: 06 :: PD1
	
	IN1_2 		:: 18 :: PB0
						:: 19 :: PB1
	DLAG/EN1	:: 20 :: PB2
	HALL_A1		:: 39 :: PB3
	HALL_B1 	:: 40 :: PB4
	HALL_Z1 	:: 41 :: PB5
	ENC2_A 		:: 42 :: PB6
	ENC2_B 		:: 43 :: PB7
	ENC2_Z 		:: 45 :: PB8
	HALL_A2		:: 46 :: PB9
	USART3_TX :: 21 :: PB10	
	USART3_RX	:: 22 :: PB11
	EN1_1 		:: 25 :: PB12
	EN1_2 		:: 26 :: PB13
	TIM15 		:: 27 :: PB14
	EN1_3			:: 28 :: PB15
	DLAG/EN2 	:: 02 :: PC13
	OSC32_IN 	:: 03 :: PC14
	OSC32_OUT :: 04 :: PC15
	
  ******************************************************************************
	****** Regroup Pin connection ******
	*** GPIO Input ***
	HALL_A1		:: 39 :: PB3
	HALL_B1 	:: 40 :: PB4
	HALL_Z1 	:: 41 :: PB5

	HALL_A2		:: 46 :: PB8-->PA10
	HALL_B2		:: 32 :: PA11
	HALL_Z2		:: 33 :: PA12
	
	*** GPIO Output ***
	DLAG/EN1	:: 20 :: PB2
	DLAG/EN2 	:: 02 :: PC13

	EN1_1 		:: 25 :: PB12
	EN1_2 		:: 26 :: PB13
	EN1_3			:: 28 :: PB15
	
	EN2_1 		:: 15 :: PA5
	EN2_2 		:: 13 :: PA3
	EN2_3 		:: 14 :: PA4
	
	*** PWM ***********
	IN1_3 		:: 16 :: PA6 :: TIM3_CH1, !!!!!, AF2
	IN1_1 		:: 17 :: PA7 :: TIM3_CH2, !!!!!, AF2
	IN1_2 		:: 18 :: PB0 :: TIM3_CH3, !!!!!, AF2
	
	IN2_1 		:: 10 :: PA0 :: TIM2_CH1, AF1
	IN2_2 		:: 11 :: PA1 :: TIM2_CH2, AF1
	IN2_3 		:: 12 :: PA2 :: TIM2_CH3, AF1
	
	*** Encoder *******
	ENC1_A 		:: 29 :: PA8 :: TIM1_CH1, AF6
	ENC1_B 		:: 30 :: PA9 :: TIM1_CH1, AF6

	ENC2_A 		:: 42 :: PB6 :: TIM4_CH1, AF2
	ENC2_B 		:: 43 :: PB7 :: TIM4_CH1, AF2

	*** other *********
	OSC32_IN 	:: 03 :: PC14, -- need jumper
	OSC32_OUT :: 04 :: PC15, -- need jumper
	
	*** test pin ****** 
	Loop0			:: 38 :: PA15, -- need jumper
	Loop1			:: 19 :: PB1, -- need jumper						
	Loop2			:: 27 :: PB14, -- need jumper						

	*** Serial Wire Debug ****
	SWDIO	 		:: 34 :: PA13/SWDIO,
	SWCLK 		:: 37 :: PA14/SWCLK,

	*** UART *********
	USART3_TX :: 21 :: PB10	
	USART3_RX	:: 22 :: PB11

	*** I2C **********
	I2C1_SCL	:: 45 :: PB8
	I2C1_SDA	:: 46 :: PB9
	
	*************** Change Log 4/21/15 *******************
	ENC2_Z -->I2C1_SCL	:: 45 :: PB8, 
	HALL_A2-->I2C1_SDA	:: 46 :: PB9, 
	ENC1_Z -->HALL_A2   :: 31 :: PA10,
	
	*** --> the new pin out:: 
	HALL_A2		:: 31 :: PA10
	HALL_B2		:: 32 :: PA11
	HALL_Z2		:: 33 :: PA12
	
	*** I2C *********
	I2C1_SCL	:: 45 :: PB8
	I2C1_SDA	:: 46 :: PB9
	
	******************************************************************************

TIM6_DAC_IRQn //16-bit time base, cout up, Prescaler 1~65536
TIM7_IRQn //16-bit time base, cout up, Prescaler 1~65536
TIM1_BRK_TIM15_IRQn //16-bit time base, cout up, Prescaler 1~65536
TIM1_UP_TIM16_IRQn //16-bit time base, cout up, Prescaler 1~65536
TIM1_TRG_COM_TIM17_IRQn //16-bit time base, cout up, Prescaler 1~65536
//TIM8, 16-bit time base, cout up/down, Prescaler 1~65536

	****** Loop Definition ******
	*** Commutation Loop ***

	*** Motor Control Loop ***

	*** Sensor Fusion Loop ***

	*** Body Control Loop ***
	
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F3xx_HAL_Examples
  * @{
  */

/** @addtogroup TIM_PWMOutput
  * @{
  */
#define LOOP0_STEP_SIZE	5	  /* 100 KHz/20KHz =5 */ 
#define LOOP1_STEP_SIZE	100	  /* 100 KHz/1KHz =100 */
#define LOOP2_STEP_SIZE	1000  /* 100 KHz/100Hz =1000 */
#define SERIAL_LOOP1 1
#define SERIAL_LOOP2 1
#define QEI_DEF	1

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile float pi;
volatile float Kc2r;
volatile float freq;	//Hz
volatile float flap;
volatile float flap_d;
volatile float flap_dd;

volatile double Ts;
volatile float time1;
volatile float time_save;
volatile float two_pi_f;

volatile int16_t Pos_count1;
volatile int16_t Pos_count2;
volatile int32_t Z_count2;
volatile uint8_t CH_A2;
volatile uint8_t CH_B2;
volatile uint8_t Direct2;

volatile float y; 
volatile float y_dot;
volatile float y_ddot; 
volatile float y_save;

volatile float yd; 
volatile float yd_dot;
volatile float yd_ddot; 

volatile float e;
volatile float e_dot;

volatile uint32_t i;

/******UART Double********/
union
{
   double f;
   uint8_t c[8];
} uart_double;

/******UART Float********/
union
{
   float f;
   uint8_t c[4];
} uart_float;

/*******UART Uint32*******/
union
{
   int32_t f;
   uint8_t c[4];
} uart_int32;

/*******UART int16*******/
union
{
   int16_t f;
   uint8_t c[2];
} uart_int16;


/******* timer file *******/
volatile uint8_t LOOP1_flag;
volatile uint8_t LOOP2_flag;

volatile int32_t Uin1;
volatile int32_t Uin2;

volatile uint32_t PWM_cycle;
volatile uint32_t Vin1;
volatile uint32_t Vin2;
volatile uint8_t Hall_1;
volatile uint8_t Hall_2;
volatile uint8_t Hall_State1;
volatile uint8_t Hall_State2;

volatile uint8_t Ddirect1;
volatile uint8_t Ddirect2;

/* Timer handler declaration */
TIM_HandleTypeDef    Tim3Handle; // PWM1
TIM_HandleTypeDef    Tim2Handle; // PWM2

TIM_HandleTypeDef    Tim1Handle; // Encoder1
TIM_HandleTypeDef    Tim4Handle; // Encoder2

TIM_HandleTypeDef    TimHandle1;
TIM_HandleTypeDef    TimHandle2;
TIM_HandleTypeDef    TimHandle3;

/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig;
TIM_Encoder_InitTypeDef sConfigEncoder;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/
void PWM1_Init_Start(uint32_t PWM_cycle);
void PWM2_Init_Start(uint32_t PWM_cycle);
void PWM1_Set_SingleEdge(uint32_t duty_ch1, uint32_t duty_ch2, uint32_t duty_ch3);
void PWM2_Set_SingleEdge(uint32_t duty_ch1, uint32_t duty_ch2, uint32_t duty_ch3);
void QEI1_Init_Start(void);
void QEI2_Init_Start(void);
int16_t QEI1_GetPosition(void);
int16_t QEI2_GetPosition(void);
void QEI1_Reset(void);
void QEI2_Reset(void);

void Looptimer1_Int_Start(uint32_t LOOP_STEP_SIZE);
void Looptimer2_Int_Start(uint32_t LOOP_STEP_SIZE);
void Looptimer3_Int_Start(uint32_t LOOP_STEP_SIZE);

void GPIO_Init(void);

uint8_t loop1(void);
uint8_t loop2(void);

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	/*************Variable Initializations*****************/
	/*3600/2=1800-->40K, 3600/3.6=100-->72K, 3600/4=900-->80K, 3600/5=720-->100K*/
	PWM_cycle =900; 
	/********************/
	
	Vin1=0;
	Vin2=0;
	
	Ddirect1=1;
	Ddirect2=1;
	
	Hall_1=0x05;
	Hall_2=0x05;
	Hall_State1=1;
	Hall_State2=1;
	
	Z_count2=0;
	CH_A2=0;
	CH_B2=0;
	Direct2=1;
	Pos_count1=0;
	Pos_count2=0;
	
	Ts=(double)LOOP1_STEP_SIZE/1000000.0;
	
  /* STM32F3xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure LED3 */
//BSP_LED_Init(LED3);

  /* Configure the system clock to have a system clock = 72 Mhz */
  SystemClock_Config();

/*************QEI Initializations****************/		
#if QEI_DEF
	QEI1_Init_Start();
	QEI2_Init_Start();
#endif


	GPIO_Init();
	/**** GPIO Input ***
	HALL_A1		:: 39 :: PB3
	HALL_B1 	:: 40 :: PB4
	HALL_Z1 	:: 41 :: PB5

	HALL_A2		:: 31 :: PA10
	HALL_B2		:: 32 :: PA11
	HALL_Z2		:: 33 :: PA12
	
	*** GPIO Output ***
	DLAG/EN1	:: 20 :: PB2
	DLAG/EN2 	:: 02 :: PC13

	//Driver Enable Output 1
	 
	EN1_1 		:: 25 :: PB12
	EN1_2 		:: 26 :: PB13
	EN1_3			:: 28 :: PB15
	
	//Driver Enable Output 2
	
	EN2_1 		:: 15 :: PA5
	EN2_2 		:: 13 :: PA3
	EN2_3 		:: 14 :: PA4
	*/
	// To get the level of a pin configured in input mode use HAL_GPIO_ReadPin().
	// To set/reset the level of a pin configured in output mode use HAL_GPIO_WritePin()/HAL_GPIO_TogglePin().
	// clear pin &= ~(1UL << pos);
	// set pin |= (1UL << pos);
	// HIGH, GPIO_PIN_SET
	
		
/*******************Serial for duty cycle*****************************/
  // Vin1
	// Vin2
//Direction1
//Direction2

/********************Loop test_pin Initializations********************/	
	/*** test pin ****** 
	Loop0			:: 38 :: PA15
	Loop1			:: 19 :: PB1						
	Loop2			:: 27 :: PB14 */
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);// test1_pin=LOW, 
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);// test2_pin=LOW, 
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);// test3_pin=LOW, 
						
/****** Disable Drive *********/						
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

/*************UART Initializations****************/

#if SERIAL_LOOP1
//UARTInit(0, 115200);
#endif

#if SERIAL_LOOP2
//UARTInit(0, 115200);
#endif

 /*************IRQn NVIC Set Priority****************/

//	NVIC_SetPriority(QEI_IRQn,2);
//	NVIC_SetPriority(PWM1_IRQn,3);

//	NVIC_SetPriority(TIMER0_IRQn,0);
//	NVIC_SetPriority(EINT3_IRQn,2);

//	NVIC_SetPriority(TIMER1_IRQn,3);
//	NVIC_SetPriority(TIMER2_IRQn,4);

//  NVIC_SetPriority(UART0_IRQn,5);
 /*************Loop Initializations*****************/
	LOOP1_flag=0;
	LOOP2_flag=0;
/***************************************************/

/*************PWM Initializations*****************/
	PWM1_Init_Start( PWM_cycle);
	PWM2_Init_Start( PWM_cycle);
	
	 PWM1_Set_SingleEdge( 0, 0, 0);
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);// !RESET_A1=LOW, EN1_1 :: 25 :: PB12
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);// !RESET_B1=LOW, EN1_2 :: 26 :: PB13
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);// !RESET_C1=LOW, EN1_3 :: 28 :: PB15

	 PWM2_Set_SingleEdge( 0, 0, 0);
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);// !RESET_C2=LOW, EN2_1 :: 15 :: PA5
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);// !RESET_B2=LOW, EN2_2 :: 13 :: PA3
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);// !RESET_A2=LOW, EN2_3 :: 14 :: PA4

/*************QEI Initializations****************/		
#if QEI_DEF
	QEI1_Reset();
	QEI2_Reset();
#endif

/*************Timer Initializations****************/
	Looptimer1_Int_Start(LOOP0_STEP_SIZE);
#if SERIAL_LOOP1
	Looptimer2_Int_Start(LOOP1_STEP_SIZE);
#endif
#if SERIAL_LOOP2	
	Looptimer3_Int_Start(LOOP2_STEP_SIZE);
#endif	

/********** Enable drive**************/
	/**** GPIO Output ***
	DLAG/EN1	:: 20 :: PB2
	DLAG/EN2 	:: 02 :: PC13*/
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  while (1)
  {
		
#if SERIAL_LOOP1
   if(LOOP1_flag) {loop1();}
#endif
	 
#if SERIAL_LOOP2	 
   if(LOOP2_flag) {loop2();}
#endif
		
  }

}

/*********************Trajectory Tracking Loop*********************************/ 
uint8_t loop1(void)
{
	LOOP1_flag=0;
/*************************************************/
	time1=time1+Ts; 
#if QEI_DEF

//	y1=((double)Pos_count1)*Kc2r;
//	y2=((double)Pos_count2)*Kc2r;
	// QEI_GetVelocity;
//	y1_dot=(y-y_save)/Ts;
	//y1_save=y;											//takes 10us 
#endif //QEI_DEF

// yd=flap*sin(two_pi_f*time1);

//if(time1<2.0)
//{	yd=yd*time1*0.5;}																 
	
//	yd=1;
//	Uin=(uint32_t)((yd)*900.0+1800.0);
/*1v->150*/	/*3V->450*/ /*6V->900*/ /*9V->1350*/ /*12V->1800*/
/*
	if(Uin >3600)   
	{Vin=3600;}
	else if(Uin < 0)
	{Vin= 0;} 
	else {Vin=Uin;}													//takes 20us
*/
//#if SERIAL_LOOP1
//	/*******UART int16*******/
//	uart_int16.f = 1000;	   						//takes 150us(115200) 
//	UARTSend( 0, uart_int16.c, 2 ); 	
//#endif		

/***************************************/
/**** test pin ****** 
	Loop1			:: 19 :: PB1*/
 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1); 
/***************************************/	 
  return (1);
}
/*****************************************************************************/
uint8_t loop2(void)
{
  LOOP2_flag=0;
/**********************************************/

	/******UART Float********/
//	uart_float.f = 123123.023123;	                //takes 350us(115200)
//	UARTSend( 0, uart_float.c, 4 ); 
	/************************/
					 
	/******UART Double********/
//	uart_double.f = yd;	                //takes 700us(115200) 
//	UARTSend( 0, uart_double.c, 8 ); 
//	uart_double.f = y;
//	UARTSend( 0, uart_double.c, 8 );	
//	uart_double.f = (double)Vin;
//	UARTSend( 0, uart_double.c, 8 );
	/************************/
	
	/*******UART int16*******/
//	uart_int16.f = 1000;	   						//takes 200us(115200) 
//	UARTSend( 0, uart_int16.c, 2 ); 	
	
	/*******UART Uint32*******/
//	uart_int32.f = Pos_count1;	   						//takes 350us(115200) 
//	UARTSend( 0, uart_int32.c, 4 ); 	

/*
	uart_int32.f = Vin;	   						        //takes 350us(115200) 
	UARTSend( 0, uart_int32.c, 4 );
	
	uart_int32.f = (int32_t)(yd/Kc2r);	   	 //takes 350us(115200) 
	UARTSend( 0, uart_int32.c, 4 );
*/
	/********Body control loop********/	 
	Pos_count1=QEI1_GetPosition();  //takes 10us 
	Pos_count2=QEI2_GetPosition();  //takes 10us 
	
	
  /*************************/	 
	Uin1=-400;
	Uin2=-400;
	if(Uin1>=0)
	{
		Ddirect1=1;
		Vin1=Uin1;
	} else {
		Ddirect1=0;
		Vin1=(uint32_t)-Uin1;
	}
	
	if(Uin2>=0)
	{
		Ddirect2=1;
		Vin2=Uin2;
	} else {
		Ddirect2=0;
		Vin2=(uint32_t)-Uin2;
	}
/*
	Hall_1++;
	Hall_2++;
	if(Hall_1>6)
{Hall_1=0;}
	if(Hall_2>6)
{Hall_2=0;}
*/
/***************************************/
/**** test pin ****** 				
	Loop2			:: 27 :: PB14
	*/	
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
/***************************************/
	return (1);
}




/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED3 on */
//  BSP_LED_On(LED3);
  while(1)
  {
		
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV                     = 1
  *            PLLMUL                         = RCC_PLL_MUL9 (9)
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    Error_Handler();
  }
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

void PWM1_Init_Start(uint32_t PWM_cycle)
{
	/**** PWM ***
	IN1_3 		:: 16 :: PA6 :: TIM3_CH1, !!!!!, AF2
	IN1_1 		:: 17 :: PA7 :: TIM3_CH2, !!!!!, AF2
	IN1_2 		:: 18 :: PB0 :: TIM3_CH3, !!!!!, AF2
	
	IN2_1 		:: 10 :: PA0 :: TIM2_CH1, AF1
	IN2_2 		:: 11 :: PA1 :: TIM2_CH2, AF1
	IN2_3 		:: 12 :: PA2 :: TIM2_CH3, AF1
	*/
	
	/* Compute the prescaler value to have TIM3 counter clock equal to 72 MHz */
  /* Counter Prescaler value */
	uint32_t uhPrescalerValue = (uint32_t) (SystemCoreClock / 72000000) - 1;


  /*##-1- Configure the TIM peripheral #######################################*/ 
  /* -----------------------------------------------------------------------
  TIM3 Configuration: generate 3 PWM signals with 3 different duty cycles.
  
    TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM3CLK = 2 * PCLK1  
      PCLK1 = HCLK / 2 
      => TIM3CLK = HCLK = SystemCoreClock
          
    To get TIM3 counter clock at 72 MHz, the prescaler is computed as follows:
       Prescaler = (TIM3CLK / TIM3 counter clock) - 1=0
       Prescaler = (SystemCoreClock /72 MHz) - 1=0
                                              
    To get TIM3 output clock at 80 KHz, the period (ARR)) is computed as follows:
       ARR = (TIM3 counter clock / TIM3 output clock) - 1 = 72000K/80K - 1 =900-1
           = 899
                  
    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
    TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
  
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f3xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock 
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency     
  ----------------------------------------------------------------------- */ 
  
  /* Initialize TIMx peripheral as follows:
       + Prescaler = (SystemCoreClock/72000000) - 1
       + Period = 900-1
      + ClockDivision = 0
      + Counter direction = Up
  */
  Tim3Handle.Instance = TIM3;
  Tim3Handle.Init.Prescaler = uhPrescalerValue;
  Tim3Handle.Init.Period = (PWM_cycle - 1);
  Tim3Handle.Init.ClockDivision = 0;
  Tim3Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_PWM_Init(&Tim3Handle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure the PWM channels #########################################*/ 
  /* Common configuration for all channels */
  sConfig.OCMode = TIM_OCMODE_PWM1;
  sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode = TIM_OCFAST_DISABLE;

  /* Set the pulse value for channel 1 */
  sConfig.Pulse = 0;  
  if(HAL_TIM_PWM_ConfigChannel(&Tim3Handle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /* Set the pulse value for channel 2 */
  sConfig.Pulse = 0;
  if(HAL_TIM_PWM_ConfigChannel(&Tim3Handle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /* Set the pulse value for channel 3 */
  sConfig.Pulse = 0;
  if(HAL_TIM_PWM_ConfigChannel(&Tim3Handle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  
	  /*##-3- Start PWM signals generation #######################################*/ 
  /* Start channel 1 */
  if(HAL_TIM_PWM_Start(&Tim3Handle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  /* Start channel 2 */
  if(HAL_TIM_PWM_Start(&Tim3Handle, TIM_CHANNEL_2) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  /* Start channel 3 */
  if(HAL_TIM_PWM_Start(&Tim3Handle, TIM_CHANNEL_3) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
 
}

void PWM1_Set_SingleEdge(uint32_t duty_ch1, uint32_t duty_ch2, uint32_t duty_ch3)
{
		/**** PWM ***
	IN1_3 		:: 16 :: PA6 :: TIM3_CH1, !!!!!, AF2
	IN1_1 		:: 17 :: PA7 :: TIM3_CH2, !!!!!, AF2
	IN1_2 		:: 18 :: PB0 :: TIM3_CH3, !!!!!, AF2
	*/
 /* Set the pulse value for channel 1 */
  sConfig.Pulse = duty_ch3;  
  if(HAL_TIM_PWM_ConfigChannel(&Tim3Handle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /* Set the pulse value for channel 2 */
  sConfig.Pulse = duty_ch1;
  if(HAL_TIM_PWM_ConfigChannel(&Tim3Handle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /* Set the pulse value for channel 3 */
  sConfig.Pulse = duty_ch2;
  if(HAL_TIM_PWM_ConfigChannel(&Tim3Handle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
	
		  /*##-3- Start PWM signals generation #######################################*/ 
  /* Start channel 1 */
  if(HAL_TIM_PWM_Start(&Tim3Handle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  /* Start channel 2 */
  if(HAL_TIM_PWM_Start(&Tim3Handle, TIM_CHANNEL_2) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  /* Start channel 3 */
  if(HAL_TIM_PWM_Start(&Tim3Handle, TIM_CHANNEL_3) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
}

void PWM2_Init_Start(uint32_t PWM_cycle)
{
	/*
  	*** PWM ***
	IN1_3 		:: 16 :: PA6 :: TIM3_CH1, !!!!!, AF2
	IN1_1 		:: 17 :: PA7 :: TIM3_CH2, !!!!!, AF2
	IN1_2 		:: 18 :: PB0 :: TIM3_CH3, !!!!!, AF2
	
	IN2_1 		:: 10 :: PA0 :: TIM2_CH1, AF1
	IN2_2 		:: 11 :: PA1 :: TIM2_CH2, AF1
	IN2_3 		:: 12 :: PA2 :: TIM2_CH3, AF1
	*/
	
	/* Compute the prescaler value to have TIM2 counter clock equal to 72 MHz */
  /* Counter Prescaler value */
	uint32_t uhPrescalerValue = (uint32_t) (SystemCoreClock / 72000000) - 1;


  /*##-1- Configure the TIM peripheral #######################################*/ 
  /* -----------------------------------------------------------------------
  TIM2 Configuration: generate 3 PWM signals with 3 different duty cycles.
  
    TIM2 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM2CLK = 2 * PCLK1  
      PCLK1 = HCLK / 2 
      => TIM2CLK = HCLK = SystemCoreClock
          
    To get TIM2 counter clock at 72 MHz, the prescaler is computed as follows:
       Prescaler = (TIM2CLK / TIM2 counter clock) - 1=0
       Prescaler = (SystemCoreClock /72 MHz) - 1=0
                                              
    To get TIM2 output clock at 80 KHz, the period (ARR)) is computed as follows:
       ARR = (TIM2 counter clock / TIM2 output clock) - 1 = 72000K/80K - 1 =900-1
           = 899
                  
    TIM2 Channel1 duty cycle = (TIM2_CCR1/ TIM2_ARR)* 100 = 50%
    TIM2 Channel2 duty cycle = (TIM2_CCR2/ TIM2_ARR)* 100 = 37.5%
    TIM2 Channel3 duty cycle = (TIM2_CCR3/ TIM2_ARR)* 100 = 25%
  
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f3xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock 
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency     
  ----------------------------------------------------------------------- */ 
  
  /* Initialize TIMx peripheral as follows:
       + Prescaler = (SystemCoreClock/72000000) - 1
       + Period = 900-1
      + ClockDivision = 0
      + Counter direction = Up
  */
  Tim2Handle.Instance = TIM2;
  Tim2Handle.Init.Prescaler = uhPrescalerValue;
  Tim2Handle.Init.Period = (PWM_cycle - 1);
  Tim2Handle.Init.ClockDivision = 0;
  Tim2Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_PWM_Init(&Tim2Handle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure the PWM channels #########################################*/ 
  /* Common configuration for all channels */
  sConfig.OCMode = TIM_OCMODE_PWM1;
  sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode = TIM_OCFAST_DISABLE;

  /* Set the pulse value for channel 1 */
  sConfig.Pulse = 0;  
  if(HAL_TIM_PWM_ConfigChannel(&Tim2Handle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /* Set the pulse value for channel 2 */
  sConfig.Pulse = 0;
  if(HAL_TIM_PWM_ConfigChannel(&Tim2Handle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /* Set the pulse value for channel 3 */
  sConfig.Pulse = 0;
  if(HAL_TIM_PWM_ConfigChannel(&Tim2Handle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  
	  /*##-3- Start PWM signals generation #######################################*/ 
  /* Start channel 1 */
  if(HAL_TIM_PWM_Start(&Tim2Handle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  /* Start channel 2 */
  if(HAL_TIM_PWM_Start(&Tim2Handle, TIM_CHANNEL_2) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  /* Start channel 3 */
  if(HAL_TIM_PWM_Start(&Tim2Handle, TIM_CHANNEL_3) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
 
}

void PWM2_Set_SingleEdge(uint32_t duty_ch1, uint32_t duty_ch2, uint32_t duty_ch3)
{
		/**** PWM ***
	IN2_1 		:: 10 :: PA0 :: TIM2_CH1, AF1
	IN2_2 		:: 11 :: PA1 :: TIM2_CH2, AF1
	IN2_3 		:: 12 :: PA2 :: TIM2_CH3, AF1
	*/
 /* Set the pulse value for channel 1 */
  sConfig.Pulse = duty_ch1;  
  if(HAL_TIM_PWM_ConfigChannel(&Tim2Handle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /* Set the pulse value for channel 2 */
  sConfig.Pulse = duty_ch2;
  if(HAL_TIM_PWM_ConfigChannel(&Tim2Handle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /* Set the pulse value for channel 3 */
  sConfig.Pulse = duty_ch3;
  if(HAL_TIM_PWM_ConfigChannel(&Tim2Handle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
	  
	  /*##-3- Start PWM signals generation #######################################*/ 
  /* Start channel 1 */
  if(HAL_TIM_PWM_Start(&Tim2Handle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  /* Start channel 2 */
  if(HAL_TIM_PWM_Start(&Tim2Handle, TIM_CHANNEL_2) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  /* Start channel 3 */
  if(HAL_TIM_PWM_Start(&Tim2Handle, TIM_CHANNEL_3) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
}

void QEI1_Init_Start(void)
{
		/*
(#) Initialize the TIM low level resources by implementing the following functions 
        (++) Encoder mode output : HAL_TIM_Encoder_MspInit(), check!
(#) Initialize the TIM low level resources :
        (##) Enable the TIM interface clock using __TIMx_CLK_ENABLE(); , check!
        (##) TIM pins configuration
            (+++) Enable the clock for the TIM GPIOs using the following function:
             __GPIOx_CLK_ENABLE();   , check!
            (+++) Configure these TIM pins in Alternate function mode using HAL_GPIO_Init();  , check!
	
(#) The external Clock can be configured, if needed (the default clock is the 
         internal clock from the APBx), using the following function:
         HAL_TIM_ConfigClockSource, the clock configuration should be done before 
         any start function.  
(#) Configure the TIM in the desired functioning mode using one of the 
Initialization function of this driver:
        (++) HAL_TIM_Encoder_Init: to use the Timer Encoder Interface.
(#) Activate the TIM peripheral using one of the start functions depending from the feature used: 
        (++) Encoder mode output : HAL_TIM_Encoder_Start(), HAL_TIM_Encoder_Start_DMA(), HAL_TIM_Encoder_Start_IT().
(#) The DMA Burst is managed with the two following functions:
         HAL_TIM_DMABurst_WriteStart()
         HAL_TIM_DMABurst_ReadStart()

*/
  /**** Encoder ***
	ENC1_A 		:: 29 :: PA8 :: TIM1_CH1, AF6
	ENC1_B 		:: 30 :: PA9 :: TIM1_CH2, AF6
	
	ENC2_A 		:: 42 :: PB6 :: TIM4_CH1, AF2
	ENC2_B 		:: 43 :: PB7 :: TIM4_CH2, AF2
	*/
	
	/* Compute the prescaler value to have TIM1 counter clock equal to 72 MHz */
  /* Counter Prescaler value */
	//uint32_t uhPrescalerValue = (uint32_t) (SystemCoreClock / 72000000) - 1;


  /*##-1- Configure the TIM peripheral #######################################*/ 
  /* -----------------------------------------------------------------------
  TIM1 Configuration:
  
    TIM1 input clock (TIM1CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM1CLK = 2 * PCLK1  
      PCLK1 = HCLK / 2 
      => TIM1CLK = HCLK = SystemCoreClock
          
    To get TIM1 counter clock at 72 MHz, the prescaler is computed as follows:
       Prescaler = (TIM1CLK / TIM1 counter clock) - 1=0
       Prescaler = (SystemCoreClock /72 MHz) - 1=0
                                              
    To get TIM1 output clock at 80 KHz, the period (ARR)) is computed as follows:
       ARR = (TIM1 counter clock / TIM1 output clock) - 1 = 72000K/72000K - 1 =1-1
           = 0
  
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f3xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock 
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency     
  ----------------------------------------------------------------------- */ 
  
  /* Initialize TIMx peripheral as follows:
       + Prescaler = (SystemCoreClock/72000000) - 1
       + Period = 900-1
      + ClockDivision = 0
      + Counter direction = Up
  */
  Tim1Handle.Instance = TIM1;
  Tim1Handle.Init.Prescaler = 0;
  Tim1Handle.Init.Period = 0xFFFF;
  Tim1Handle.Init.ClockDivision = 0;
  Tim1Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	/*
	* TIM_EncoderMode: specifies the TIMx Encoder Mode.
  *          This parameter can be one of the following values:
  *            @arg TIM_EncoderMode_TI1: Counter counts on TI1FP1 edge depending on TI2FP2 level.
  *            @arg TIM_EncoderMode_TI2: Counter counts on TI2FP2 edge depending on TI1FP1 level.
  *            @arg TIM_EncoderMode_TI12: Counter counts on both TI1FP1 and TI2FP2 edges depending
	* TIM_IC1Polarity: specifies the IC1 Polarity
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPolarity_Falling: IC Falling edge.
  *            @arg TIM_ICPolarity_Rising: IC Rising edge.
  * TIM_IC2Polarity: specifies the IC2 Polarity
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPolarity_Falling: IC Falling edge.
  *            @arg TIM_ICPolarity_Rising: IC Rising edge.
	*/
	sConfigEncoder.EncoderMode=TIM_ENCODERMODE_TI12; //4X
	sConfigEncoder.IC1Polarity=TIM_ICPOLARITY_RISING;
	sConfigEncoder.IC2Polarity=TIM_ICPOLARITY_RISING;	
	
  if(HAL_TIM_Encoder_Init(&Tim1Handle, &sConfigEncoder) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

	  if(HAL_TIM_Encoder_Start(&Tim1Handle, TIM_CHANNEL_ALL) != HAL_OK) // TIM_CHANNEL_1 and TIM_CHANNEL_2
  {
    /* Initialization Error */
    Error_Handler();
  }
}

void QEI2_Init_Start(void)
{
		/*
(#) Initialize the TIM low level resources by implementing the following functions 
        (++) Encoder mode output : HAL_TIM_Encoder_MspInit(), check!
(#) Initialize the TIM low level resources :
        (##) Enable the TIM interface clock using __TIMx_CLK_ENABLE(); , check!
        (##) TIM pins configuration
            (+++) Enable the clock for the TIM GPIOs using the following function:
             __GPIOx_CLK_ENABLE();   , check!
            (+++) Configure these TIM pins in Alternate function mode using HAL_GPIO_Init();  , check!
	
(#) The external Clock can be configured, if needed (the default clock is the 
         internal clock from the APBx), using the following function:
         HAL_TIM_ConfigClockSource, the clock configuration should be done before 
         any start function.  
(#) Configure the TIM in the desired functioning mode using one of the 
Initialization function of this driver:
        (++) HAL_TIM_Encoder_Init: to use the Timer Encoder Interface.
(#) Activate the TIM peripheral using one of the start functions depending from the feature used: 
        (++) Encoder mode output : HAL_TIM_Encoder_Start(), HAL_TIM_Encoder_Start_DMA(), HAL_TIM_Encoder_Start_IT().
(#) The DMA Burst is managed with the two following functions:
         HAL_TIM_DMABurst_WriteStart()
         HAL_TIM_DMABurst_ReadStart()

*/
  /**** Encoder ***
	ENC1_A 		:: 29 :: PA8 :: TIM1_CH1, AF6
	ENC1_B 		:: 30 :: PA9 :: TIM1_CH2, AF6
	
	ENC2_A 		:: 42 :: PB6 :: TIM4_CH1, AF2
	ENC2_B 		:: 43 :: PB7 :: TIM4_CH2, AF2
	*/
	
	/* Compute the prescaler value to have TIM1 counter clock equal to 72 MHz */
  /* Counter Prescaler value */
	//uint32_t uhPrescalerValue = (uint32_t) (SystemCoreClock / 72000000) - 1;


  /*##-1- Configure the TIM peripheral #######################################*/ 
  /* -----------------------------------------------------------------------
  TIM4 Configuration:
  
    TIM4 input clock (TIM1CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM1CLK = 2 * PCLK1  
      PCLK1 = HCLK / 2 
      => TIM4CLK = HCLK = SystemCoreClock
          
    To get TIM4 counter clock at 72 MHz, the prescaler is computed as follows:
       Prescaler = (TIM1CLK / TIM1 counter clock) - 1=0
       Prescaler = (SystemCoreClock /72 MHz) - 1=0
                                              
    To get TIM4 output clock at 80 KHz, the period (ARR)) is computed as follows:
       ARR = (TIM4 counter clock / TIM4 output clock) - 1 = 72000K/72000K - 1 =1-1
           = 0
  
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f3xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock 
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency     
  ----------------------------------------------------------------------- */ 
  
  /* Initialize TIMx peripheral as follows:
       + Prescaler = (SystemCoreClock/72000000) - 1
       + Period =  0xFFFF;
      + ClockDivision = 0
      + Counter direction = Up
  */
  Tim4Handle.Instance = TIM4;
  Tim4Handle.Init.Prescaler = 0;
  Tim4Handle.Init.Period = 0xFFFF;
  Tim4Handle.Init.ClockDivision = 0;
  Tim4Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	
	sConfigEncoder.EncoderMode=TIM_ENCODERMODE_TI12; //4X
	sConfigEncoder.IC1Polarity=TIM_ICPOLARITY_RISING;
	sConfigEncoder.IC2Polarity=TIM_ICPOLARITY_RISING;	
	
  if(HAL_TIM_Encoder_Init(&Tim4Handle, &sConfigEncoder) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

	 	  if(HAL_TIM_Encoder_Start(&Tim4Handle, TIM_CHANNEL_ALL) != HAL_OK) // TIM_CHANNEL_1 and TIM_CHANNEL_2
  {
    /* Initialization Error */
    Error_Handler();
  }
}

int16_t QEI1_GetPosition(void)
{	
	/**** Encoder *******
	ENC1_A 		:: 29 :: PA8 :: TIM1_CH1, AF6
	ENC1_B 		:: 30 :: PA9 :: TIM1_CH1, AF6

	ENC2_A 		:: 42 :: PB6 :: TIM4_CH1, AF2
	ENC2_B 		:: 43 :: PB7 :: TIM4_CH1, AF2*/
	 /* Get the Counter Register value */
  return (int16_t) Tim1Handle.Instance->CNT;
}
int16_t QEI2_GetPosition(void)
{
		/**** Encoder *******
	ENC1_A 		:: 29 :: PA8 :: TIM1_CH1, AF6
	ENC1_B 		:: 30 :: PA9 :: TIM1_CH1, AF6

	ENC2_A 		:: 42 :: PB6 :: TIM4_CH1, AF2
	ENC2_B 		:: 43 :: PB7 :: TIM4_CH1, AF2*/
	 /* Get the Counter Register value */
  return (int16_t) Tim4Handle.Instance->CNT;
}
void QEI1_Reset(void)
{
		/**** Encoder *******
	ENC1_A 		:: 29 :: PA8 :: TIM1_CH1, AF6
	ENC1_B 		:: 30 :: PA9 :: TIM1_CH1, AF6

	ENC2_A 		:: 42 :: PB6 :: TIM4_CH1, AF2
	ENC2_B 		:: 43 :: PB7 :: TIM4_CH1, AF2*/
	Tim1Handle.Instance->CNT=0;
}
void QEI2_Reset(void)
{
		/**** Encoder *******
	ENC1_A 		:: 29 :: PA8 :: TIM1_CH1, AF6
	ENC1_B 		:: 30 :: PA9 :: TIM1_CH1, AF6

	ENC2_A 		:: 42 :: PB6 :: TIM4_CH1, AF2
	ENC2_B 		:: 43 :: PB7 :: TIM4_CH1, AF2*/
	Tim4Handle.Instance->CNT=0;
	
}


void GPIO_Init(void)
{
	 /*************GIO Initializations*****************/

	/*	*** GPIO Input ***
	HALL_A1		:: 39 :: PB3
	HALL_B1 	:: 40 :: PB4
	HALL_Z1 	:: 41 :: PB5

	HALL_A2		:: 31 :: PA10
	HALL_B2		:: 32 :: PA11
	HALL_Z2		:: 33 :: PA12
	
	*** GPIO Output ***
	DLAG/EN1	:: 20 :: PB2
	DLAG/EN2 	:: 02 :: PC13

	//Driver Enable Output 1
	 
	EN1_1 		:: 25 :: PB12
	EN1_2 		:: 26 :: PB13
	EN1_3			:: 28 :: PB15
	
	//Driver Enable Output 2
	
	EN2_1 		:: 15 :: PA5
	EN2_2 		:: 13 :: PA3
	EN2_3 		:: 14 :: PA4
	
		*** test pin ****** 
	Loop0			:: 38 :: PA15
	Loop1			:: 19 :: PB1						
	Loop2			:: 27 :: PB14					
	
	*/
	GPIO_InitTypeDef  GPIO_InitStruct;
  /* -1- Enable GPIOA GPIOB GPIOC Clock (to be able to program the configuration registers) */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();

	/* --- Configure IOs in input push-up mode */
		/*	*** GPIO Input ***
	HALL_A1		:: 39 :: PB3
	HALL_B1 	:: 40 :: PB4
	HALL_Z1 	:: 41 :: PB5

	HALL_A2		:: 31 :: PA10
	HALL_B2		:: 32 :: PA11
	HALL_Z2		:: 33 :: PA12*/
	//HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);
	//HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11);
  GPIO_InitStruct.Pin = ( GPIO_PIN_12 |GPIO_PIN_11 | GPIO_PIN_10 );
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
	
	GPIO_InitStruct.Pin = (GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 );
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH; 
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 
  
	/* -2- Configure IOs in output push-pull mode */
  GPIO_InitStruct.Pin = ( GPIO_PIN_15 |GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 );
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 

  GPIO_InitStruct.Pin = (GPIO_PIN_15 | GPIO_PIN_13 | GPIO_PIN_12 | GPIO_PIN_2 | GPIO_PIN_14 |GPIO_PIN_1 );
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH; 
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 
	
	GPIO_InitStruct.Pin = (GPIO_PIN_13);
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); 

	//To get the level of a pin configured in input mode use HAL_GPIO_ReadPin().
	//To set/reset the level of a pin configured in output mode use HAL_GPIO_WritePin()/HAL_GPIO_TogglePin().
	
	// clear pin &= ~(1UL << pos);
	// set pin |= (1UL << pos);
	
	//make sure the output pins are set to LOW.
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);// !RESET_A1=LOW, EN1_1 :: 25 :: PB12
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);// !RESET_B1=LOW, EN1_2 :: 26 :: PB13
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);// !RESET_C1=LOW, EN1_3 :: 28 :: PB15

	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);// !RESET_C2=LOW, EN2_1 :: 15 :: PA5
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);// !RESET_B2=LOW, EN2_2 :: 13 :: PA3
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);// !RESET_A2=LOW, EN2_3 :: 14 :: PA4
}

void Looptimer1_Int_Start(uint32_t LOOP_STEP_SIZE) // TIM7, 20 KHz, Commutation Loop
{
  /*##-1- Configure the TIM peripheral #######################################*/ 
  /* -----------------------------------------------------------------------
    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM3CLK = 2 * PCLK1  
      PCLK1 = HCLK / 2 
      => TIM3CLK = HCLK = SystemCoreClock
    To get TIM3 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    Prescaler = (SystemCoreClock /10 KHz) - 1
       
    Note:SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock 
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency  
  ----------------------------------------------------------------------- */  
  /* Compute the prescaler value to have TIMx counter clock equal to 100 KHz */
  uint32_t uwPrescalerValue = (uint32_t) (SystemCoreClock / 100000) - 1;
  
  /* Set TIMx instance */
  TimHandle1.Instance = TIM7;
  
  /* Initialize TIMx peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = (SystemCoreClock/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
     */
  TimHandle1.Init.Period = LOOP_STEP_SIZE - 1; //100 KHz/20KHz =5
  TimHandle1.Init.Prescaler = uwPrescalerValue;
  TimHandle1.Init.ClockDivision = 0;
  TimHandle1.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&TimHandle1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if(HAL_TIM_Base_Start_IT(&TimHandle1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

}

void Looptimer2_Int_Start(uint32_t LOOP_STEP_SIZE) // TIM15, 2 KHz, IMU Loop
{
  /*##-1- Configure the TIM peripheral #######################################*/ 
  /* -----------------------------------------------------------------------
    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM3CLK = 2 * PCLK1  
      PCLK1 = HCLK / 2 
      => TIM3CLK = HCLK = SystemCoreClock
    To get TIM3 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    Prescaler = (SystemCoreClock /10 KHz) - 1
       
    Note:SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock 
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency  
  ----------------------------------------------------------------------- */  
  /* Compute the prescaler value to have TIMx counter clock equal to 100 KHz */
  uint32_t uwPrescalerValue = (uint32_t) (SystemCoreClock / 100000) - 1;
  
  /* Set TIMx instance */
  TimHandle2.Instance = TIM15;
  
  /* Initialize TIMx peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = (SystemCoreClock/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
     */
  TimHandle2.Init.Period = LOOP_STEP_SIZE - 1; //100 KHz/2KHz =50
  TimHandle2.Init.Prescaler = uwPrescalerValue;
  TimHandle2.Init.ClockDivision = 0;
  TimHandle2.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&TimHandle2) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if(HAL_TIM_Base_Start_IT(&TimHandle2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
	
}

void Looptimer3_Int_Start(uint32_t LOOP_STEP_SIZE) // TIM16, 100Hz, Control Loop
{
  /*##-1- Configure the TIM peripheral #######################################*/ 
  /* -----------------------------------------------------------------------
    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM3CLK = 2 * PCLK1  
      PCLK1 = HCLK / 2 
      => TIM3CLK = HCLK = SystemCoreClock
    To get TIM3 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    Prescaler = (SystemCoreClock /10 KHz) - 1
       
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock 
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency  
  ----------------------------------------------------------------------- */  
  /* Compute the prescaler value to have TIMx counter clock equal to 100 KHz */
  uint32_t uwPrescalerValue = (uint32_t) (SystemCoreClock / 100000) - 1;
  
  /* Set TIMx instance */
  TimHandle3.Instance = TIM16;
  
  /* Initialize TIMx peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = (SystemCoreClock/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle3.Init.Period = LOOP_STEP_SIZE - 1; //100 KHz/100Hz =1000
  TimHandle3.Init.Prescaler = uwPrescalerValue;
  TimHandle3.Init.ClockDivision = 0;
  TimHandle3.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&TimHandle3) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if(HAL_TIM_Base_Start_IT(&TimHandle3) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
	
}
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
