/**
  ******************************************************************************
  * @file    TIM/TIM_PWMOutput/Src/stm32f3xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-June-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_it.h"
   

/** @addtogroup STM32F3xx_HAL_Examples
  * @{
  */

/** @addtogroup TIM_PWMOutput
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef    TimHandle1;
extern TIM_HandleTypeDef    TimHandle2;
extern TIM_HandleTypeDef    TimHandle3;

/***************motor control parameters*******************/	 
extern volatile uint8_t LOOP1_flag;
extern volatile uint8_t LOOP2_flag;
extern volatile uint32_t PWM_cycle;
extern volatile uint32_t Vin1;
extern volatile uint32_t Vin2;
extern volatile uint8_t Ddirect1;
extern volatile uint8_t Ddirect2;

extern volatile uint8_t Hall_1;
extern volatile uint8_t Hall_2;
extern volatile uint8_t Hall_State1;
extern volatile uint8_t Hall_State2;


/***************motor control PWM definition*******************/	
extern void PWM1_Set_SingleEdge(uint32_t duty_ch1, uint32_t duty_ch2, uint32_t duty_ch3);
extern void PWM2_Set_SingleEdge(uint32_t duty_ch1, uint32_t duty_ch2, uint32_t duty_ch3);

/* Pin definition
	*** PWM ***
	IN1_1 		:: 17 :: PA7	
	IN1_2 		:: 18 :: PB0
	IN1_3 		:: 16 :: PA6
	
	IN2_1 		:: 10 :: PA0
	IN2_2 		:: 11 :: PA1
	IN2_3 		:: 12 :: PA2
	
	*** Encoder ***
	ENC1_A 		:: 29 :: PA8
	ENC1_B 		:: 30 :: PA9
	ENC1_Z 		:: 31 :: PA10
	
	ENC2_A 		:: 42 :: PB6
	ENC2_B 		:: 43 :: PB7
	ENC2_Z 		:: 45 :: PB8
*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/*                 STM32F3xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f3xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */ 
/*void PPP_IRQHandler(void)
{
}*/
/**
  * @brief  This function handles TIM interrupt request.
  * @param  None
  * @retval None
  */
void TIM7_IRQHandler(void) //20KHz
{
  HAL_TIM_IRQHandler(&TimHandle1);
/*************************GPIO dspace**********************************/
/*************************************************************************/
//	Vin1=(uint32_t)((LPC_GPIO0->FIOPIN >> 17) & 0x0FFF);	 //bit28-bit17  
//  Vin2=(uint32_t)((LPC_GPIO0->FIOPIN >> 0) & 0x0FFF);	 //bit11-bit0  
	
	/*PWM freq: /1-->20K, /2-->40K, /4-->80K, /5-->100K*/
	/********************/
	
//	Ddirect1=(uint8_t)((LPC_GPIO1->FIOPIN >> 31) & 0x0001);	 //dspace bit13, LPC P1.31
//	Ddirect2=(uint8_t)((LPC_GPIO0->FIOPIN >> 16) & 0x0001);	 //dspace bit29, LPC P0.16
	
	/*************************************************************************/
	/**************************Commutation Loop*******************************/
	/**** GPIO Input *****
	HALL_A1		:: 39 :: PB3
	HALL_B1 	:: 40 :: PB4
	HALL_Z1 	:: 41 :: PB5

	HALL_A2		:: 31 :: PA10
	HALL_B2		:: 32 :: PA11
	HALL_Z2		:: 33 :: PA12*/

	uint8_t Hall_A1= (uint8_t) (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3) & 0x01);
	uint8_t Hall_B1= (uint8_t) (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4) & 0x01);
	uint8_t Hall_C1= (uint8_t) (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5) & 0x01);
  Hall_1=(uint8_t)(((Hall_C1 << 2)|(Hall_B1 << 1)|(Hall_A1)) & 0x07);	 //example, C1:bit2.12 B1:bit2.11 A1:bit2.10 

	uint8_t Hall_A2= (uint8_t) (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10) & 0x01);
	uint8_t Hall_B2= (uint8_t) (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11) & 0x01);
	uint8_t Hall_C2= (uint8_t) (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12) & 0x01);
	Hall_2=(uint8_t)(((Hall_C2 << 2)|(Hall_B2 << 1)|(Hall_A2)) & 0x07);	 //example, C2:bit1.29 B2:bit1.28 A2:bit1.27 

	if(Ddirect1==1)
{
	//postive direction
	switch ( Hall_1 )
	{
	  case 0x04:   //state 1, Hall_C,Hall_B,Hall_A:(100)
		Hall_State1 =1; //b-->C
		break; 
		
	  case 0x05:	 //state 2, Hall_C,Hall_B,Hall_A:(101)
		Hall_State1 =2; //b-->a
		break;
	  
		case 0x01:   //state 3, Hall_C,Hall_B,Hall_A:(001)
		Hall_State1 =3; //c-->a
		break;

	  case 0x03:   //state 4, Hall_C,Hall_B,Hall_A:(011)
		Hall_State1 =4; //c-->b
		break;
		
	  case 0x02:   //state 5, Hall_C,Hall_B,Hall_A:(010)
		Hall_State1 =5; //a-->b
		break;

	  case 0x06:   //state 6, Hall_C,Hall_B,Hall_A:(110)
		Hall_State1 =6; //a-->c
		break;

	  case 0x07:   //state 7, Hall_C,Hall_B,Hall_A:(111)
		Hall_State1 =7;
 		break;

	  case 0x00:   //state 0, Hall_C,Hall_B,Hall_A:(000)
		Hall_State1 =0;
 		break;
	}
} 
if(Ddirect1==0)
{
	//negative direction
	switch ( Hall_1 )
	{
	  case 0x03:   //state 1, Hall_C,Hall_B,Hall_A:(011)
		Hall_State1 =1;
		break; 
	
	  case 0x02:	 //state 2, Hall_C,Hall_B,Hall_A:(010)
		Hall_State1 =2;
		break;
	  
		case 0x06:   //state 3, Hall_C,Hall_B,Hall_A:(110)
		Hall_State1 =3;
		break;

	  case 0x04:   //state 4, Hall_C,Hall_B,Hall_A:(100)
		Hall_State1 =4;
		break;
		
	  case 0x05:   //state 5, Hall_C,Hall_B,Hall_A:(101)
		Hall_State1 =5;
		break;

	  case 0x01:   //state 6, Hall_C,Hall_B,Hall_A:(001)
		Hall_State1 =6;
		break;

	  case 0x07:   //state 7, Hall_C,Hall_B,Hall_A:(111)
		Hall_State1 =7;
 		break;

	  case 0x00:   //state 0, Hall_C,Hall_B,Hall_A:(000)
		Hall_State1 =0;
 		break;
	}	
}
	if(Ddirect2==1)
{
		//negative direction
	switch ( Hall_2 )
	{
	  case 0x03:   //state 1, Hall_C,Hall_B,Hall_A:(011)
		Hall_State2 =1;
		break; 
	
	  case 0x02:	 //state 2, Hall_C,Hall_B,Hall_A:(010)
		Hall_State2 =2;
		break;
	  
		case 0x06:   //state 3, Hall_C,Hall_B,Hall_A:(110)
		Hall_State2 =3;
		break;

	  case 0x04:   //state 4, Hall_C,Hall_B,Hall_A:(100)
		Hall_State2 =4;
		break;
		
	  case 0x05:   //state 5, Hall_C,Hall_B,Hall_A:(101)
		Hall_State2 =5;
		break;

	  case 0x01:   //state 6, Hall_C,Hall_B,Hall_A:(001)
		Hall_State2 =6;
		break;

	  case 0x07:   //state 7, Hall_C,Hall_B,Hall_A:(111)
		Hall_State2 =7;
 		break;

	  case 0x00:   //state 0, Hall_C,Hall_B,Hall_A:(000)
		Hall_State2 =0;
 		break;
	}
} 
if(Ddirect2==0)
{
	//postive direction
	switch ( Hall_2 )
	{
	  case 0x04:   //state 1, Hall_C,Hall_B,Hall_A:(100)
		Hall_State2 =1; //b-->C
		break; 
		
	  case 0x05:	 //state 2, Hall_C,Hall_B,Hall_A:(101)
		Hall_State2 =2; //b-->a
		break;
	  
		case 0x01:   //state 3, Hall_C,Hall_B,Hall_A:(001)
		Hall_State2 =3; //c-->a
		break;

	  case 0x03:   //state 4, Hall_C,Hall_B,Hall_A:(011)
		Hall_State2 =4; //c-->b
		break;
		
	  case 0x02:   //state 5, Hall_C,Hall_B,Hall_A:(010)
		Hall_State2 =5; //a-->b
		break;

	  case 0x06:   //state 6, Hall_C,Hall_B,Hall_A:(110)
		Hall_State2 =6; //a-->c
		break;

	  case 0x07:   //state 7, Hall_C,Hall_B,Hall_A:(111)
		Hall_State2 =7;
 		break;

	  case 0x00:   //state 0, Hall_C,Hall_B,Hall_A:(000)
		Hall_State2 =0;
 		break;
	}
}
		switch ( Hall_State1 )
{
	  case 0x01: //b-->C
	  PWM1_Set_SingleEdge( 0, Vin1, 0);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);// !RESET_A1=LOW, EN1_1 :: 25 :: PB12
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);// !RESET_B1=HIGH, EN1_2 :: 26 :: PB13
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);// !RESET_C1=HIGH, EN1_3 :: 28 :: PB15
		break; 
		
	  case 0x02: //b-->a
		PWM1_Set_SingleEdge( 0, Vin1, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);// !RESET_A1=HIGH, EN1_1 :: 25 :: PB12
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);// !RESET_B1=HIGH, EN1_2 :: 26 :: PB13
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);// !RESET_C1=LOW, EN1_3 :: 28 :: PB15
		break;
	  
		case 0x03: //c-->a
		PWM1_Set_SingleEdge( 0, 0, Vin1);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);// !RESET_A1=HIGH, EN1_1 :: 25 :: PB12
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);// !RESET_B1=LOW, EN1_2 :: 26 :: PB13
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);// !RESET_C1=HIGH, EN1_3 :: 28 :: PB15
		
		break;

	  case 0x04: //c-->b
		PWM1_Set_SingleEdge( 0, 0, Vin1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);// !RESET_A1=LOW, EN1_1 :: 25 :: PB12
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);// !RESET_B1=HIGH, EN1_2 :: 26 :: PB13
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);// !RESET_C1=HIGH, EN1_3 :: 28 :: PB15

		
		break;
		
	  case 0x05: //a-->b		
		PWM1_Set_SingleEdge( Vin1, 0, 0);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);// !RESET_A1=HIGH, EN1_1 :: 25 :: PB12
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);// !RESET_B1=HIGH, EN1_2 :: 26 :: PB13
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);// !RESET_C1=LOW, EN1_3 :: 28 :: PB15
		
		break;

	  case 0x06: //a-->c
		PWM1_Set_SingleEdge( Vin1, 0, 0);		
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);// !RESET_A1=HIGH, EN1_1 :: 25 :: PB12
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);// !RESET_B1=LOW, EN1_2 :: 26 :: PB13
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);// !RESET_C1=HIGH, EN1_3 :: 28 :: PB15
		
		break;

	  case 0x07:
		PWM1_Set_SingleEdge( 0, 0, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);// !RESET_A1=LOW, EN1_1 :: 25 :: PB12
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);// !RESET_B1=LOW, EN1_2 :: 26 :: PB13
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);// !RESET_C1=LOW, EN1_3 :: 28 :: PB15
 		break;

	  case 0x00: 
		PWM1_Set_SingleEdge( 0, 0, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);// !RESET_A1=LOW, EN1_1 :: 25 :: PB12
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);// !RESET_B1=LOW, EN1_2 :: 26 :: PB13
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);// !RESET_C1=LOW, EN1_3 :: 28 :: PB15
		
 		break;
}
		switch ( Hall_State2 )
{
	  case 0x01: //b-->C
		PWM2_Set_SingleEdge( 0, Vin2, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);// !RESET_C2=LOW, EN2_1 :: 15 :: PA5
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);// !RESET_B2=HIGH, EN2_2 :: 13 :: PA3
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);// !RESET_A2=HIGH, EN2_3 :: 14 :: PA4
		
		break; 
		
	  case 0x02: //b-->a
		PWM2_Set_SingleEdge( 0, Vin2, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);// !RESET_C2=HIGH, EN2_1 :: 15 :: PA5
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);// !RESET_B2=HIGH, EN2_2 :: 13 :: PA3
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);// !RESET_A2=LOW, EN2_3 :: 14 :: PA4
		
		break;
	  
		case 0x03: //c-->a
		PWM2_Set_SingleEdge( 0, 0, Vin2);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);// !RESET_C2=HIGH, EN2_1 :: 15 :: PA5
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);// !RESET_B2=LOW, EN2_2 :: 13 :: PA3
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);// !RESET_A2=HIGH, EN2_3 :: 14 :: PA4
		
		break;

	  case 0x04: //c-->b
		PWM2_Set_SingleEdge( 0, 0, Vin2);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);// !RESET_C2=LOW, EN2_1 :: 15 :: PA5
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);// !RESET_B2=HIGH, EN2_2 :: 13 :: PA3
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);// !RESET_A2=HIGH, EN2_3 :: 14 :: PA4
		
		break;
		
	  case 0x05: //a-->b
		PWM2_Set_SingleEdge( Vin2, 0, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);// !RESET_C2=HIGH, EN2_1 :: 15 :: PA5
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);// !RESET_B2=HIGH, EN2_2 :: 13 :: PA3
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);// !RESET_A2=LOW, EN2_3 :: 14 :: PA4
		
		break;

	  case 0x06: //a-->c
		PWM2_Set_SingleEdge( Vin2, 0, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);// !RESET_C2=HIGH, EN2_1 :: 15 :: PA5
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);// !RESET_B2=LOW, EN2_2 :: 13 :: PA3
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);// !RESET_A2=HIGH, EN2_3 :: 14 :: PA4
		
		break;

	  case 0x07:
		PWM2_Set_SingleEdge( 0, 0, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);// !RESET_C2=LOW, EN2_1 :: 15 :: PA5
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);// !RESET_B2=LOW, EN2_2 :: 13 :: PA3
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);// !RESET_A2=LOW, EN2_3 :: 14 :: PA4
		
 		break;

	  case 0x00: 
		PWM2_Set_SingleEdge( 0, 0, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);// !RESET_C2=LOW, EN2_1 :: 15 :: PA5
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);// !RESET_B2=LOW, EN2_2 :: 13 :: PA3
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);// !RESET_A2=LOW, EN2_3 :: 14 :: PA4
		
 		break;
}
/***************************************/
/**** test pin ******/ 
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
/***************************************/
}

void TIM15_IRQHandler(void) //2KHz
{
  HAL_TIM_IRQHandler(&TimHandle2);
/*************************************************************************/
/*************************************************************************/
    LOOP1_flag=1; 
/*************************************************************************/
/*************************************************************************/
}

void TIM16_IRQHandler(void) //100Hz
{
	  HAL_TIM_IRQHandler(&TimHandle3);
/*************************************************************************/
/*************************************************************************/
   LOOP2_flag=1; 
/*************************************************************************/
/*************************************************************************/
}
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
