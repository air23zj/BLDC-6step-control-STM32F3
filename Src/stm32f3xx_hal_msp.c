/**
  ******************************************************************************
  * @file    TIM/TIM_PWMOutput/Src/stm32f3xx_hal_msp.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-June-2014
  * @brief   HAL MSP module.
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

/** @addtogroup STM32F3xx_HAL_Examples
  * @{
  */

/** @defgroup HAL_MSP
  * @brief HAL MSP module.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */


/**
  * @brief TIM MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx Peripheral clock enable */
  __TIM3_CLK_ENABLE();
  __TIM2_CLK_ENABLE();
	
  /* Enable GPIO Channels Clock */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
	
		/* Pin configurations */
		
  /**** PWM ***
	IN1_3 		:: 16 :: PA6 :: TIM3_CH1, !!!!!, AF2
	IN1_1 		:: 17 :: PA7 :: TIM3_CH2, !!!!!, AF2
	IN1_2 		:: 18 :: PB0 :: TIM3_CH3, !!!!!, AF2
	
	IN2_1 		:: 10 :: PA0 :: TIM2_CH1, AF1
	IN2_2 		:: 11 :: PA1 :: TIM2_CH2, AF1
	IN2_3 		:: 12 :: PA2 :: TIM2_CH3, AF1
	*/
	
	GPIO_InitStruct.Pin = ( GPIO_PIN_7 | GPIO_PIN_6 );
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN; //GPIO_PULLUP
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate=GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
	
  GPIO_InitStruct.Pin = (GPIO_PIN_0);
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN; //GPIO_PULLUP
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH; 
	GPIO_InitStruct.Alternate=GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 
	
	GPIO_InitStruct.Pin = ( GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN; //GPIO_PULLUP
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate=GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
	
}
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
	
	/*
TIM6_DAC_IRQn //16-bit time base, cout up, Prescaler 1~65536
TIM7_IRQn //16-bit time base, cout up, Prescaler 1~65536
TIM1_BRK_TIM15_IRQn //16-bit time base, cout up, Prescaler 1~65536
TIM1_UP_TIM16_IRQn //16-bit time base, cout up, Prescaler 1~65536
TIM1_TRG_COM_TIM17_IRQn //16-bit time base, cout up, Prescaler 1~65536
//TIM8, 16-bit time base, cout up/down, Prescaler 1~65536
*/
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx Peripheral clock enable */
  __TIM7_CLK_ENABLE();
  __TIM15_CLK_ENABLE();
  __TIM16_CLK_ENABLE();
	
  /*##-2- Configure the NVIC for TIMx ########################################*/
  /* Set Interrupt Group Priority */ 
  HAL_NVIC_SetPriority(TIM7_IRQn, 1, 0);
  HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 2, 0);
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 3, 0);
	
  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
	
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
	/*
(#) Initialize the TIM low level resources by implementing the following functions 
        (++) Encoder mode output : HAL_TIM_Encoder_MspInit()
(#) Initialize the TIM low level resources :
        (##) Enable the TIM interface clock using __TIMx_CLK_ENABLE(); 
        (##) TIM pins configuration
            (+++) Enable the clock for the TIM GPIOs using the following function:
             __GPIOx_CLK_ENABLE();   
            (+++) Configure these TIM pins in Alternate function mode using HAL_GPIO_Init();  
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
  GPIO_InitTypeDef   GPIO_InitStruct;
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx Peripheral clock enable */
	__TIM4_CLK_DISABLE();
  __TIM4_CLK_ENABLE();
	__TIM1_CLK_ENABLE();

	
  /* Enable GPIO Channels Clock */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
	
		/* Pin configurations */
		
  /**** Encoder ***
	ENC1_A 		:: 29 :: PA8 :: TIM1_CH1, AF6
	ENC1_B 		:: 30 :: PA9 :: TIM1_CH2, AF6
	
	ENC2_A 		:: 42 :: PB6 :: TIM4_CH1, AF2
	ENC2_B 		:: 43 :: PB7 :: TIM4_CH2, AF2
	*/

	GPIO_InitStruct.Pin = (GPIO_PIN_7 | GPIO_PIN_6);
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP; //GPIO_PULLUP
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH; 
	GPIO_InitStruct.Alternate=GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 
	
	GPIO_InitStruct.Pin = ( GPIO_PIN_9 | GPIO_PIN_8 );
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP; //GPIO_PULLUP
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate=GPIO_AF6_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
	

	
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
