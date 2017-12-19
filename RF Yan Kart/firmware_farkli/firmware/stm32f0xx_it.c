/**
  ******************************************************************************
  * @file    Template/stm32f0xx_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    17-January-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_it.h"
#include "main.h"

uint16_t a;
extern uint8_t tus,tus2,basildi;
uint16_t one_sec=0;
__IO uint32_t CaptureNumber, PeriodValue;
uint32_t IC1ReadValue1 = 0, IC1ReadValue2 =0;
__IO uint32_t TimingDelay = 0, TimingDelay2 = 0;

extern __IO _Bool charge;

extern uint16_t data5[150];
extern uint8_t adc_sayac;
extern _Bool data_ok,control;
extern float Get_Int_Temp(void);

extern _Bool rtc_alarm;
extern uint8_t gelen2[30];

extern uint8_t NRF24L01_RxPacket(uint8_t *rxbuf);

void get_rtcc(void);
/** @addtogroup STM32F0xx_DISCOVERY_Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
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
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
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
	TimingDelay--;
	TimingDelay2++;
	
}

void ADC1_IRQHandler(void)
{
	if(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==SET)
	{
		ADC_ClearITPendingBit(ADC1,ADC_FLAG_EOC);
		
		data5[adc_sayac++]=ADC_GetConversionValue(ADC1);
		
		if(adc_sayac==150)
		{
				data_ok=1;
				adc_sayac=0;
		}
		//TIM3->CCR1=a>>1;
		//TIM3->CCR2=a>>1;
		//TIM3->CCR3=a>>1;
		//TIM3->CCR4=a>>1;
	
	}
	
		if(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOSEQ)==SET)
	{
		ADC_ClearITPendingBit(ADC1,ADC_FLAG_EOSEQ);
		

	
	}
	
	


}//
uint8_t gelen[30];
uint8_t temp22;
int8_t bitval=0;
void EXTI4_15_IRQHandler(void)
{
	
	
  if(EXTI_GetITStatus(EXTI_Line10) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line10);
		NRF24L01_RxPacket(gelen2);

		//NRF24L01_Write_Reg(0xE2,0xff);//Çå³ýRX FIFO¼Ä´æÆ÷ 
		//NRF24L01_RxPacket(&gelen);
		//temp22=NRF24L01_Read_Reg(0x11);
		//NRF24L01_Write_Reg(0x31,temp22);
  }
	
	if(EXTI_GetITStatus(EXTI_Line15) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line15);
		NRF_Eslestir();
  }
}

void EXTI0_1_IRQHandler(void)
{
	
  if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line1);
		charge=1;
  }
	

}

void RTC_IRQHandler(void)
{
  if (RTC_GetITStatus(RTC_IT_ALRA) != RESET)
  {
		//bitval=GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_3);
		//GPIO_WriteBit(GPIOB,GPIO_Pin_3,!bitval);
    /* Clear the Alarm A Pending Bit */
    RTC_ClearITPendingBit(RTC_IT_ALRA);
    
    /* Clear EXTI line17 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line17);    
  }  
}


void TIM14_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM14, TIM_IT_CC1) != RESET)
  {
    /* Clear TIM14 Capture Compare 1 interrupt pending bit */
    TIM_ClearITPendingBit(TIM14, TIM_IT_CC1);
    
    if(CaptureNumber == 0)
    {
      /* Get the Input Capture value */
      IC1ReadValue1 = TIM_GetCapture1(TIM14);
      CaptureNumber = 1;
    }
    else if(CaptureNumber == 1)
    {
       /* Get the Input Capture value */
       IC1ReadValue2 = TIM_GetCapture1(TIM14); 
       TIM_ITConfig(TIM14, TIM_IT_CC1, DISABLE);

       /* Capture computation */
       if (IC1ReadValue2 > IC1ReadValue1)
       {
         PeriodValue = (IC1ReadValue2 - IC1ReadValue1);
       }
       else
       {
         PeriodValue = ((0xFFFF - IC1ReadValue1) + IC1ReadValue2);
       }
       /* capture of two values is done */
       CaptureNumber = 2;
    }
  }
}

void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)!=RESET)
	{
		TIM_ClearFlag(TIM3,TIM_FLAG_Update);
		bitval=GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_3);
		GPIO_WriteBit(GPIOB,GPIO_Pin_3,!bitval);
	}


}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                            */
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
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
