/**
  ******************************************************************************
  * @file    Template/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    17-January-2014
  * @brief   Main program body
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
#include "main.h"
#include "stm32f0xx_gpio.h"
#include "24l01.h"


#define adc_bound		(uint32_t)0x40012440
#define vref				*((uint16_t*)0x1FFFF7BA)

#define MyAdress		0x02

uint16_t data5[150];
uint8_t adc_sayac=0;
uint16_t Adc_Data[3];
_Bool data_ok = 0,control=0 ;
uint8_t sayac=0;
_Bool rtc_alarm;

__IO _Bool charge=0;
uint8_t eslesme_tamam=0;

extern __IO uint32_t TimingDelay , TimingDelay2 ;
extern __IO uint32_t CaptureNumber, PeriodValue;

void delay(uint16_t delayy);
void init_Adc(void);
void dma1(void);
void Perform_measurement(void);
void init_I2C1(void);
void GPIO_init1(void);
extern float get_temp(float pt_value);
extern void NRF24L01_Init(void);
extern _Bool Si7013_Detect( tip *deviceId);
extern uint8_t NRF24L01_Read_Reg(uint8_t reg);
static uint32_t GetLSIFrequency(void);

static void EXTI_Config(void);
void GPIO_STOP(void);

uint16_t sayac1=0;
uint8_t reg=0,reg2=0;

float Bat_voltage=0.0f,Gas_voltage=0.0f;
float Vdda=3.35f;

_Bool Si7021_ok=0;

tip tip1[16];
tip tip2;

uint8_t gelen2[30];
uint8_t giden[30];

int32_t temp_data=0,rh_data=0;

float temp=0.0f,rh=0.0f;

uint32_t freq=0;
int main(void)
{
		
	//SysTick_Config((uint32_t)SysTick_CLKSource_HCLK_Div8/300);
	
	GPIO_init1();
	init_Adc();	
	init_I2C1();
	NRF24L01_Init();
	NRF24L01_RX_Mode();
	EXTI_Config();
	RTC_Config();
	//SysTick_Configuration();
	//RTC_AlarmConfig();
	
	//Si7021_ok=Si7013_Detect(tip1);

	  /* SysTick interrupt each 10 ms */
  if (SysTick_Config(SystemCoreClock / 100))
  { 
    while (1);
  }
	
	
	DBGMCU_Config(DBGMCU_CR_DBG_STOP,ENABLE);
	
	reg=NRF24L01_Read_Reg(0x00);
	reg=NRF24L01_Read_Reg(0x01);
	reg=NRF24L01_Read_Reg(0x02);
	reg=NRF24L01_Read_Reg(0x03);
	reg=NRF24L01_Read_Reg(0x04);
	reg=NRF24L01_Read_Reg(0x05);
	reg=NRF24L01_Read_Reg(0x06);
	reg=NRF24L01_Read_Reg(0x07);
	
  while (1)
  {
		
		RTC_AlarmConfig();
		NRF24L01_SLEEP_Mode();
		GPIO_STOP();
		PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

    RTC_ITConfig(RTC_IT_ALRA, DISABLE);
    RTC_AlarmCmd(RTC_Alarm_A, DISABLE);
		
    SYSCLKConfig_STOP();
		
		//GPIO_init1();
		//init_Adc();	
		//init_I2C1();
		//NRF24L01_Init();
		//NRF24L01_RX_Mode();
		//EXTI_Config();
		//freq=GetLSIFrequency();
		//GPIO_init1();
		//init_Adc();	
		//EXTI_Config();
		NRF24L01_WAKEUP_Mode();
		////Perform_measurement();
		Delay(50);	
		TimingDelay2=0;
		
		while(TimingDelay2<=1000)
		{
			NRF24L01_RxPacket(gelen2);
			if(gelen2[0]==MyAdress)
			{
				//EXTI_DeInit();
				Perform_measurement();
				
				gelen2[0]=0;
				NRF24L01_TX_Mode();
				
				sprintf((char *)giden,"%1.2f %1.2f %1.2f %2.2f %2.1f",Bat_voltage,Gas_voltage,Vdda,temp,rh);
				
				NRF24L01_TxPacket(giden);
				
				delay(10);
				NRF24L01_RX_Mode();
				break;
			}
		}
	}
			
}  


void delay(uint16_t delayy)
{
	uint16_t temp=0;
	
	for(temp=0;temp<delayy;temp++)
	{
		__nop();
		__nop();
		__nop();
				__nop();
		__nop();
		__nop();
				__nop();
		__nop();
		__nop();
				__nop();
		__nop();
		__nop();
				__nop();
		__nop();
		__nop();
				__nop();
		__nop();
		__nop();
				__nop();
		__nop();
		__nop();
				__nop();
		__nop();
		__nop();
				__nop();
		__nop();
		__nop();
				__nop();
		__nop();
		__nop();
				__nop();
		__nop();
		__nop();
				__nop();
		__nop();
		__nop();
	}


}
uint8_t cal_fac=0;
void init_Adc(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		ADC_InitTypeDef adc;
		NVIC_InitTypeDef int_;
	
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0 | GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN;
		GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
		
		GPIO_Init(GPIOA,&GPIO_InitStructure);
		
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    adc.ADC_Resolution=ADC_Resolution_12b;
    adc.ADC_DataAlign=ADC_DataAlign_Right;
    adc.ADC_ContinuousConvMode=ENABLE;
    adc.ADC_ExternalTrigConv=ADC_ExternalTrigConv_T1_TRGO;
    adc.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None;
    adc.ADC_ScanDirection=ADC_ScanDirection_Upward;


    ADC_Init(ADC1,&adc);
    ADC_ClockModeConfig(ADC1,ADC_ClockMode_SynClkDiv2);
    

    ADC_ChannelConfig(ADC1,ADC_Channel_0,ADC_SampleTime_239_5Cycles);
		ADC_ChannelConfig(ADC1,ADC_Channel_1,ADC_SampleTime_239_5Cycles );
		ADC_ChannelConfig(ADC1, ADC_Channel_Vrefint , ADC_SampleTime_239_5Cycles );
		
		ADC_VrefintCmd(ENABLE);
		//ADC_TempSensorCmd(ENABLE);
		
    cal_fac=ADC_GetCalibrationFactor(ADC1);

    

    //ADC_DMACmd(ADC1,ENABLE);
		
		int_.NVIC_IRQChannel=ADC1_COMP_IRQn;
		int_.NVIC_IRQChannelCmd=ENABLE;
		int_.NVIC_IRQChannelPriority=4;
		
		NVIC_Init(&int_);
		
		ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);
		ADC_ITConfig(ADC1,ADC_IT_EOSEQ,ENABLE);


    

    //ADC_StartOfConversion(ADC1);

}

void dma1(void)
{
	DMA_InitTypeDef	dma;
	
	NVIC_InitTypeDef int_;
	
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

		dma.DMA_BufferSize=3;
		//dma.DMA_DIR=DMA_DIR_PeripheralDST;
		dma.DMA_DIR=DMA_DIR_PeripheralSRC;
		dma.DMA_MemoryInc=DMA_MemoryInc_Enable;
		dma.DMA_Mode=DMA_Mode_Circular;
		//dma.DMA_PeripheralBaseAddr=tim3_bound ;
		dma.DMA_PeripheralBaseAddr=adc_bound;
		dma.DMA_MemoryBaseAddr =(uint32_t)&Adc_Data;
		dma.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;
		dma.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
		dma.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
		dma.DMA_Priority=DMA_Priority_High;
		dma.DMA_M2M=DISABLE;
	
		DMA_Init(DMA1_Channel1,&dma);
		DMA_Cmd(DMA1_Channel1, ENABLE); 
		
		DMA_ClearITPendingBit(DMA_IT_TC);
		
		DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);
		
		int_.NVIC_IRQChannel=DMA1_Channel1_IRQn;
		int_.NVIC_IRQChannelCmd=ENABLE;
		int_.NVIC_IRQChannelPriority=3;
		NVIC_Init(&int_);
}//


void Perform_measurement(void)
{
    uint32_t temp1_bat=0;
		uint32_t temp2_gas=0;
		uint32_t Temp3_vdda=0;
	
			ADC_Cmd(ADC1,ENABLE);
		while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY));
		
		//uint8_t sayac11=0;
	
		adc_sayac=0;

	
		ADC_StartOfConversion(ADC1);
	
		while(data_ok==0);
		data_ok=0;
		ADC_StopOfConversion(ADC1);
		ADC_Cmd(ADC1,DISABLE);
		for(sayac=0;sayac<150;sayac+=3)
		{

		//VSense=vref;
			temp1_bat +=  data5[sayac];
		//VSense=vref;
			temp2_gas += data5[sayac+1];
		
			Temp3_vdda += data5[sayac+2];
		}
	
		
		temp1_bat= temp1_bat / 50;
		temp2_gas = temp2_gas /50;
		Temp3_vdda = Temp3_vdda/50;
		
		Vdda = 3.3f * ((float)vref / (float)Temp3_vdda);
		Bat_voltage= Vdda*2 *(temp1_bat/4096.0f);
		Gas_voltage=Vdda*2 * (temp2_gas/4096.0f);
		
		temp=0;
		rh=0;
		
		for(sayac=0;sayac<10;sayac++)
		{
		
			Si7013_ReadNoHoldRHAndTemp(&rh_data,&temp_data);
			temp+=temp_data;
			rh+=rh_data;
		
		}
		temp=temp/10000.0f;
		rh=rh/10000.0f;
		
		if(Bat_voltage>3.5f)
		{
				//GPIO_SetBits(GPIOB,GPIO_Pin_5);
		}
		else
		{
			//GPIO_ResetBits(GPIOB,GPIO_Pin_5);
		}
		if(Bat_voltage>3.8f)
		{
				//GPIO_SetBits(GPIOB,GPIO_Pin_4);
		}
		else
		{
			//GPIO_ResetBits(GPIOB,GPIO_Pin_4);
		}
		if(Bat_voltage>4.1f)
		{
				//GPIO_SetBits(GPIOB,GPIO_Pin_3);
		}
		else
		{
			//GPIO_ResetBits(GPIOB,GPIO_Pin_3);
		}
		
		
}

void GPIO_init1(void)
{
	
	GPIO_InitTypeDef gpio_init;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);

	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOBEN, ENABLE);
		
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5  ;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_InitStructure.GPIO_OType= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	gpio_init.GPIO_Mode=GPIO_Mode_OUT;
	gpio_init.GPIO_OType=GPIO_OType_PP;
	gpio_init.GPIO_Pin=_5v_pin;
	gpio_init.GPIO_PuPd=GPIO_PuPd_NOPULL;
	gpio_init.GPIO_Speed=GPIO_Speed_10MHz;
	
	GPIO_Init( _5v_port,&gpio_init);
	
	gpio_init.GPIO_Pin=_Gas_pin;
	
	GPIO_Init(_Gas_port,&gpio_init);
	
	GPIO_SetBits(_5v_port,_5v_pin);
	GPIO_SetBits(_Gas_port,_Gas_pin);
}

void init_I2C1(void)
{
	
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;
	RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);
	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	// enable clock for SCL and SDA pins
	
	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOBEN,ENABLE);

	
	/* setup SCL and SDA pins
	 * You can connect I2C1 to two different
	 * pairs of pins:
	 * 1. SCL on PB6 and SDA on PB7 
	 * 2. SCL on PB8 and SDA on PB9
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // we are going to use PB6 and PB7
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;		// set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;			// enable pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// init GPIOB
	
	// Connect I2C1 pins to AF  
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_1);	// SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_1); // SDA
	
	// configure I2C1 
	I2C_InitStruct.I2C_Timing = 100000; 		// 100kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;		// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_InitStruct.I2C_AnalogFilter= I2C_AnalogFilter_Enable;
	I2C_InitStruct.I2C_DigitalFilter=0;
	I2C_Init(I2C1, &I2C_InitStruct);				// init I2C1
	
	// enable I2C1
	I2C_Cmd(I2C1, ENABLE);
}

static void EXTI_Config(void)
{
	
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
  /* Enable GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  /* Configure PC13 pins as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	  /* Configure PC13 pins as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = Buton_pin;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Connect EXTI13 Line to PC13 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource10);
	EXTI_ClearITPendingBit(EXTI_Line10);
	
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource15);
	EXTI_ClearITPendingBit(EXTI_Line15);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource1);
	EXTI_ClearITPendingBit(EXTI_Line1);

  /* Configure EXTI13 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line10;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	  /* Configure EXTI1 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_Init(&EXTI_InitStructure);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line15;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI4_15 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	  /* Enable and set EXTI1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
  NVIC_Init(&NVIC_InitStructure);
}

static void RTC_Config(void)
{
  RTC_TimeTypeDef   RTC_TimeStructure;
  RTC_InitTypeDef   RTC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	
	uint32_t LSIFreq = 0;
  
  /* RTC Configuration **********************************************************/ 
  /* Enable the PWR clock */

  	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  
  /* Allow access to RTC */
		PWR_BackupAccessCmd(ENABLE);

  /* Reset back up registers */
		RCC_BackupResetCmd(ENABLE);
		RCC_BackupResetCmd(DISABLE);
  /* Enable the LSE */
  RCC_LSICmd(ENABLE);
  
  /* Wait till LSE is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
  {}
  
  /* Select the RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
  
  /* Enable the RTC Clock */
  RCC_RTCCLKCmd(ENABLE);
 
  RTC_DeInit(); 
  /* Wait for RTC APB registers synchronisation */
  RTC_WaitForSynchro();  
		
	LSIFreq = GetLSIFrequency();
  
  /* Set RTC calendar clock to 1 HZ (1 second) */
  RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
  RTC_InitStructure.RTC_AsynchPrediv = 99;
  RTC_InitStructure.RTC_SynchPrediv = (LSIFreq/100) - 1;
  
  if (RTC_Init(&RTC_InitStructure) == ERROR)
  {
    while(1);
  }

  /* Set the time to 01h 00mn 00s AM */
  RTC_TimeStructure.RTC_H12     = RTC_H12_AM;
  RTC_TimeStructure.RTC_Hours   = 0x01;
  RTC_TimeStructure.RTC_Minutes = 0x00;
  RTC_TimeStructure.RTC_Seconds = 0x00;  
  
  RTC_SetTime(RTC_Format_BCD, &RTC_TimeStructure);
    
  /* Configure EXTI line 17 (connected to the RTC Alarm event) */
  EXTI_ClearITPendingBit(EXTI_Line17);
  EXTI_InitStructure.EXTI_Line = EXTI_Line17;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* NVIC configuration */
  NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
}


static void RTC_AlarmConfig(void)
{  
  RTC_TimeTypeDef   RTC_TimeStructure;
  RTC_AlarmTypeDef  RTC_AlarmStructure;
	


  /* Get current time */
  //RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);

  /* Set the alarm to current time + 5s */
  RTC_AlarmStructure.RTC_AlarmTime.RTC_H12     = RTC_H12_AM;
  RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours   = 0x01;
  RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = 0x01;
  RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = 0x00;
  RTC_AlarmStructure.RTC_AlarmDateWeekDay = 31;
  RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
  RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay | RTC_AlarmMask_Minutes |
                                     RTC_AlarmMask_Hours;
  RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure);
	
		  /* Set the time to 01h 00mn 00s AM */
  RTC_TimeStructure.RTC_H12     = RTC_H12_AM;
  RTC_TimeStructure.RTC_Hours   = 0x01;
  RTC_TimeStructure.RTC_Minutes = 0x00;
  RTC_TimeStructure.RTC_Seconds = 0x00;  
  
  RTC_SetTime(RTC_Format_BCD, &RTC_TimeStructure);
   
  /* Enable the RTC Alarm A interrupt */
  RTC_ITConfig(RTC_IT_ALRA, ENABLE);

  /* Enable the alarm */
  RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
    
  /* Clear the Alarm A Pending Bit */
  RTC_ClearITPendingBit(RTC_IT_ALRA);  
}

static void SYSCLKConfig_STOP(void)
{  
  /* After wake-up from STOP reconfigure the system clock */
  /* Enable HSE */
  RCC_HSICmd(ENABLE);
  
  /* Wait till HSE is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET)
  {}
  
  /* Enable PLL */
  RCC_PLLCmd(ENABLE);
  
  /* Wait till PLL is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
  {}
  
  /* Select PLL as system clock source */
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  
  /* Wait till PLL is used as system clock source */
  while (RCC_GetSYSCLKSource() != 0x08)
  {}
}

static uint32_t GetLSIFrequency(void)
{
  NVIC_InitTypeDef   NVIC_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  RCC_ClocksTypeDef  RCC_ClockFreq;

  /* TIM14 configuration *******************************************************/ 
  /* Enable TIM14 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
  
  /* Reset TIM14 registers */
  TIM_DeInit(TIM14);

  /* Configure TIM14 prescaler */
  TIM_PrescalerConfig(TIM14, 0, TIM_PSCReloadMode_Immediate);

  /* Connect internally the TIM14_CH1 to the RTC clock output */
  TIM_RemapConfig(TIM14, TIM14_RTC_CLK);

  /* TIM14 configuration: Input Capture mode ---------------------
     The reference clock(LSE or external) is connected to TIM14 CH1
     The Rising edge is used as active edge,
     The TIM14 CCR1 is used to compute the frequency value 
  ------------------------------------------------------------ */
  TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM14, &TIM_ICInitStructure);

  /* Enable the TIM14 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable TIM14 counter */
  TIM_Cmd(TIM14, ENABLE);

  /* Reset the flags */
  TIM14->SR = 0;
    
  /* Enable the CC1 Interrupt Request */  
  TIM_ITConfig(TIM14, TIM_IT_CC1, ENABLE);


  /* Wait until the TIM14 get 2 LSI edges (refer to TIM14_IRQHandler() in 
    stm32F0xx_it.c file) ******************************************************/
  while(CaptureNumber != 2)
  {
  }
  /* Deinitialize the TIM14 peripheral registers to their default reset values */
  TIM_DeInit(TIM14);


  /* Compute the LSI frequency, depending on TIM14 input clock frequency (PCLK1)*/
  /* Get SYSCLK, HCLK and PCLKx frequency */
  RCC_GetClocksFreq(&RCC_ClockFreq);
  
  /* PCLK1 prescaler equal to 1 => TIMCLK = PCLK1 */
  return ((RCC_ClockFreq.PCLK_Frequency / PeriodValue) * 8);
}

void SysTick_Configuration(void)
{
  /* SysTick interrupt each 250 ms */
  if (SysTick_Config((SystemCoreClock/8) / 4))
  { 
    /* Capture error */ 
    while (1);
  }

  /* Select AHB clock(HCLK) divided by 8 as SysTick clock source */
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);

  /* Set SysTick Preemption Priority to 1 */
  NVIC_SetPriority(SysTick_IRQn, 0x04);
}
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);

}

void Make_packet(uint8_t *vdata,uint8_t count)
{

	
}

void GPIO_STOP(void)
{
	GPIO_InitTypeDef gpio_init;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
	
	gpio_init.GPIO_Mode=GPIO_Mode_AN;
	gpio_init.GPIO_OType=GPIO_OType_OD;
	gpio_init.GPIO_Pin=GPIO_Pin_All;
	gpio_init.GPIO_PuPd=GPIO_PuPd_NOPULL;
	
	
	//GPIO_Init( GPIOA,&gpio_init);
	//GPIO_Init( GPIOB,&gpio_init);
	
	GPIO_Write(GPIOA,0x0000);
	GPIO_Write(GPIOB,0x0000);
	
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,DISABLE);
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,DISABLE);
	


	
	//ADC_Cmd(ADC1,DISABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);
}

void NRF_Eslestir(void)
{
			
			uint32_t base_add=0;
	
			NRF24L01_TX_Mode();
			delay(100);
			//led_hizli_flas();
			sprintf((char *)giden,"A%04u",base_add);
				
			NRF24L01_TxPacket(giden);
			while(1)
			{
				gelen2[0]=0;
				
				
				NRF24L01_RX_Mode();
				
				while(NRF24L01_RxPacket(gelen2));
				
				if(gelen2[0]==0x01)
					break;
				
				NRF24L01_TX_Mode();
						
				sprintf((char *)giden,"A%04u",base_add);
				delay(10);
				NRF24L01_TxPacket(giden);
				base_add++;
				//A0000001
				
				//delay(10);
				//NRF24L01_RX_Mode();
				//delay(100);
				//NRF24L01_RxPacket(gelen2);
				//if(gelen2!=0)
				//		eslesme_tamam=1;
				//else	
						
			}
			led_yavas_flas();
			eslesme_tamam=1;
	

}
uint16_t PrescalerValue=0;
void led_yavas_flas(void)
{
	
	
	
	TIM_TimeBaseInitTypeDef tim1;
	NVIC_InitTypeDef nvic;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
	TIM_DeInit(TIM1);
	
	PrescalerValue = (uint16_t) (SystemCoreClock  / 6000000) - 1;
	
	tim1.TIM_ClockDivision=0;
	tim1.TIM_CounterMode=TIM_CounterMode_Up;
	tim1.TIM_Period=1000;
	tim1.TIM_Prescaler=1000;
	tim1.TIM_RepetitionCounter=0;
	
	TIM_TimeBaseInit(TIM1,&tim1);
	
	  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM1, PrescalerValue, TIM_PSCReloadMode_Immediate);
	
	nvic.NVIC_IRQChannel=TIM1_BRK_UP_TRG_COM_IRQn;
	nvic.NVIC_IRQChannelCmd=ENABLE;
	nvic.NVIC_IRQChannelPriority=1;
	
	NVIC_Init(&nvic);
	
	TIM_ClearFlag(TIM1,TIM_FLAG_Update);
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
	
	TIM_Cmd(TIM1,ENABLE);
	
}

void led_hizli_flas(void)
{
	
		NVIC_InitTypeDef nvicStructure;
	  TIM_TimeBaseInitTypeDef timerInitStructure; 

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
 

    timerInitStructure.TIM_Prescaler = 40000;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 50000;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &timerInitStructure);
    
	

    nvicStructure.NVIC_IRQChannel = TIM3_IRQn;
    nvicStructure.NVIC_IRQChannelPriority=1;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
		
    NVIC_Init(&nvicStructure);
		
		TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
		
		TIM_Cmd(TIM3, ENABLE);

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

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
