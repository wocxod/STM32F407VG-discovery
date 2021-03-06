#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h> 
#include <stm32f4xx_rcc.h> 
#include <stm32f4xx_tim.h> 
#include <stm32f4xx_exti.h> 
#include <misc.h> 
#include <stm32f4xx_syscfg.h>
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"


#define PER 1000

volatile uint16_t signal=200;

	
	void init_gpio()
		{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); 
	

	GPIO_InitTypeDef init; 
	init.GPIO_Mode = GPIO_Mode_AF; 
	init.GPIO_OType = GPIO_OType_PP; 
	init.GPIO_Pin = GPIO_Pin_12; 
	init.GPIO_Speed = GPIO_Speed_100MHz; 
	GPIO_Init(GPIOD,&init); 
	
				GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4); 

	GPIO_InitTypeDef initF; 
	initF.GPIO_Mode = GPIO_Mode_OUT; 
	initF.GPIO_OType = GPIO_OType_PP; 
	initF.GPIO_Pin = GPIO_Pin_13			|GPIO_Pin_14 ; 
	initF.GPIO_Speed = GPIO_Speed_100MHz; 
	GPIO_Init(GPIOD,&initF); 		
	}
	
void DMA_TIM_init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);                     
	DMA_InitTypeDef          DMA_Setting;

	DMA_Setting.DMA_Channel = DMA_Channel_2;
	DMA_Setting.DMA_BufferSize = sizeof(signal);   
	DMA_Setting.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_Setting.DMA_Memory0BaseAddr = (uint32_t) &signal;
	DMA_Setting.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_Setting.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_Setting.DMA_Mode = DMA_Mode_Circular;
	DMA_Setting.DMA_PeripheralBaseAddr = (uint32_t) &TIM4->CCR1;
	DMA_Setting.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	
	DMA_Setting.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	DMA_Setting.DMA_Priority = DMA_Priority_High;
		
	
	TIM_DMACmd(TIM4,TIM_DMA_CC1,ENABLE);
	
	DMA_ITConfig(DMA1_Stream0,DMA_IT_TC,ENABLE);
	DMA_Init(DMA1_Stream0, &DMA_Setting);

	DMA_Cmd(DMA1_Stream0, ENABLE);
}
void TIM4_IRQHandler ( void );
void tim_init()
{
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 
	 TIM_TimeBaseInitTypeDef base_timer; 
        TIM_TimeBaseStructInit(&base_timer); 
 
        base_timer.TIM_Prescaler = 8400 - 1;    
        base_timer.TIM_Period = PER;  
        base_timer.TIM_CounterMode = TIM_CounterMode_Up; 
        TIM_TimeBaseInit(TIM4, &base_timer); 
 
        TIM_OCInitTypeDef oc_init; 
        TIM_OCStructInit(&oc_init);
        oc_init.TIM_OCMode = TIM_OCMode_PWM1;    
        oc_init.TIM_OutputState = TIM_OutputState_Enable; 
	
         
				
	
        oc_init.TIM_OCPolarity = TIM_OCPolarity_High; 
        TIM_OC1Init(TIM4,&oc_init);  
        TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable); 
       	NVIC_EnableIRQ ( TIM4_IRQn );
        TIM_Cmd(TIM4,ENABLE); 
		
} 


void TIM4_IRQHandler ( void )
{
  if ( TIM_GetITStatus ( TIM4, TIM_IT_Update ) != RESET )
  {
    TIM_ClearITPendingBit ( TIM4, TIM_IT_Update );
    
    GPIO_ToggleBits(GPIOD,GPIO_Pin_14);
		
    ADC_SoftwareStartConv ( ADC1 );   
   
    
  }
} 

 
		static void TimerInit ( void );


void adcStart ( void );


void TimerInit ( void );


void TIM3_IRQHandler ( void );
	
	void init_dma_module(void)
{
	
	 RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_DMA2, ENABLE );
	DMA_InitTypeDef       DMA_InitStructure;
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(ADC3->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&signal;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init ( DMA2_Stream4, &DMA_InitStructure );
  DMA_Cmd ( DMA2_Stream4, ENABLE );
	
	ADC_DeInit ( );     // turn ADC off 
  
  // ADC Common Init
	ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit ( &ADC_CommonInitStructure );

  // ADC1 Init
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init ( ADC1, &ADC_InitStructure );

  
  ADC_RegularChannelConfig ( ADC1, ADC_Channel_3, 1, ADC_SampleTime_3Cycles );

  
  ADC_DMARequestAfterLastTransferCmd ( ADC1, ENABLE );


  ADC_DMACmd ( ADC1, ENABLE );

 
  ADC_Cmd ( ADC1, ENABLE );
	
	 TimerInit ( ); 
	}
	void TimerInit ( void )
{
  TIM_TimeBaseInitTypeDef timerStruct;

  RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM3, ENABLE ); 

 
  timerStruct.TIM_ClockDivision = 0;
  timerStruct.TIM_CounterMode = TIM_CounterMode_Up;
  timerStruct.TIM_Prescaler = 33600-1;  
  timerStruct.TIM_Period = 50;          
  TIM_TimeBaseInit ( TIM3, &timerStruct );

  TIM_ITConfig ( TIM3, TIM_IT_Update, ENABLE );  
  
  NVIC_EnableIRQ ( TIM3_IRQn );
} 


void TIM3_IRQHandler ( void )
{
  if ( TIM_GetITStatus ( TIM3, TIM_IT_Update ) != RESET )
  {
    TIM_ClearITPendingBit ( TIM3, TIM_IT_Update );
    
    GPIO_ToggleBits(GPIOD,GPIO_Pin_13);
		
    ADC_SoftwareStartConv ( ADC1 );   
    
  } 
} 


void adcStart ( void )
{
  TIM_Cmd ( TIM3, ENABLE );
} 


void adcStop ( void )
{
  TIM_Cmd ( TIM3, DISABLE );
} 
	
int main(void)
{  
		
		
   init_gpio(); 
		tim_init();

		init_dma_module();
		adcStart ( );
	
		DMA_TIM_init();
		
		
    while (1)
			{ 
				
			}	
}
