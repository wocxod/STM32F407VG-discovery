#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h> 
#include <stm32f4xx_rcc.h> 
#include <stm32f4xx_tim.h> 
#include <stm32f4xx_exti.h> 
#include <misc.h> 
#include <stm32f4xx_syscfg.h>
#include "stm32f4xx_adc.h"

#define PER 10

	
	u16 readADC1(u8 channel)
{
  
  ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_3Cycles);
 
  ADC_SoftwareStartConv(ADC1);
 
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
 
  return ADC_GetConversionValue(ADC1);
}
	
	
	void init_ADC()
	{
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE); 
		ADC_DeInit();
		 
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
		GPIO_InitTypeDef porta_setup;                                         
		porta_setup.GPIO_Mode = GPIO_Mode_AN;             
		porta_setup.GPIO_Pin = GPIO_Pin_3;                
		GPIO_Init(GPIOA, &porta_setup);                       
				
				ADC_InitTypeDef adc_setup; 
		adc_setup.ADC_ContinuousConvMode = DISABLE; 
		adc_setup.ADC_Resolution = ADC_Resolution_12b; 
		adc_setup.ADC_ScanConvMode = DISABLE; 
		adc_setup.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None; 
		adc_setup.ADC_DataAlign = ADC_DataAlign_Right; 
		 
		ADC_Init(ADC1, &adc_setup); 
		ADC_Cmd(ADC1,ENABLE); 
		
	}
	
	
	
	void init_gpio()
		{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 

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
	initF.GPIO_Pin = GPIO_Pin_14; 
	initF.GPIO_Speed = GPIO_Speed_100MHz; 
	GPIO_Init(GPIOD,&initF); 		
	}
	


  

 
int main(void)
{  
		int flag = 1;
		int half =0;
		init_ADC();
   init_gpio(); 
	
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
	
        oc_init.TIM_Pulse = 1;   

	
        oc_init.TIM_OCPolarity = TIM_OCPolarity_High; 
        TIM_OC1Init(TIM4,&oc_init);  
        TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable); 
       
        TIM_Cmd(TIM4,ENABLE); 
    while (1)
			{ 
				int period;
				
				oc_init.TIM_Pulse = readADC1(ADC_Channel_3)/500;
					 TIM_OC1Init(TIM4,&oc_init);					
										
			}	
}

