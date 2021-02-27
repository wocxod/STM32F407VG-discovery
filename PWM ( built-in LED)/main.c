#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h> 
#include <stm32f4xx_rcc.h> 
#include <stm32f4xx_tim.h> 
#include <stm32f4xx_exti.h> 
#include <misc.h> 
#include <stm32f4xx_syscfg.h>

#define PER 150


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
				if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12)==1 && flag==1)
				{	
					if (oc_init.TIM_Pulse > PER || oc_init.TIM_Pulse < 0) 
					{
						
						GPIO_ToggleBits(GPIOD,GPIO_Pin_14);
						if (half==0)
							{
								half=1;
								
								oc_init.TIM_Pulse = PER-1;
							}else
							{
								half=0;
								oc_init.TIM_Pulse = 1;
							}
							
						
					}
					
					else
					{
						if(half ==0) oc_init.TIM_Pulse+=10;
						else oc_init.TIM_Pulse-=10;
						flag=0;
					}
			
				}
				
				 TIM_OC1Init(TIM4,&oc_init);
				
				
				if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12)==0) flag=1;
			} 
			
			
   
}
