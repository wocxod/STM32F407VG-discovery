#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_tim.h"


GPIO_InitTypeDef gpio;
EXTI_InitTypeDef exti; 
NVIC_InitTypeDef nvic;
I2S_InitTypeDef i2s;
I2C_InitTypeDef i2c;
TIM_TimeBaseInitTypeDef timer;
uint8_t state = 0x00;
int p =1;
int f=0;


void Delay(uint32_t s)                                    
{
    volatile uint32_t i;                                    
    RCC_ClocksTypeDef rcc;                                 
 
    RCC_GetClocksFreq (&rcc);                              
    i = (rcc.HCLK_Frequency/16)*s;                         
 
    for (i; i != 0; i--);                                 
}

void initGPIO()
{
    // Включаем тактирование
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 | RCC_APB1Periph_SPI3, ENABLE);
    
    // У I2S свой отдельный источник тактирования, имеющий повышенную точность, включаем и его
    RCC_PLLI2SCmd(ENABLE);
    // Reset сигнал для CS43L22
    gpio.GPIO_Pin = GPIO_Pin_4;;
	
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &gpio);
    // Выводы I2C1
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_OD;
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
    // А теперь настраиваем выводы I2S
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_10 | GPIO_Pin_12;
    GPIO_Init(GPIOC, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOA, &gpio);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);
		
//		gpio.GPIO_Pin = GPIO_Pin_0;
//		gpio.GPIO_Mode = GPIO_Mode_IN;
//		gpio.GPIO_OType = GPIO_OType_OD;
//		gpio.GPIO_Speed = GPIO_Speed_2MHz;
//		gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
//		GPIO_Init(GPIOA,&gpio);
		
		GPIO_InitTypeDef initF; 
	initF.GPIO_Mode = GPIO_Mode_OUT; 
	initF.GPIO_OType = GPIO_OType_PP; 
	initF.GPIO_Pin = GPIO_Pin_13			|GPIO_Pin_14 ; 
	initF.GPIO_Speed = GPIO_Speed_100MHz; 
	GPIO_Init(GPIOD,&initF); 		
	
	
	
    // Сбрасываем Reset в ноль
    GPIO_ResetBits(GPIOD, GPIO_Pin_4);
}

void initI2S()
{
    SPI_I2S_DeInit(SPI3);
    i2s.I2S_AudioFreq = I2S_AudioFreq_48k;
    i2s.I2S_MCLKOutput = I2S_MCLKOutput_Enable;
    i2s.I2S_DataFormat = I2S_DataFormat_16b;
    i2s.I2S_Mode = I2S_Mode_MasterTx;
    i2s.I2S_Standard = I2S_Standard_Phillips;
    i2s.I2S_CPOL = I2S_CPOL_Low;
    I2S_Init(SPI3, &i2s);
}

void initI2C()
{
    I2C_DeInit(I2C1);
    i2c.I2C_ClockSpeed = 100000;
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_OwnAddress1 = 0x33;
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_Cmd(I2C1, ENABLE);
    I2C_Init(I2C1, &i2c);
}

void writeI2CData(uint8_t bytesToSend[], uint8_t numOfBytesToSend)
{
    uint8_t currentBytesValue = 0;
    // Ждем пока шина освободится
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
    
    // Генерируем старт
    I2C_GenerateSTART(I2C1, ENABLE);
    while (!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB));
    
    // Посылаем адрес микросхеме 
    I2C_Send7bitAddress(I2C1, 0x94, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    // И наконец отправляем наши данные
    while (currentBytesValue < numOfBytesToSend)
    {
        I2C_SendData(I2C1, bytesToSend[currentBytesValue]);
        currentBytesValue++;
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
    }
    while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF));
    I2C_GenerateSTOP(I2C1, ENABLE);
}

void delay(uint32_t delayTime)
{
    uint32_t i = 0;
    for (i = 0; i < delayTime; i++);
}

void initCS32L22()
{
    uint8_t sendBuffer[2];
    GPIO_SetBits(GPIOD, GPIO_Pin_4);
    delay(0xFFFF);
    delay(0xFFFF);
    delay(0xFFFF);
    delay(0xFFFF);
    delay(0xFFFF);
    sendBuffer[0] = 0x0D;
    sendBuffer[1] = 0x01;
    writeI2CData(sendBuffer, 2);
    sendBuffer[0] = 0x00;
    sendBuffer[1] = 0x99;
    writeI2CData(sendBuffer, 2);
    sendBuffer[0] = 0x47;
    sendBuffer[1] = 0x80;
    writeI2CData(sendBuffer, 2);
    sendBuffer[0] = 0x32;
    sendBuffer[1] = 0xFF;
    writeI2CData(sendBuffer, 2);
    sendBuffer[0] = 0x32;
    sendBuffer[1] = 0x7F;
    writeI2CData(sendBuffer, 2);
    sendBuffer[0] = 0x00;
    sendBuffer[1] = 0x00;
    writeI2CData(sendBuffer, 2);
    sendBuffer[0] = 0x04;
    sendBuffer[1] = 0xAF;
    writeI2CData(sendBuffer, 2);
    sendBuffer[0] = 0x0D;
    sendBuffer[1] = 0x70;
    writeI2CData(sendBuffer, 2);
    sendBuffer[0] = 0x05;
    sendBuffer[1] = 0x81;
    writeI2CData(sendBuffer, 2);
    sendBuffer[0] = 0x06;
    sendBuffer[1] = 0x07;
    writeI2CData(sendBuffer, 2);
    sendBuffer[0] = 0x0A;
    sendBuffer[1] = 0x00;
    writeI2CData(sendBuffer, 2);
    sendBuffer[0] = 0x27;
    sendBuffer[1] = 0x00;
    writeI2CData(sendBuffer, 2);
    sendBuffer[0] = 0x1A;
    sendBuffer[1] = 0x0A;
    writeI2CData(sendBuffer, 2);
    sendBuffer[0] = 0x1B;
    sendBuffer[1] = 0x0A;
    writeI2CData(sendBuffer, 2);
    sendBuffer[0] = 0x1F;
    sendBuffer[1] = 0x0F;
    writeI2CData(sendBuffer, 2);
    sendBuffer[0] = 0x02;
    sendBuffer[1] = 0x9E;
    writeI2CData(sendBuffer, 2);
}


void initTimer()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 7200;
    //timer.TIM_Period = p;
    TIM_TimeBaseInit(TIM2, &timer);
}

void TIM2_IRQHandler()
{		
		
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		
    state ^= 0x01;
		
}
void notefrec(int period)
{		//delay(0x000F);
		
			timer.TIM_Period = period;
			TIM_TimeBaseInit(TIM2, &timer);
		
	
	
}

void init_EXTI()                                          
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);   
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);  
     GPIO_InitTypeDef GPIO_InitStructure;                                                       
 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;               
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;            
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;        
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;          
    GPIO_Init(GPIOA, &GPIO_InitStructure);          
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
		exti.EXTI_Line = EXTI_Line0;                            
    exti.EXTI_Mode = EXTI_Mode_Interrupt;                   
                                                            
    exti.EXTI_Trigger = EXTI_Trigger_Rising;                
    exti.EXTI_LineCmd = ENABLE;                             
    EXTI_Init(&exti);                                       
 
    nvic.NVIC_IRQChannel = EXTI0_IRQn;                      
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);   
    nvic.NVIC_IRQChannelPreemptionPriority = 0;            
    nvic.NVIC_IRQChannelSubPriority = 0;                    
    nvic.NVIC_IRQChannelCmd = ENABLE;                       
    NVIC_Init(&nvic);                                       
}
	
	
	
	
void EXTI0_IRQHandler(void) {
	EXTI_ClearITPendingBit(EXTI_Line0); //Очищаем флаг прерывания
 		
		int a = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
		int b=0;
		for(a;a==1;b++)
	{
		a = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
		
		if(b==200){ 
			
			GPIO_ToggleBits(GPIOD,GPIO_Pin_14);
			
			
			if(p<11) notefrec(++p);
			else p=1;
				
	
	
		}
		
		
	}

}


/***************************************************************************************/
int main(void)
{
    
    __enable_irq();
   
    initGPIO();
    initTimer();
    initI2C();
    initI2S();
    initCS32L22();
   
    I2S_Cmd(SPI3, ENABLE);
    
			TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM2, ENABLE);
		
    NVIC_EnableIRQ(TIM2_IRQn);
		init_EXTI() ;
    while(1)
    {	
			
			
		
		
        
        if (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE)) // можно ли передавать данные
        {
            if (state == 0x00)
            {
                
                SPI_I2S_SendData(SPI3, 0x00);
            }
            else
            {
                
                SPI_I2S_SendData(SPI3, 0xFF);
            }
        }
				
				
    
	}
}
