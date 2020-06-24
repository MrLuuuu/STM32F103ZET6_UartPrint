//  author: luyouqi
//    data:2020.6.24
// version:1.0
//    mail:real_luyouqi@163.com
#include <stdio.h>
#include "stm32f10x.h"
#include "led.h"
#include "delay.h"
#include "key.h"
#include "timer.h"
#include "beep.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"

unsigned int ADC_ConvertedValue=0;

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
void ADInit(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_USART1,(FunctionalState)(ENABLE));
	GPIO_InitTypeDef ADC_C1_GPIO;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,(FunctionalState)(ENABLE));
	ADC_C1_GPIO.GPIO_Mode=GPIO_Mode_AIN;
	ADC_C1_GPIO.GPIO_Pin=GPIO_Pin_1;
	GPIO_Init(GPIOC,&ADC_C1_GPIO);

	DMA_InitTypeDef ADC_DMA;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,(FunctionalState)(ENABLE));
	DMA_DeInit(DMA1_Channel1);
	ADC_DMA.DMA_PeripheralBaseAddr=(u32)&(ADC1->DR);
	ADC_DMA.DMA_MemoryBaseAddr=(u32)&ADC_ConvertedValue;
	ADC_DMA.DMA_DIR=DMA_DIR_PeripheralSRC;
	ADC_DMA.DMA_BufferSize=1;
	ADC_DMA.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	ADC_DMA.DMA_MemoryInc=DMA_MemoryInc_Disable;
	ADC_DMA.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	ADC_DMA.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;
	ADC_DMA.DMA_Mode=DMA_Mode_Circular;
	ADC_DMA.DMA_Priority=DMA_Priority_High;
	ADC_DMA.DMA_M2M=DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1,&ADC_DMA);
	DMA_Cmd(DMA1_Channel1,(FunctionalState)(ENABLE));

	ADC_InitTypeDef ADC1_Init;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,(FunctionalState)(ENABLE));
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);
	ADC1_Init.ADC_Mode=ADC_Mode_Independent;
	ADC1_Init.ADC_ScanConvMode=DISABLE;
	ADC1_Init.ADC_ContinuousConvMode=ENABLE;
	ADC1_Init.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;
	ADC1_Init.ADC_DataAlign=ADC_DataAlign_Right;
	ADC1_Init.ADC_NbrOfChannel=1;
	ADC_Init(ADC1,&ADC1_Init);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_11,1,ADC_SampleTime_55Cycles5);
	ADC_DMACmd(ADC1,ENABLE);
	ADC_Cmd(ADC1,ENABLE);
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);

}	


int main(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	 
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
 	  USART_DeInit(USART1);  //��λ����1
	  //USART1_TX   PA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure); //��ʼ��PA9
   
    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);  //��ʼ��PA10     
  /* USARTx configured as follow:
        - BaudRate = 9600 baud  ������
        - Word Length = 8 Bits  ���ݳ���
        - One Stop Bit          ֹͣλ
        - No parity             У�鷽ʽ
        - Hardware flow control disabled (RTS and CTS signals) Ӳ��������
        - Receive and transmit enabled                         ʹ�ܷ��ͺͽ���
  */
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART1, &USART_InitStructure);
  USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ��� 

	LED_Init();//LED��ʼ��
  KEY_Init();//������ʼ��
  SysTick_Init();//��ʱ��ʼ��
	BEEP_Init();   //��������ʼ��
	ADInit();
  while (1)
  {
		char sendbuf[7];
		float Volte;
		Volte=(float)ADC_ConvertedValue/4096*3300;//???mv
		sprintf(sendbuf,"%f\n",Volte);
		
		for(unsigned char count=0;count<strlen(sendbuf)-2;count++)
		{
			sendbuf[strlen(sendbuf)-count-1]=sendbuf[strlen(sendbuf)-count-3];
		}
		sendbuf[0]='m';
		sendbuf[1]='V';
	printf("%s",sendbuf);
		//ʹ��printf����ѭ�����͹̶���Ϣ
//  printf("\n\rUSART Printf Example: ���·�������ĪM3S�����崮�ڲ��Գ���\r");
	Delay_ms(500);		  
  LED2_REV;	
  }
}


PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* ѭ���ȴ�ֱ�����ͽ���*/
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

/*----------------------�·��� ������̳��www.doflye.net--------------------------*/
