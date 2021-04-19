#include "sys.h"
#include "usart.h"	 
#include "stdarg.h"	 
#include "stdio.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos ʹ��	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/8/18
//�汾��V1.5
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
//V1.5�޸�˵��
//1,�����˶�UCOSII��֧��
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
////�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
//#if 1
//#pragma import(__use_no_semihosting)             
////��׼����Ҫ��֧�ֺ���                 
//struct __FILE 
//{ 
//	int handle; 

//}; 

//FILE   __stdout;       
////����_sys_exit()�Ա���ʹ�ð�����ģʽ    
//_sys_exit(int x) 
//{ 
//	x = x; 
//	
//} 
////�ض���fputc���� 
//int std :: fputc(int ch, FILE *f)
//{      
//	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); 
//    USART_SendData(USART1,(uint8_t)ch);   
//	return ch;
//}
//#endif 

/*ʹ��microLib�ķ���*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
 
 /*
*������ :itoa
*����   :����������ת��Ϊ�ַ���
*����   :-radix =10 ��ʾ10���ƣ��������Ϊ0
					-value Ҫת����������
					-buf ת������ַ���
					-radix =10
*���		:��
*����		:��
*����		:�� USARTx_printf()����
*/
static char *itoa(int value, char *string, int radix)
{
	int  i,d;
	int  flag=0;
	char  *ptr=string;
//��������10������
	if(radix!=10)
	{
		*ptr=0;
		return string;
	}
	if(!value)   //��0  ��ʾ�ַ�0
	{
		*ptr++=0x30;
		*ptr	=0;
		return string;	
	}
//��ӡ�-��
	if(value<0)
	{
			*ptr++='-';
			value *=-1; //value ȡ��
	}
	for(i=10000;i>0;i/=10)
	{
		d=value/i;
		if(d|| flag)
		{
			*ptr++=(char)(d+0x30);
			value-=(d*i);
			flag=1;		
		}	
	}
	//��ֵ
	*ptr=0;
	return string;
}
/*
*������ :USARTx_printf
*����   :��ʽ�������������printf,��δ�õ�c��
*����   :-USARTx ����ͨ��
					-Data ��������ָ��
					-...��������
*���		:��
*����		:��
*����		:�ⲿ����
					����Ӧ�ã�USARTx_printf( USART1,"this is a demo\r\n");
										USARTx_printf( USART1,"%d\r\n", i);
*/
void USARTx_printf(USART_TypeDef* USARTx,const char *Data,...)
{
	const char *s;
	int d;
	char buf[16];
	va_list ap;
	va_start(ap, Data);
	while(*Data !=0)
	{
		if(*Data ==0x5c) //'/'
		{
			switch( *++Data )
			{
				case 'r':
							USART_SendData(USARTx, 0x0d);
							Data++;
							break;
				case 'n':
							USART_SendData(USARTx, 0x0a);
							Data++;
							break;
				default:
							Data++;
							break;
			}
		}
		else if(*Data =='%')
		{
			switch( *++Data )
			{
				case 's':
							s=va_arg(ap, const char*);
							for(;*s ; s++)
							{
								USART_SendData(USARTx, *s);
								while( USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
							}
							Data++;
							break;
				case 'd':
							d=va_arg(ap, int);
							itoa(d,buf,10);
							for(s=buf;*s ; s++)
							{
								USART_SendData(USARTx, *s);
								while( USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
							}
							Data++;
							break;
				default:
							Data++;
							break;
			}
		
		} //end of else if
		else USART_SendData(USARTx, *Data++);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
	}
}
 
 
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	  

//��ʼ��IO ����1 
//bound:������
void uart_init(u32 bound){
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
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


   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

    USART_Init(USART1, &USART_InitStructure); //��ʼ������
#if EN_USART1_RX		  //���ʹ���˽���  
   //Usart1 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
   
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�����ж�
#endif
    USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ��� 

}

#ifdef __cplusplus
extern "C" {
#endif

void USART1_IRQHandler(void)                	//����1�жϷ������
//void USART1_IRQHandler(void) 
	{
	u8 Res;
#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
		{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������
		
		if((USART_RX_STA&0x8000)==0)//����δ���
			{
			if(USART_RX_STA&0x4000)//���յ���0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else USART_RX_STA|=0x8000;	//��������� 
				}
			else //��û�յ�0X0D
				{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
					}		 
				}
			}   		 
     } 
#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
	OSIntExit();  											 
#endif
} 
	
#ifdef __cplusplus
}
#endif

#endif	

