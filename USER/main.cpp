#include "math.h"
#include "string.h"
#include "ctype.h"
#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "stdio.h"
#include "mLog.h"
#include "servo.h"
#include "usartproc.h"

// debug info
/*
������RS485 send���ݵĹ����м����ͺ��ȡ��Ӧ֮ǰ����user_main_****��ӡ����
���õĽ����������getServoResponse�н��յ������Ӧ���ӡ�����͵�����
*/

// global
enum{ STEP=100 };


int GetNexAng(int curang){
	if(curang+STEP>=0 && curang+STEP<=0xfff){
		return curang+STEP;
	}else if(curang+STEP<0){
		return 0;
	}else{
		return curang-STEP;
	}
}


int main(void)
{		
	uint8_t servoId=1;
	static bool bfirst=true;
	USARTProc usartproc;
	
	delay_init();	    	 			//��ʱ������ʼ��	  
	NVIC_Configuration(); 	 	//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(9600);	 				//���ڳ�ʼ��Ϊ9600
// 	LED_Init();			     	//LED�˿ڳ�ʼ��
//	KEY_Init();          	//��ʼ���밴�����ӵ�Ӳ���ӿ�  
	
	Servo servo;
	servo.OpenPort();
	
 	while(1)
	{
		usartproc.ReceiveNumber();
		if(usartproc.GetState()){
			//USARTx_printf(usartproc.GetUsart(), "data=%d, str=%s \r\n", usartproc.GetData(), usartproc.GetReceiveStr());
			delay_ms(1000);
			
			if(bfirst){
				bfirst=false;
				bool bflag=servo.pingServo(1);
				if(bflag){
					int tem=servo.getTemperature(servoId);
					int angle=servo.getServoPresentPosition(servoId, &angle);
					delay_ms(1000);
					user_main_info("angle: %d", angle);
					user_main_info("tem: %d", tem);
					
					//servo.torqueOn(servoId);
					servo.setOperatingMode(servoId, TORQUE_CONTROL_MODE);
					servo.setServoGoalTorque(servoId, 4);
				}else{
					user_main_error("cannot connect the servo!!");
				}
			}
		}
//			int curang;
//			servo.getServoAngle(servoId, &curang);
//			delay_ms(1000);
//			servo.setServoAngle(servoId, GetNexAng(curang));
//			delay_ms(1000);
			
//			servo.ledOn(servoId);
//			delay_ms(1000);
//			servo.ledOff(servoId);
//			delay_ms(1000);
  }
}
