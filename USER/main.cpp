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
不能在RS485 send数据的过程中及发送后获取响应之前利用user_main_****打印数据
采用的解决方法：在getServoResponse中接收到电机响应后打印曾发送的数据
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
	
	delay_init();	    	 			//延时函数初始化	  
	NVIC_Configuration(); 	 	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(9600);	 				//串口初始化为9600
// 	LED_Init();			     	//LED端口初始化
//	KEY_Init();          	//初始化与按键连接的硬件接口  
	
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
