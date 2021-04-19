#ifndef USART_PROC_H
#define USART_PROC_H

#include "usart.h"
#include "string.h"
#include "math.h"

class USARTProc{
public:
	USARTProc(){
		m_usart=USART1;
		m_bstate=false;
		memset(receive_data, '\0', RECIVE_DATA_MAX_LEN);
	}
	USARTProc(USART_TypeDef* usart){
		m_usart=usart;
		m_bstate=false;
		memset(receive_data, '\0', RECIVE_DATA_MAX_LEN);
	}
	
	int GetData() const{
		return m_data;
	}
	bool GetState() const{
		return m_bstate;
	}
	const char* GetReceiveStr(){
		return receive_data;
	}
	USART_TypeDef* GetUsart(){
		return m_usart;
	}
	
	void ReceiveNumber(){
    while(1){
			m_bstate=false;
			if(USART_RX_STA & 0x8000){
				int r, data; 						
				u16 len;
				bool bsign=true; 				// true --> +, false --> -
				len = USART_RX_STA & 0x3fff;
				r = len;
        m_data = 0;
				memset(receive_data, '\0', RECIVE_DATA_MAX_LEN);
				
        for(int t = 0; t < len; t++){
					USART_SendData(m_usart, USART_RX_BUF[t]);
					while(USART_GetFlagStatus(m_usart, USART_FLAG_TC) != SET){}
						receive_data[t] = USART_RX_BUF[t];
						if(receive_data[t]=='-'){
							bsign = false;
							data = 0;
						//}else if(isdigit(receive_data[t])){
						}else{
							data = (int)receive_data[t] - 48;
						}
						r = r - 1;
						m_data = m_data + data * (pow(10.0, r));
				}
				
				if(!bsign)
					m_data = -m_data;
				
				USART_RX_STA = 0;
				m_bstate=true;
				
				#ifdef DEBUG_INFO
					USARTx_printf(m_usart, "\r\n");
					USARTx_printf(m_usart, "data=%d, bsign=%d, bstate=%d \r\n", m_data, bsign, m_bstate);
				#endif
				
				break;
			}
		}
	}
private:
	enum {RECIVE_DATA_MAX_LEN=60};
private:
	char receive_data[RECIVE_DATA_MAX_LEN];
	int m_data; 			
	bool m_bstate; 		
private:
	USART_TypeDef * m_usart;
};

#endif

