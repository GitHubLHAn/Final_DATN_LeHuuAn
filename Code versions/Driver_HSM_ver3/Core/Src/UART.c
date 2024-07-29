#include "main.h"
#include "stm32f4xx_hal_tim.h"
#include "stdio.h"

extern UART_HandleTypeDef huart3;
extern UART uart_var;
extern CONTROL_OBJECT control_var;
extern ENCODER_OBJECT encoder_var;
extern StateMachine vStateMa;
extern RAMP ramp_rpm_var;

uint8_t MSG[30] = {'\0'};

/*_________________________________________________________________________________________________________________________*/
	/*Initiate data*/
	void UART_Init(UART *pUART){
		for(int i=0; i<2000; i++)
		{			
			pUART->data_1[i] = 0;
			pUART->data_2[i] = 0;
			pUART->data_3[i] = 0;
			pUART->data_4[i] = 0;
		}
		pUART->saving = 0;
		pUART->save_done = 0;
		pUART->sending = 0;
		pUART->cnt = 0;
		pUART->num_data = 0;
		
		HAL_GPIO_WritePin(RS485_EN3_GPIO_Port, RS485_EN3_Pin, GPIO_PIN_SET);		// Set the transmit mode for IC RS485	
	}	
/*_________________________________________________________________________________________________________________________*/
	void SAVE_DATA(mDATA get_mode)
	{
		if(get_mode == GET_C_DATA && uart_var.saving == 1)		// get current value
		{
			uart_var.num_data = 1000;
			if(uart_var.cnt < uart_var.num_data)
			{
				uart_var.data_1[uart_var.cnt] = (int)(1000*control_var.Iq_sp); 		//(int)(1000*control_var.Ia_fb);
				uart_var.data_2[uart_var.cnt] = (int)(1000*control_var.Iq_fb); 		//(int)(1000*control_var.Ib_fb);
				uart_var.data_3[uart_var.cnt] = (int)(1000*control_var.Id_sp);
				uart_var.data_4[uart_var.cnt] = (int)(1000*control_var.Id_fb);
				uart_var.cnt++;
			}
			else uart_var.save_done = 1;
		}
		/*-----------------------------*/
		if(get_mode == GET_S_DATA && uart_var.saving == 1)		// get speed value
		{		
			uart_var.num_data = 1000;
			if(uart_var.cnt < uart_var.num_data)
			{
				uart_var.data_1[uart_var.cnt] = (int)(100*control_var.rpm_sp);
				uart_var.data_2[uart_var.cnt] = (int)(100*encoder_var.rpm_fb);
				uart_var.data_3[uart_var.cnt] = (int)(100*control_var.theta_e);
				uart_var.data_4[uart_var.cnt] = 0;
				uart_var.cnt++;
			}
			else uart_var.save_done = 1;
		}
		/*-----------------------------*/
		if(get_mode == GET_P_DATA && uart_var.saving == 1)
		{		// get position value
		
		}
		/*-----------------------------*/
		if(uart_var.save_done == 1 && uart_var.saving == 1)
		{
			uart_var.saving = 0;
			uart_var.cnt = 0;
			uart_var.sending = 1;
			Motor_Disable();
		}
	}
/*_________________________________________________________________________________________________________________________*/
	/*Send_data*/
	void SEND_DATA(void)
	{
		if(uart_var.sending ==1)
		{
			if(uart_var.cnt < uart_var.num_data)
			{
				sprintf((char*)MSG,"\n%d %d %d %d %d", uart_var.data_1[uart_var.cnt],uart_var.data_2[uart_var.cnt],
																							uart_var.data_3[uart_var.cnt],uart_var.data_4[uart_var.cnt],uart_var.cnt);
				HAL_UART_Transmit(&huart3, (uint8_t *)MSG, sizeof(MSG), 100);		
				uart_var.cnt++;
			}
			else
			{
				uart_var.sending = 0;
				HAL_UART_Transmit(&huart3, (uint8_t *)"\nDone!!!", sizeof("\nDone!!!"), 30);	
			}				
		}
	}
		
			