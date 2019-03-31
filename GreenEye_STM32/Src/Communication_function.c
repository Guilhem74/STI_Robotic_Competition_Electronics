#include "Communication_function.h"

void Transmit_UART(uint8_t * T) 
{ 
		strcpy((char*)BUFFER_TX[Indice_Stop_TX],(char*)T);
		Indice_Stop_TX=(Indice_Stop_TX+1)%SIZE_BUFFER;
		if(huart2.gState == HAL_UART_STATE_READY)
				HAL_UART_Transmit_DMA(&huart2,BUFFER_TX[Indice_Start_TX],strlen((char*)BUFFER_TX[Indice_Start_TX]));

	
} 
