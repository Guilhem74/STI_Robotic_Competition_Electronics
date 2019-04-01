#include "Communication_function.h"

void Transmit_UART(uint8_t * T) 
{ 
		strcpy((char*)BUFFER_TX[Indice_Stop_TX],(char*)T);
		Indice_Stop_TX=(Indice_Stop_TX+1)%SIZE_BUFFER;
		if(huart2.gState == HAL_UART_STATE_READY)
				HAL_UART_Transmit_DMA(&huart2,BUFFER_TX[Indice_Start_TX],strlen((char*)BUFFER_TX[Indice_Start_TX]));

	
} 
void Analyse_RX_Buffer()
{
	if(Indice_Start_RX==Indice_Stop_RX)
		return;
	//New message to Analyse
	strcpy((char *)UART_RX_Analyse,(char *) BUFFER_RX[Indice_Start_RX]);
	Indice_Start_RX=(Indice_Start_RX+1)%SIZE_BUFFER;
	uint8_t Answer[64]="";
	sprintf((char*)Answer,"%s %s","OK: ",(char*) UART_RX_Analyse);
	int16_t Parameter1_Number=0;
	uint8_t Parameter1_Letter=0;
	sscanf((char *)UART_RX_Analyse,"%c%d %s",&Parameter1_Letter,(int*) &Parameter1_Number,UART_RX_Analyse);
	switch(Parameter1_Letter)
	{
		case 'G':		Transmit_UART(Answer);
			break;
		case 'M':   Transmit_UART(Answer);
			break;
		case 'O':   Transmit_UART(Answer);
			break;
		default:
			break;
	}
		
}
