#include "Communication_function.h"

void Transmit_UART(uint8_t * T) 
{ 
		strcpy((char*)BUFFER_TX[Indice_Stop_TX],(char*)T);
		strcpy((char*)BUFFER_TX_UART2[Indice_Stop_TX_UART2],(char*)T);

		Indice_Stop_TX=(Indice_Stop_TX+1)%SIZE_BUFFER;
		Indice_Stop_TX_UART2=(Indice_Stop_TX_UART2+1)%SIZE_BUFFER;

		if(huart6.gState == HAL_UART_STATE_READY)
				HAL_UART_Transmit_DMA(&huart6,BUFFER_TX[Indice_Start_TX],strlen((char*)BUFFER_TX[Indice_Start_TX]));
		if(huart2.gState == HAL_UART_STATE_READY)
				HAL_UART_Transmit_DMA(&huart2,BUFFER_TX_UART2[Indice_Start_TX_UART2],strlen((char*)BUFFER_TX_UART2[Indice_Start_TX_UART2]));

	
} 
void Analyse_RX_Buffer()
{
	if(Indice_Start_RX!=Indice_Stop_RX)
	{//New message to Analyse
			strcpy((char *)UART_RX_Analyse,(char *) BUFFER_RX[Indice_Start_RX]);
			Indice_Start_RX=(Indice_Start_RX+1)%SIZE_BUFFER;
	}else if(Indice_Start_RX_UART2!=Indice_Stop_RX_UART2)
	{
		strcpy((char *)UART_RX_Analyse,(char *) BUFFER_RX_UART2[Indice_Start_RX_UART2]);
		Indice_Start_RX_UART2=(Indice_Start_RX_UART2+1)%SIZE_BUFFER;
	}
	else
		return;
	//strcpy((char *) BUFFER_RX[Indice_Start_RX]," ");
	uint8_t Answer[64]="";
	sprintf((char*)Answer,"%s %s","OK: ",(char*) UART_RX_Analyse);
	 const char s[2] = " ";
   char *token;
   /* get the first token */
   token = strtok((char *)UART_RX_Analyse, s);
   while( token != NULL ) {
      Transmit_UART( (uint8_t*)token );
      Transmit_UART( "\n" );
      token = strtok(NULL, s);
   }
	 int Parameter1_Letter,Parameter1_Number;
	switch(Parameter1_Letter)
	{
		case 'G':		
			switch(Parameter1_Number)
			{
				case 92:
					if(UPDATE_POS_PARAMETERS==0)
					{//We just received an update, pos hsan't been updated
						ANGLE_POS_RAD_CACHE=ANGLE_POS_RAD;
						X_POS_MM_CACHE=X_POS_MM;
						Y_POS_MM_CACHE=Y_POS_MM;
					}
					else
						UPDATE_POS_PARAMETERS=0;

					break;
				}
			break;
		case 'M':   Transmit_UART(Answer);
			break;
		case 'O':   Transmit_UART(Answer);
			break;
		default:
			break;
	}
		
}
