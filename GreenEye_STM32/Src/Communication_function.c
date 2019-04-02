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
	 String_Analysis(UART_RX_Analyse);
		
}
void String_Analysis(uint8_t* Input)
{
	int char_counter = 0;
	char letter;
	float value;
	float Table_values[20];
	uint8_t Table_Letter[20];
		int Parameters_N=0;
	while (next_statement(&letter, &value, (char *) Input, &char_counter))
	{
		 Table_values[Parameters_N]= (float)value;
		 Table_Letter[Parameters_N]= (uint8_t)letter;
		 Parameters_N++;	
	}
	if(Parameters_N==0)
		return;
	uint8_t Answer[40];
	switch(Table_Letter[0])
	{
		case 'G':
				switch((int) Table_values[0])
				{
					case 92:
						COMMAND_G92(Table_Letter+1,Table_values+1,Parameters_N-1);
					break;
				}
			break;
		default :
				sprintf((char*)Answer,"Unrecognized %s\r\n",UART_RX_Analyse);
				Transmit_UART(Answer);
			break;
	}
}

void COMMAND_G92(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters )
{
	if(UPDATE_POS_PARAMETERS==0)
	{
		ANGLE_POS_RAD_CACHE=ANGLE_POS_RAD;
		X_POS_MM_CACHE=X_POS_MM;
		Y_POS_MM_CACHE=Y_POS_MM;
	}
	int j=0;
	while(j<Number_Parameters)
	{
			switch(Table_Parameters_Letter[j])
			{
				case 'X':
					X_POS_MM_CACHE=Table_Parameters_Number[j];				
					break;
				case 'Y':
					Y_POS_MM_CACHE=Table_Parameters_Number[j];
					break;
				case 'A':
					ANGLE_POS_RAD_CACHE=Table_Parameters_Number[j];
					break;
			}
			j++;
	}
	uint8_t Answer[40];
	sprintf((char*)Answer,"OK: X=%0.2f Y=%0.2f A=%0.2f \r\n",X_POS_MM_CACHE,Y_POS_MM_CACHE,ANGLE_POS_RAD_CACHE);
	Transmit_UART(Answer);
	UPDATE_POS_PARAMETERS=1;
}
//https://github.com/x893/CNC-STM32/blob/master/src/application/gcode.c
int next_statement(char *letter, float *double_ptr, char *line, int *char_counter)
{
	while (line[*char_counter] == ' ') (*char_counter)++;

	if (line[*char_counter] == 0 || line[*char_counter] == ';' ||
		line[*char_counter] == '\n' || line[*char_counter] == '\r') return 0;
	*letter = line[*char_counter];
	if ((*letter < 'A') || (*letter > 'Z'))
	{
		return 0;
	}
	(*char_counter)++;
	return read_double(line, char_counter, double_ptr);
}
int read_double(char *line, int *char_counter, float *double_ptr)
{
	char *start = line + *char_counter;
	char *end;

	*double_ptr = strtod_M(start, &end);
	if (end == start)
	{
		return 0;
	}
	*char_counter = (int)(end - line);
	return 1;
}
float strtod_M(const char *str, char **endptr)
{//Kind of atoi function
	float number = 0.0;
	float div = 0.0;
	int8_t negative = 0;
	int8_t plus = 0;
	int8_t skip = 1;
	char c;
	while ((c = *str) != 0)
	{
		if (c == '+')
		{
			if (skip && !plus)
			{
				plus = 1;
				skip = 0;
			}
			else
				break;
		}
		else if (skip && !negative && c == '-')
		{
			if (skip && !negative)
			{
				negative = 1;
				skip = 0;
			}
			else
				break;
		}
		else if (c == '.')
		{
			if (div == 0.0f)
				div = 1.0f;
			else
				break;
		}
		else if (c >= '0' && c <= '9')
		{
			skip = 0;
			if (div == 0.0f)
				number = number * 10.0f + (float)(c - '0');
			else
			{
				div *= 10.0f;
				number += ((float)(c - '0') / div);
			}
		}
		else if (!skip)
		{
			break;
		}
		str++;
	}

	if (negative) number = -number;
	if (endptr != NULL) *endptr = (char *)str;
	return number;
}
