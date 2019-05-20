#include "Communication_function.h"
#include <string.h>
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
					case 0:
						COMMAND_G0(Table_Letter+1,Table_values+1,Parameters_N-1);//Set destination
					break;
					case 1:
						COMMAND_G1(Table_Letter+1,Table_values+1,Parameters_N-1);//Set destination
					break;
					case 2:
						COMMAND_G2(Table_Letter+1,Table_values+1,Parameters_N-1);//Set PWM
					break;
					case 92:
						COMMAND_G92(Table_Letter+1,Table_values+1,Parameters_N-1);//Set actual position
					break;
				}
			break;
		case 'M':
				switch((int) Table_values[0])
				{
					case 3:
						COMMAND_M3(Table_Letter+1,Table_values+1,Parameters_N-1);//Enable position control
						break;
					case 92:
						COMMAND_M92(Table_Letter+1,Table_values+1,Parameters_N-1);//Robot parameters, wheel size...
						break;
					case 112:
						COMMAND_M112(Table_Letter+1,Table_values+1,Parameters_N-1);//Emergency Stop
						break;
					case 135:
						COMMAND_M135(Table_Letter+1,Table_values+1,Parameters_N-1);//Set loop interval (Doesn't modify the periodicity but the value used in the algorithm)
						break;
					case 201:
						COMMAND_M201(Table_Letter+1,Table_values+1,Parameters_N-1);//Set acceleration, speed and braking parameters
						break;
					case 202:
						COMMAND_M202(Table_Letter+1,Table_values+1,Parameters_N-1);//Set final bool diameter
						break;
					case 301:
						COMMAND_M301(Table_Letter+1,Table_values+1,Parameters_N-1);//Set PID parameters
						break;
					
					
				}
			break;
		case 'O':
				switch((int) Table_values[0])
				{	
				case 1:
					COMMAND_O1(Table_Letter+1,Table_values+1,Parameters_N-1);//Set PWM
					break;
				}
				break;
		default :
				sprintf((char*)Answer,"Unrecognized %s\r\n",UART_RX_Analyse);
				Transmit_UART(Answer);
			break;
	}
}
void COMMAND_O1(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters )
{//O1 H0/1 Sx
	int j=0,Type=-1,PWM_CACHE=0;
	while(j<Number_Parameters)
	{
			switch(Table_Parameters_Letter[j])
			{
				case 'H':
					Type=Table_Parameters_Number[j];				
					break;
				case 'S':
					PWM_CACHE=Table_Parameters_Number[j];
					break;
			}
			j++;
	}
	

	uint8_t Answer[40];
	if(Type==1)
	{
		CONTROL_ENABLED=0;
		TIM2->CCR1=PWM_CACHE;
		sprintf((char*)Answer,"OK: PWM H1 S%d \r\n",PWM_CACHE);
	}
	else if(Type==0)
	{//Right
		CONTROL_ENABLED=0;
		TIM2->CCR2=PWM_CACHE;
		sprintf((char*)Answer,"OK: PWM H0 S%d \r\n",PWM_CACHE);
	}
	else
	{
		sprintf((char*)Answer,"KO: Select a tool(H0 or H1)\r\n ");
	}
	Transmit_UART(Answer);
}
void COMMAND_G0(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters )
{//G0 Xx Yx Ax
	if(UPDATE_DEST_PARAMETERS==0)
	{
		ANGLE_DES_RAD_CACHE=ANGLE_DES_RAD;
		X_DES_MM_CACHE=X_DES_MM;
		Y_DES_MM_CACHE=Y_DES_MM;
	}
	int j=0;
	while(j<Number_Parameters)
	{
			switch(Table_Parameters_Letter[j])
			{
				case 'X':
					X_DES_MM_CACHE=Table_Parameters_Number[j];				
					break;
				case 'Y':
					Y_DES_MM_CACHE=Table_Parameters_Number[j];
					break;
				case 'A':
					ANGLE_DES_RAD_CACHE=Table_Parameters_Number[j]*PI/180;
					break;
			}
			j++;
	}
	uint8_t Answer[40];
	sprintf((char*)Answer,"OK: X=%0.2f Y=%0.2f A=%0.2f \r\n",X_DES_MM_CACHE,Y_DES_MM_CACHE,ANGLE_DES_RAD_CACHE);
	Transmit_UART(Answer);
	UPDATE_DEST_PARAMETERS=1;
}
void COMMAND_G1(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters )
{//G1 Rx Ly

	int j=0;
	while(j<Number_Parameters)
	{
			switch(Table_Parameters_Letter[j])
			{
				case 'R':
					R_SPEED_TARGET=Table_Parameters_Number[j];				
					break;
				case 'L':
					L_SPEED_TARGET=Table_Parameters_Number[j];
					break;
			}
			j++;
	}
	uint8_t Answer[40];
	sprintf((char*)Answer,"OK: D=%0.2f G=%0.2f \r\n",R_SPEED_TARGET,L_SPEED_TARGET);
	Transmit_UART(Answer);
	UPDATE_DEST_PARAMETERS=1;
}
void COMMAND_G2(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters )
{//G1 Rx Ly

	int j=0;
	while(j<Number_Parameters)
	{
			switch(Table_Parameters_Letter[j])
			{
				case 'R':
					R_SPEED_TARGET=Table_Parameters_Number[j];				
					break;
				case 'L':
					L_SPEED_TARGET=Table_Parameters_Number[j];
					break;
			}
			j++;
	}
	uint8_t Answer[40];
	sprintf((char*)Answer,"OK: D=%0.2f G=%0.2f \r\n",R_SPEED_TARGET,L_SPEED_TARGET);
	Transmit_UART(Answer);
	UPDATE_DEST_PARAMETERS=1;
}
void COMMAND_G92(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters )
{//G92 Xx Yx Ax
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
void COMMAND_M3(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters )
{// M3 
	CONTROL_ENABLED=1;
	uint8_t Answer[40];
	sprintf((char*)Answer,"OK: Control Enabled \r\n");
	Transmit_UART(Answer);
}
void COMMAND_M92(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters )
{// M92 Dx Ax
	if(UPDATE_POS_PARAMETERS==0)
	{
	 TICS_2_MM_CACHE=TICS_2_MM;
   SPACING_WHEELS_CACHE=SPACING_WHEELS;
	}
	
	int j=0;
	while(j<Number_Parameters)
	{
			switch(Table_Parameters_Letter[j])
			{
				case 'D':
					TICS_2_MM_CACHE=Table_Parameters_Number[j];
					break;
				case 'A':
					SPACING_WHEELS_CACHE=Table_Parameters_Number[j];	
					break;
			}
			j++;
	}
	uint8_t Answer[40];
	sprintf((char*)Answer,"OK: M92 Tics/MM S=%0.2f SpacingWheels=%0.2f\r\n",TICS_2_MM_CACHE,SPACING_WHEELS_CACHE);
	Transmit_UART(Answer);
	UPDATE_POS_PARAMETERS=1;
}
void COMMAND_M112(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters )
{// M112
	CONTROL_ENABLED=0;
	TIM2->CCR1=0;//Set PWM to 0
	TIM2->CCR2=0;//Set PWM to 0
	uint8_t Answer[40];
	sprintf((char*)Answer,"OK: Emergency Stop \r\n");
	Transmit_UART(Answer);
}
void COMMAND_M135(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters )
{// M135 Sx
	if(Table_Parameters_Letter[0]=='S')
		LOOP_CONTROL_TIMING_HZ=Table_Parameters_Number[0];
	uint8_t Answer[40];
	sprintf((char*)Answer,"OK: Timing loop : %0.2f \r\n",LOOP_CONTROL_TIMING_HZ);
	Transmit_UART(Answer);
}
void COMMAND_M201(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters )
{// M201 H0/1 Sx Ax Bx 
	if(UPDATE_CONTROL_PARAMETERS==0)
	{
		SPEED_MAX_DISTANCE_MM_S_CACHE=SPEED_MAX_DISTANCE_MM_S;
		ACCELERATION_MAX_DISTANCE_MM_S2_CACHE=ACCELERATION_MAX_DISTANCE_MM_S2;
		BRAKING_MAX_DISTANCE_MM_S2_CACHE=BRAKING_MAX_DISTANCE_MM_S2;
		
		SPEED_MAX_ANGLE_MM_S_CACHE=SPEED_MAX_ANGLE_MM_S;
		ACCELERATION_MAX_ANGLE_MM_S2_CACHE=ACCELERATION_MAX_ANGLE_MM_S2;
		BRAKING_MAX_ANGLE_MM_S2_CACHE=BRAKING_MAX_ANGLE_MM_S2;
	}
	
	int j=0;
	int Type=-1;;
	while(j<Number_Parameters)
	{
			switch(Table_Parameters_Letter[j])
			{
				case 'H':
					Type=Table_Parameters_Number[j];
					break;
				case 'A':
					if(Type==0)
							ACCELERATION_MAX_DISTANCE_MM_S2_CACHE=Table_Parameters_Number[j];
					else if(Type==1)
							ACCELERATION_MAX_ANGLE_MM_S2_CACHE=Table_Parameters_Number[j];			
					break;
				case 'B':
					if(Type==0)
							BRAKING_MAX_DISTANCE_MM_S2_CACHE=Table_Parameters_Number[j];
					else if(Type==1)
							BRAKING_MAX_ANGLE_MM_S2_CACHE=Table_Parameters_Number[j];	
					break;
				case 'S':
					if(Type==0)
							SPEED_MAX_DISTANCE_MM_S_CACHE=Table_Parameters_Number[j];
					else if(Type==1)
							SPEED_MAX_ANGLE_MM_S_CACHE=Table_Parameters_Number[j];	
					break;
			}
			j++;
	}
	uint8_t Answer[40];
	if(Type==0)//Distance parameters
		sprintf((char*)Answer,"OK: D_Profil S=%d A=%d B=%d \r\n",SPEED_MAX_DISTANCE_MM_S_CACHE,ACCELERATION_MAX_DISTANCE_MM_S2_CACHE,BRAKING_MAX_DISTANCE_MM_S2_CACHE);
  else if(Type==1)//Angle parameters
		sprintf((char*)Answer,"OK: A_Profil S=%d A=%d B=%d \r\n",SPEED_MAX_ANGLE_MM_S_CACHE,ACCELERATION_MAX_ANGLE_MM_S2_CACHE,BRAKING_MAX_ANGLE_MM_S2_CACHE);
	else
		sprintf((char*)Answer,"KO: No Profil selected (H0 or H1) \r\n");
	Transmit_UART(Answer);
	UPDATE_CONTROL_PARAMETERS=1;
}
void COMMAND_M202(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters )
{// M202 Dx
	if(UPDATE_CONTROL_PARAMETERS==0)
	{
		FINAL_BOOL_DISTANCE_MM_CACHE=FINAL_BOOL_DISTANCE_MM;
	}
	
	int j=0;
	while(j<Number_Parameters)
	{
			switch(Table_Parameters_Letter[j])
			{
				case 'D':
							FINAL_BOOL_DISTANCE_MM_CACHE=Table_Parameters_Number[j];	
					break;
			}
			j++;
	}
	uint8_t Answer[40];
	sprintf((char*)Answer,"Ok: Bool_D_mm=%0.2f \r\n",FINAL_BOOL_DISTANCE_MM_CACHE);
	Transmit_UART(Answer);
	UPDATE_CONTROL_PARAMETERS=1;
}
void COMMAND_M301(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters )
{// M301 H0/1 Px Ix Dx
	if(UPDATE_CONTROL_PARAMETERS==0)
	{
		P_DISTANCE_CACHE= P_DISTANCE;
		I_DISTANCE_CACHE= I_DISTANCE;
		D_DISTANCE_CACHE= D_DISTANCE;
		P_ANGLE_CACHE= P_ANGLE;
		I_ANGLE_CACHE= I_ANGLE;
		D_ANGLE_CACHE= D_ANGLE;
	}
	int j=0;
	int Type=-1;;
	while(j<Number_Parameters)
	{
			switch(Table_Parameters_Letter[j])
			{
				case 'H':
					Type=Table_Parameters_Number[j];
					break;
				case 'P':
						if(Type==0)
							P_DISTANCE_CACHE=Table_Parameters_Number[j];
						else if(Type==1)
							P_ANGLE_CACHE=Table_Parameters_Number[j];	
					break;
				case 'D':
						if(Type==0)
							D_DISTANCE_CACHE=Table_Parameters_Number[j];
						else if(Type==1)
							D_ANGLE_CACHE=Table_Parameters_Number[j];	
					break;
				case 'I':
						if(Type==0)
							I_DISTANCE_CACHE=Table_Parameters_Number[j];
						else if(Type==1)
							I_ANGLE_CACHE=Table_Parameters_Number[j];
					break;
			}
			j++;
	}
	uint8_t Answer[40];
	if(Type==0)//Distance PID selected
		sprintf((char*)Answer,"OK: D_PID P=%0.2f I=%0.2f D=%0.2f \r\n",P_DISTANCE_CACHE,I_DISTANCE_CACHE,D_DISTANCE_CACHE);
	else if(Type==1)
	  sprintf((char*)Answer,"OK: A_PID P=%0.2f I=%0.2f D=%0.2f \r\n",P_ANGLE_CACHE,I_ANGLE_CACHE,D_ANGLE_CACHE);
	else
		sprintf((char*)Answer,"KO: No PID type selected (H0 or H1) \r\n");
	Transmit_UART(Answer);
	UPDATE_CONTROL_PARAMETERS=1;
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
