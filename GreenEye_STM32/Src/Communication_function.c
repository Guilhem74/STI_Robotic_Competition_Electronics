#include "Communication_function.h" 
extern CONTROL_TYPE REGULATOR_CACHE,REGULATOR; 
 
void Transmit_UART_2(uint8_t * T)  
{  
		strcpy((char*)BUFFER_TX_UART2[Indice_Stop_TX_UART2],(char*)T); 
		Indice_Stop_TX_UART2=(Indice_Stop_TX_UART2+1)%(SIZE_BUFFER); 
		if(huart2.gState == HAL_UART_STATE_READY) 
				HAL_UART_Transmit_DMA(&huart2,BUFFER_TX_UART2[Indice_Start_TX_UART2],strlen((char*)BUFFER_TX_UART2[Indice_Start_TX_UART2])); 
 
	 
}  
void Transmit_UART_6(uint8_t * T)  
{  
		strcpy((char*)BUFFER_TX[Indice_Stop_TX],(char*)T); 
		Indice_Stop_TX=(Indice_Stop_TX+1)%(SIZE_BUFFER);  
		if(huart6.gState == HAL_UART_STATE_READY) 
				HAL_UART_Transmit_DMA(&huart6,BUFFER_TX[Indice_Start_TX],strlen((char*)BUFFER_TX[Indice_Start_TX])); 
}  
void Analyse_RX_Buffer() 
{ 
	static int Indice_RX_Analyse=0;
	if(Indice_RX_Analyse>=63)
	{
		Indice_RX_Analyse=0;
		for(int i=0;i<SIZE_UART;i++) 
			UART_RX_Analyse[i]='\0';		
	}
	if(Indice_Start_RX!=Indice_Stop_RX) 
	{//New message to Analyse 
			int j=0;
			while(BUFFER_RX_UART2[Indice_Start_RX_UART2][j]!='\n' && BUFFER_RX_UART2[Indice_Start_RX_UART2][j]!='\0'  && j<64)
			{
				UART_RX_Analyse[Indice_RX_Analyse]=BUFFER_RX_UART2[Indice_Start_RX_UART2][j];
				Indice_RX_Analyse++;
				j++;
			}
			Indice_Start_RX=(Indice_Start_RX+1)%(SIZE_BUFFER); 
	}else if(Indice_Start_RX_UART2!=Indice_Stop_RX_UART2) 
	{ 
		int j=0;
		while(BUFFER_RX_UART2[Indice_Start_RX_UART2][j]!='\n' && BUFFER_RX_UART2[Indice_Start_RX_UART2][j]!='\0'  && j<64)
		{
			UART_RX_Analyse[Indice_RX_Analyse]=BUFFER_RX_UART2[Indice_Start_RX_UART2][j];
			BUFFER_RX_UART2[Indice_Start_RX_UART2][j]='\0';
			Indice_RX_Analyse++;
			j++;
		}
		Indice_Start_RX_UART2=(Indice_Start_RX_UART2+1)%(SIZE_BUFFER); 
	} 
	else 
		return; 
	uint8_t AnalysisEOF=0; 
	for(int i=0;i<Indice_RX_Analyse;i++) 
		{ 
			if(UART_RX_Analyse[i]=='\r') 
				AnalysisEOF=1; 
		} 
	if(AnalysisEOF==1) 
	{ 
		String_Analysis(UART_RX_Analyse); 
		for(int i=0;i<SIZE_UART;i++) 
			UART_RX_Analyse[i]='\0';		
		Indice_RX_Analyse=0;
	} 
 
		 
} 
void String_Analysis(uint8_t* Input) 
{ 
	int char_counter = 0; 
	char letter='\0'; 
	float value=-1; 
	 float Table_values[20]={'\0'}; 
	 uint8_t Table_Letter[20]={0}; 
		int Parameters_N=0; 
	while (next_statement(&letter, &value, (char *) Input, &char_counter)) 
	{ 
		 Table_values[Parameters_N]= (float)value; 
		 Table_Letter[Parameters_N]= (uint8_t)letter; 
		 Parameters_N++;	 
	} 
	if(Parameters_N==0) 
		return; 
	uint8_t Answer[40]=""; 
 
	switch(Table_Letter[0]) 
	{ 
		case 'G': 
				switch((int) Table_values[0]) 
				{ 
					case 0: 
						COMMAND_G0(Table_Letter+1,Table_values+1,Parameters_N-1);//Set destination 
					break; 
					case 1: 
						COMMAND_G1(Table_Letter+1,Table_values+1,Parameters_N-1);//Set Speed destination 
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
					case 15: 
						COMMAND_M15(Table_Letter+1,Table_values+1,Parameters_N-1);//Robot parameters, wheel size... 
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
				Transmit_UART_2(Answer); 
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
	Transmit_UART_2(Answer); 
} 
void COMMAND_G0(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters )  
{//G0 Xx Yx Ax Tx R0/1 
	if(UPDATE_DEST_PARAMETERS==0)  
	{  
		ANGLE_DES_RAD_CACHE=ANGLE_DES_RAD;  
		X_DES_MM_CACHE=X_DES_MM;  
		Y_DES_MM_CACHE=Y_DES_MM;  
		TIMEOUT_MS_CACHE=TIMEOUT_MS;  
		BACKWARD_CACHE=0;//No backward move by default 
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
				case 'T':  
					TIMEOUT_MS_CACHE=Table_Parameters_Number[j];  
					break;  
				case 'R':  
					BACKWARD_CACHE=Table_Parameters_Number[j];  
					break;  
			}  
			j++;  
	}  
	uint8_t Answer[40];  
	sprintf((char*)Answer,"OK: X=%0.2f Y=%0.2f A=%0.2f T=%0.2f B=%d\r\n",X_DES_MM_CACHE,Y_DES_MM_CACHE,ANGLE_DES_RAD_CACHE,TIMEOUT_MS_CACHE,BACKWARD_CACHE);  
	Transmit_UART_2(Answer);  
	UPDATE_DEST_PARAMETERS=1;  
	  
}  
void COMMAND_G1(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters )  
{//G1 Rx Ly  
	if(UPDATE_DEST_PARAMETERS==0)  
	{  
		SPEED_R_DES_CACHE=SPEED_R_DES;  
		SPEED_L_DES_CACHE=SPEED_L_DES;  
		TIMEOUT_MS_CACHE=TIMEOUT_MS;  
	}  
	int j=0;  
	while(j<Number_Parameters)  
	{  
			switch(Table_Parameters_Letter[j])  
			{  
				case 'R':  
					SPEED_R_DES_CACHE=Table_Parameters_Number[j];				  
					break;  
				case 'L':  
					SPEED_L_DES_CACHE=Table_Parameters_Number[j];  
					break;  
				case 'T':  
					TIMEOUT_MS_CACHE=Table_Parameters_Number[j];  
					break;  
			}  
			j++;  
	}  
	uint8_t Answer[40];  
	sprintf((char*)Answer,"OK: D=%0.2f L=%0.2f T=%0.2f\r\n",SPEED_R_DES_CACHE,SPEED_L_DES_CACHE,TIMEOUT_MS_CACHE);  
	Transmit_UART_2(Answer);  
	UPDATE_DEST_PARAMETERS=1;  
}  
void COMMAND_G2(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters )  
{//G2 Rx Ly (PWM)  
	if(UPDATE_DEST_PARAMETERS==0)  
	{  
		PWM_L_DES_CACHE=PWM_L_DES;  
		PWM_R_DES_CACHE=PWM_R_DES;  
		TIMEOUT_MS_CACHE=TIMEOUT_MS;  
	}  
	int j=0;  
	while(j<Number_Parameters)  
	{  
			switch(Table_Parameters_Letter[j])  
			{  
				case 'R':  
					PWM_R_DES_CACHE=Table_Parameters_Number[j];				  
					break;  
				case 'L':  
					PWM_L_DES_CACHE=Table_Parameters_Number[j];  
					break;  
				case 'T':  
					TIMEOUT_MS_CACHE=Table_Parameters_Number[j];  
					break;  
			}  
			j++;  
	}  
	uint8_t Answer[40];  
	sprintf((char*)Answer,"OK: D=%0.2f L=%0.2f T=%0.2f\r\n",PWM_R_DES_CACHE,PWM_L_DES_CACHE,TIMEOUT_MS_CACHE);  
	Transmit_UART_2(Answer);  
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
					ANGLE_POS_RAD_CACHE=Table_Parameters_Number[j]*PI/180;  
					break;  
			}  
			j++;  
	}  
	uint8_t Answer[40];  
	sprintf((char*)Answer,"OK: X=%0.2f Y=%0.2f A=%0.2f \r\n",X_POS_MM_CACHE,Y_POS_MM_CACHE,ANGLE_POS_RAD_CACHE);  
	Transmit_UART_2(Answer);  
	UPDATE_POS_PARAMETERS=1;  
}  
void COMMAND_M3(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters )  
{// M3   
	if(UPDATE_DEST_PARAMETERS==0)  
	{  
	 REGULATOR_CACHE=REGULATOR;  
	}  
	int j=0;  
	SENSOR_ENABLED=0x000F;  
	while(j<Number_Parameters)  
	{  
		  
			switch(Table_Parameters_Letter[j])  
			{  
				case 'H':  
					REGULATOR_CACHE=(CONTROL_TYPE) Table_Parameters_Number[j];				  
				//CONTROL_TYPE {No_Control, PWM_Control, Speed_Control,Position_Control}  
					break;  
				case 'S':  
					SENSOR_ENABLED= Table_Parameters_Number[j];				  
				//CONTROL_TYPE {No_Control, PWM_Control, Speed_Control,Position_Control}  
					break;  
			}  
			j++;  
	}  
	uint8_t Answer[40];  
	sprintf((char*)Answer,"OK: M3 H%d S%d\r\n",REGULATOR_CACHE,SENSOR_ENABLED);  
	Transmit_UART_2(Answer);  
	UPDATE_DEST_PARAMETERS=1; 
}  
void COMMAND_M15(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters ) 
{// M15
	#define A_Sensor 0
	#define B_Sensor 1
	#define C_Sensor 2
	#define D_Sensor 3
	#define E_Sensor 4
	#define F_Sensor 5
	#define G_Sensor 6
	#define H_Sensor 7
	#define I_Sensor 8
	#define J_Sensor 9
	#define K_Sensor 10
	#define L_Sensor 11
	#define M_Sensor 12
	int type=-1;
	int j=0;
	uint8_t Answer[40]; 
	while(j<Number_Parameters) 
	{ 
		 
			switch(Table_Parameters_Letter[j]) 
			{ 
				case 'S': 
					type=Table_Parameters_Number[j];				 
					break; 
			} 
			switch(type) 
			{ 
				case 1: 		
					sprintf((char*)Answer,"M15, BACK: %d;%d;%d;%d\r\n",Result_ADC[A_Sensor], Result_ADC[B_Sensor],Result_ADC[C_Sensor],Result_ADC[D_Sensor]); 
					Transmit_UART_2(Answer); 			 
					break; 
				case 2: 		
					sprintf((char*)Answer,"M15, LEFT: %d;%d;%d\r\n",Result_ADC[G_Sensor], Result_ADC[H_Sensor],Result_ADC[J_Sensor]); 
					Transmit_UART_2(Answer); 			 
					break; 
				case 4: 		
					sprintf((char*)Answer,"M15, RIGHT: %d;%d;%d\r\n",Result_ADC[E_Sensor], Result_ADC[I_Sensor],Result_ADC[F_Sensor]); 
					Transmit_UART_2(Answer); 			 
					break; 
				case 8: 		
					sprintf((char*)Answer,"M15, FRONT: %d;%d;%d\r\n",Result_ADC[K_Sensor], Result_ADC[L_Sensor],Result_ADC[M_Sensor]); 
					Transmit_UART_2(Answer); 			 
					break; 
			} 
			j++; 
	} 
	
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
	Transmit_UART_2(Answer); 
	UPDATE_POS_PARAMETERS=1; 
} 
void COMMAND_M112(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters ) 
{// M112 
	CONTROL_ENABLED=0; 
	TIM2->CCR1=0;//Set PWM to 0 
	TIM2->CCR2=0;//Set PWM to 0 
	uint8_t Answer[40]; 
	sprintf((char*)Answer,"OK: Emergency Stop \r\n"); 
	Transmit_UART_2(Answer); 
} 
void COMMAND_M135(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters ) 
{// M135 Sx 
	if(Table_Parameters_Letter[0]=='S') 
		LOOP_CONTROL_TIMING_HZ=Table_Parameters_Number[0]; 
	uint8_t Answer[40]; 
	sprintf((char*)Answer,"OK: Timing loop : %0.2f \r\n",LOOP_CONTROL_TIMING_HZ); 
	Transmit_UART_2(Answer); 
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
	Transmit_UART_2(Answer); 
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
	Transmit_UART_2(Answer); 
	UPDATE_CONTROL_PARAMETERS=1; 
} 
void COMMAND_M301(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters ) 
{// M301 H0/1 Px Ix Dx 
	if(UPDATE_CONTROL_PARAMETERS==0) 
	{ 
		P_SPEED_CACHE= P_SPEED; 
		I_SPEED_CACHE= I_SPEED; 
		D_SPEED_CACHE= D_SPEED; 
		P_ANGLE_CACHE= P_ANGLE; 
		I_ANGLE_CACHE= I_ANGLE; 
		D_ANGLE_CACHE= D_ANGLE; 
		P_DISTANCE_CACHE= P_DISTANCE; 
		I_DISTANCE_CACHE= I_DISTANCE; 
		D_DISTANCE_CACHE= D_DISTANCE; 
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
							P_SPEED_CACHE=Table_Parameters_Number[j]; 
						else if(Type==1) 
							P_ANGLE_CACHE=Table_Parameters_Number[j];	 
						else if(Type==2) 
							P_DISTANCE_CACHE=Table_Parameters_Number[j];	 
					break; 
				case 'D': 
						if(Type==0) 
							D_SPEED_CACHE=Table_Parameters_Number[j]; 
						else if(Type==1) 
							D_ANGLE_CACHE=Table_Parameters_Number[j];	 
						else if(Type==2) 
							D_DISTANCE_CACHE=Table_Parameters_Number[j]; 
					break; 
				case 'I': 
						if(Type==0) 
							I_SPEED_CACHE=Table_Parameters_Number[j]; 
						else if(Type==1) 
							I_ANGLE_CACHE=Table_Parameters_Number[j]; 
						else if(Type==2) 
							I_DISTANCE_CACHE=Table_Parameters_Number[j]; 
					break; 
			} 
			j++; 
	} 
	uint8_t Answer[40]; 
	if(Type==0)//Distance PID selected 
		sprintf((char*)Answer,"OK: S_PID P=%0.2f I=%0.2f D=%0.2f \r\n",P_SPEED_CACHE,I_SPEED_CACHE,D_SPEED_CACHE); 
	else if(Type==1) 
	  sprintf((char*)Answer,"OK: A_PID P=%0.2f I=%0.2f D=%0.2f \r\n",P_ANGLE_CACHE,I_ANGLE_CACHE,D_ANGLE_CACHE); 
	else if(Type==2) 
	  sprintf((char*)Answer,"OK: D_PID P=%0.2f I=%0.2f D=%0.2f \r\n",P_DISTANCE_CACHE,I_DISTANCE_CACHE,D_DISTANCE_CACHE); 
	else 
		sprintf((char*)Answer,"KO: No PID type selected (H0 or H1 or H2) \r\n"); 
	Transmit_UART_2(Answer); 
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
