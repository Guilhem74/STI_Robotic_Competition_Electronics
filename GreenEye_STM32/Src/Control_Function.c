#include "Control_Function.h"
#include "Communication_function.h"
#include <math.h>
int16_t Max_Delta_R=0;
int16_t Max_Delta_L=0;
extern CONTROL_TYPE REGULATOR,REGULATOR_CACHE;

void Update_POS(void)
{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);



			if(UPDATE_POS_PARAMETERS==1)
			{//Command to change position parameters
				UPDATE_POS_PARAMETERS=0;
				ANGLE_POS_RAD= ANGLE_POS_RAD_CACHE;
				X_POS_MM =  X_POS_MM_CACHE;
				Y_POS_MM =  Y_POS_MM_CACHE;
				TICS_2_MM=TICS_2_MM_CACHE;
				SPACING_WHEELS=SPACING_WHEELS_CACHE;
			}
				/* Localisation math*/
				Encoder_Right_Past=Encoder_Right;
				Encoder_Left_Past=Encoder_Left;
				Encoder_Right=TIM1->CNT;
				Encoder_Left=TIM3->CNT;
				int16_t Delta_Encoder_Right=(Encoder_Right-Encoder_Right_Past);
				int16_t Delta_Encoder_Left=Encoder_Left-Encoder_Left_Past;
				float Distance=(Delta_Encoder_Right+Delta_Encoder_Left*1.0015)*TICS_2_MM/(2);
				float Angle_rad=((float)(Delta_Encoder_Right-Delta_Encoder_Left*1.0015)*TICS_2_MM)/(SPACING_WHEELS);
				ANGLE_POS_RAD+=Angle_rad;
				X_POS_MM +=  Distance * cos(ANGLE_POS_RAD);
				Y_POS_MM +=  Distance * sin(ANGLE_POS_RAD);
				/*END Localisaiton math*/
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);

}
void Control(void)//100hz 
{ 
				static int16_t Encoder_Right_Control=0; 
				static int16_t Encoder_Left_Control=0; 
				static int16_t Encoder_Right_Past_Control=0; 
				static int16_t Encoder_Left_Past_Control=0; 
				static uint8_t Arrived_Transmitted=0; 
				#define	SIZE_SPEED_ARRAY 200 //In seconds * 100 
				static float Speed_Array_R[SIZE_SPEED_ARRAY]={0}; 
				static float Speed_Array_L[SIZE_SPEED_ARRAY]={0}; 
				static int Indice_Speed_Array=0; 
				Encoder_Right_Control=TIM1->CNT; 
				Encoder_Left_Control=TIM3->CNT; 
				int16_t Delta_Encoder_Right=Encoder_Right_Control-Encoder_Right_Past_Control; 
				Speed_Array_R[Indice_Speed_Array]=Delta_Encoder_Right*TICS_2_MM*LOOP_CONTROL_TIMING_HZ; 
				int16_t Delta_Encoder_Left=Encoder_Left_Control-Encoder_Left_Past_Control; 
				Speed_Array_L[Indice_Speed_Array]=Delta_Encoder_Left*TICS_2_MM*LOOP_CONTROL_TIMING_HZ; 
				Indice_Speed_Array=(Indice_Speed_Array+1)%SIZE_SPEED_ARRAY; 
				Encoder_Right_Past_Control=Encoder_Right_Control; 
				Encoder_Left_Past_Control=Encoder_Left_Control; 
				static float TIMEOUT_COUNTER=0; 
 
				if(UPDATE_CONTROL_PARAMETERS) 
				{ 
					UPDATE_CONTROL_PARAMETERS=0; 
					FINAL_BOOL_DISTANCE_MM=FINAL_BOOL_DISTANCE_MM_CACHE; 
					SPEED_MAX_DISTANCE_MM_S = SPEED_MAX_DISTANCE_MM_S_CACHE ; 
					ACCELERATION_MAX_DISTANCE_MM_S2 = ACCELERATION_MAX_DISTANCE_MM_S2_CACHE ; 
					BRAKING_MAX_DISTANCE_MM_S2 = BRAKING_MAX_DISTANCE_MM_S2_CACHE ; 
					SPEED_MAX_ANGLE_MM_S = SPEED_MAX_ANGLE_MM_S_CACHE ; 
					ACCELERATION_MAX_ANGLE_MM_S2 = ACCELERATION_MAX_ANGLE_MM_S2_CACHE ; 
					BRAKING_MAX_ANGLE_MM_S2 = BRAKING_MAX_ANGLE_MM_S2_CACHE ; 
					ANTICIPATION_PERCENTAGE = ANTICIPATION_PERCENTAGE_CACHE; 
					P_SPEED=P_SPEED_CACHE; 
					I_SPEED=I_SPEED_CACHE; 
					D_SPEED=D_SPEED_CACHE; 
					P_ANGLE=P_ANGLE_CACHE; 
					I_ANGLE=I_ANGLE_CACHE; 
					D_ANGLE=D_ANGLE_CACHE; 
					P_DISTANCE=P_DISTANCE_CACHE; 
					I_DISTANCE=I_DISTANCE_CACHE; 
					D_DISTANCE=D_DISTANCE_CACHE; 
					for(int j=0;j<SIZE_SPEED_ARRAY;j++) 
					{ 
						Speed_Array_R[j]=45; 
						Speed_Array_L[j]=45; 
					} 
					Arrived_Transmitted=0; 
					TIMEOUT_COUNTER=0; 
				} 
				if(UPDATE_DEST_PARAMETERS==1) 
				{//Command to change destination parameters 
					REGULATOR=REGULATOR_CACHE; 
					UPDATE_DEST_PARAMETERS=0; 
					ANGLE_DES_RAD= ANGLE_DES_RAD_CACHE; 
					X_DES_MM =  X_DES_MM_CACHE; 
					Y_DES_MM =  Y_DES_MM_CACHE; 
					SPEED_L_DES=SPEED_L_DES_CACHE; 
					SPEED_R_DES=SPEED_R_DES_CACHE; 
					PWM_R_DES=PWM_R_DES_CACHE; 
					PWM_L_DES=PWM_L_DES_CACHE; 
					TIMEOUT_MS=TIMEOUT_MS_CACHE; 
					TIMEOUT_COUNTER=0; 
					STATUS_BOOL_2=0; 
					BACKWARD=BACKWARD_CACHE; 
					for(int j=0;j<SIZE_SPEED_ARRAY;j++) 
					{ 
						Speed_Array_R[j]=45; 
						Speed_Array_L[j]=45; 
					} 
					Arrived_Transmitted=0; 
					TIMEOUT_COUNTER=0; 
				} 
				if(REGULATOR_CACHE==No_Control) 
					REGULATOR=No_Control; 
				int Error_X=(X_DES_MM-X_POS_MM); 
				int Error_Y=(Y_DES_MM-Y_POS_MM); 
				float Error_Distance=sqrt(Error_X*Error_X+Error_Y*Error_Y); 
				float True_Distance=Error_Distance; 
				float Error_Angle_Rad=0; 
				Error_Angle_Rad=atan2(Error_Y,Error_X)-ANGLE_POS_RAD; 
				while(Error_Angle_Rad>=PI) 
					Error_Angle_Rad-=2*PI; 
				while(Error_Angle_Rad<-PI) 
					Error_Angle_Rad+=2*PI; 
				if( ((Error_Angle_Rad>PI/2+PI/10||Error_Angle_Rad<-(PI/2+PI/10)) && fabs(Error_Distance)<150) || (BACKWARD==1 &&((Error_Angle_Rad>PI/2+PI/10||Error_Angle_Rad<-(PI/2+PI/10)))))//The point is behind us and not enough distance to turn around 
				{//Case we passed our arrival point or if we allowed backward movement 
					Error_Distance=-Error_Distance; //=DO NOT REVERSE 
					Error_Angle_Rad+=PI; 
				} 
				else if((Error_Angle_Rad>PI/2||Error_Angle_Rad<-(PI/2))) 
				{//Try to reverse and getting closer 
					Error_Distance=-Error_Distance; 
				} 
				while(Error_Angle_Rad>=PI) 
					Error_Angle_Rad-=2*PI; 
				while(Error_Angle_Rad<-PI) 
					Error_Angle_Rad+=2*PI; 
				Error_Distance=Error_Distance*fabs(cos(Error_Angle_Rad)); 
				if(fabs(True_Distance)<FINAL_BOOL_DISTANCE_MM) 
				{ 
					if(fabs(True_Distance)<FINAL_BOOL2_DISTANCE_MM|| STATUS_BOOL_2==1) 
					{ 
						STATUS_BOOL_2=1; 
						Error_Distance=0;	 
						if(REGULATOR ==Position_Control && Arrived_Transmitted==0) 
						{ 
							uint8_t Answer[64]; 
							sprintf((char*)Answer,"M0 X%0.2f Y%0.2f A%0.2f T0 S%d\r\n",X_POS_MM,Y_POS_MM,ANGLE_POS_RAD*180/PI,SENSOR_DETECTED);  
							Transmit_UART_2(Answer);	 
							Arrived_Transmitted=1; 
							REGULATOR_CACHE=No_Control;// Stall instead ? 
						} 
					} 
				} 
				else 
				{ 
					STATUS_BOOL_1=0; 
					STATUS_BOOL_2=0; 
				} 
				float Error_Angle_Deg=Error_Angle_Rad*(180/PI); 
				Avoidance(&Error_Distance,&Error_Angle_Deg); 
 
				float Output_Angle=PID_ANGLE(Error_Angle_Deg); 
				float Output_Distance=PID_DISTANCE(Error_Distance); 
				if(Output_Distance> SPEED_MAX_DISTANCE_MM_S) 
					Output_Distance=SPEED_MAX_DISTANCE_MM_S; 
				else if(Output_Distance< -SPEED_MAX_DISTANCE_MM_S) 
					Output_Distance=-SPEED_MAX_DISTANCE_MM_S; 
				/*Limitation of maximum speed to turn*/ 
				if(Output_Angle> SPEED_MAX_ANGLE_MM_S) 
					Output_Angle=SPEED_MAX_ANGLE_MM_S; 
				else if(Output_Angle< -SPEED_MAX_ANGLE_MM_S) 
					Output_Angle=-SPEED_MAX_ANGLE_MM_S; 
				float Output_PID_R=0; 
				float Output_PID_L=0; 
				float Error_Speed_Right=0; 
				float Error_Speed_Left=0; 
				switch(REGULATOR) 
				{//CONTROL_TYPE {No_Control, PWM_Control, Speed_Control,Position_Control} 
					case(Speed_Control): 
							R_SPEED_TARGET=SPEED_R_DES; 
							L_SPEED_TARGET=SPEED_L_DES; 
							Error_Speed_Right = R_SPEED_TARGET - Delta_Encoder_Right*TICS_2_MM*LOOP_CONTROL_TIMING_HZ; 
							Error_Speed_Left = L_SPEED_TARGET - Delta_Encoder_Left*TICS_2_MM*LOOP_CONTROL_TIMING_HZ; 
							Output_PID_R=PID_R(Error_Speed_Right); 
							Output_PID_L=PID_L(Error_Speed_Left); 
							break; 
					case(PWM_Control): 
							 Output_PID_R=PWM_R_DES; 
							 Output_PID_L=PWM_L_DES; 
						 break; 
					case(No_Control): 
							 Output_PID_R=0; 
							 Output_PID_L=0; 
						 break; 
					case(Position_Control): 
							R_SPEED_TARGET=+Output_Angle+Output_Distance; 
							L_SPEED_TARGET=-Output_Angle+Output_Distance; 
							Error_Speed_Right = R_SPEED_TARGET - Delta_Encoder_Right*TICS_2_MM*LOOP_CONTROL_TIMING_HZ; 
							Error_Speed_Left = L_SPEED_TARGET - Delta_Encoder_Left*TICS_2_MM*LOOP_CONTROL_TIMING_HZ; 
							Output_PID_R=PID_R(Error_Speed_Right); 
							Output_PID_L=PID_L(Error_Speed_Left); 
								 
						 break; 
				} 
				TIMEOUT_COUNTER++; 
				if(TIMEOUT_COUNTER>=(LOOP_CONTROL_TIMING_HZ/1000*TIMEOUT_MS)) 
				{ 
					if(REGULATOR!=No_Control) 
					{ 
						uint8_t Answer[64]; 
						sprintf((char*)Answer,"M0 X%0.2f Y%0.2f A%0.2f T1 S%d\r\n",X_POS_MM,Y_POS_MM,ANGLE_POS_RAD*180/PI,SENSOR_DETECTED);  
						Transmit_UART_2(Answer); 
							Arrived_Transmitted=1;  
					} 
					REGULATOR_CACHE=No_Control;// Stall instead ? 
				} 
				/* SPEED Regulation Part*/ 
				// 09/05/2019 PID with a speed coeff of 1.5 and P10 I1 D0 
				float Output_Right_Motor=Output_PID_R;  
				float Output_Left_Motor=Output_PID_L; 
 
				if(Output_Right_Motor>2100) 
				{ 
					Output_Right_Motor=2100; 
				} 
				else if(Output_Right_Motor<-2100) 
				{ 
					Output_Right_Motor=-2100; 
				} 
				if(Output_Left_Motor>2100) 
				{ 
					Output_Left_Motor=2100; 
				} 
				else if(Output_Left_Motor<-2100) 
				{ 
					Output_Left_Motor=-2100; 
				} 
				 
				if(Output_Right_Motor>0) 
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET); 
				else 
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET); 
				if(Output_Left_Motor>0) 
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET); 
				else 
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET); 
				TIM2->CCR2=(int) fabs(Output_Right_Motor); 
				TIM2->CCR3=(int) fabs(Output_Left_Motor); 
				if(REGULATOR ==Position_Control && Arrived_Transmitted==0)  
				{//Distance error is null, angle error too, Speed right and left too  
					volatile float Average_Speed_R=0;  
					volatile float Average_Speed_L=0;  
					for(int j=0;j<SIZE_SPEED_ARRAY;j++)  
					{  
						Average_Speed_R+=fabs(Speed_Array_R[j]);  
						Average_Speed_L+=fabs(Speed_Array_L[j]);  
  
					}		  
					if(Average_Speed_R<2*SIZE_SPEED_ARRAY && Average_Speed_L<2*SIZE_SPEED_ARRAY && SENSOR_DETECTED>1)  
					{  
						uint8_t Answer[64];  
						sprintf((char*)Answer,"M0 X%0.2f Y%0.2f A%0.2f T2 S%d\r\n",X_POS_MM,Y_POS_MM,ANGLE_POS_RAD*180/PI,SENSOR_DETECTED);   
						Transmit_UART_2(Answer);	  
						Arrived_Transmitted=1;  
						REGULATOR_CACHE=No_Control;// Stall instead ?  
					}  
				}  
		static int Count=0; 
		if(Count>2) 
		{ 
			Count=0; 
			uint8_t Answer[64];  
			sprintf((char*)Answer,"M0 X%0.2f Y%0.2f A%0.2f\r\n",X_POS_MM,Y_POS_MM,ANGLE_POS_RAD*180/PI);  
			HAL_UART_Transmit(&huart6,Answer,strlen((char*)Answer),1000); 
		} 
		Count++; 
 
				 
				 
} 

float PID_R(float Error) {
  static float Previous_Error = 0;
	static float Local_Error=0;
	Previous_Error=Local_Error;
	Local_Error=Error;
  float Delta_Error = Local_Error - Previous_Error; // D
  static int i=0;
	#define Integrator_Size 10
	float Sum_Error;
	static float Table_Error[Integrator_Size]={0};
	Table_Error[i]=Local_Error;
	i=(i+1)%Integrator_Size;
 for(int j=0;j<Integrator_Size;j++)
  {
	  Sum_Error+=  Table_Error[j];
  }
	Sum_Error=Sum_Error/Integrator_Size;
	float Output =
      P_SPEED * Local_Error + I_SPEED * (Sum_Error)  +
      D_SPEED * Delta_Error; // On determine la commande a envoyer
//	uint8_t Answer[64];
//	sprintf((char*)Answer,"%0.2f;%0.2f;%0.2f;%0.2f\r\n",Local_Error,Delta_Error,Sum_Error,Output);
//	Transmit_UART_2(Answer);
  

  return Output;
}
float PID_L(float Error) {
  static float Previous_Error = 0;
	static float Local_Error=0;
	Previous_Error=Local_Error;
	Local_Error=Error;
  float Delta_Error = Local_Error - Previous_Error; // D
  static int i=0;
	#define Integrator_Size 10
	float Sum_Error;
	static float Table_Error[Integrator_Size]={0};
	Table_Error[i]=Local_Error;
	i=(i+1)%Integrator_Size;
  for(int j=0;j<Integrator_Size;j++)
  {
    Sum_Error+=  Table_Error[j];
  }
	Sum_Error=Sum_Error/Integrator_Size;
	float Output =
      P_SPEED * Local_Error + I_SPEED * Sum_Error  +
      D_SPEED * Delta_Error; // On determine la commande a envoyer
//	uint8_t Answer[64];
//	sprintf((char*)Answer,"%0.2f;%0.2f;%0.2f;%0.2f\r\n",Local_Error,Delta_Error,Sum_Error,Output);
//	Transmit_UART_2(Answer);
  

  return Output;
}
float PID_ANGLE(float Error) {
  static float Previous_Error = 0;
	static float Local_Error=0;
	Previous_Error=Local_Error;
	Local_Error=Error;
  float Delta_Error = Local_Error - Previous_Error; // D
  static int i=0;
	#define Integrator_Size 10
	float Sum_Error;
	static float Table_Error[Integrator_Size]={0};
	Table_Error[i]=Local_Error;
	i=(i+1)%Integrator_Size;
  for(int j=0;j<Integrator_Size;j++)
  {
    Sum_Error+=  Table_Error[j];
  }
	Sum_Error=Sum_Error/Integrator_Size;
	float Output =
      P_ANGLE * Local_Error + I_ANGLE * Sum_Error  +
      D_ANGLE * Delta_Error; // On determine la commande a envoyer
//	uint8_t Answer[64];
//	sprintf((char*)Answer,"%0.2f;%0.2f;%0.2f;%0.2f\r\n",Local_Error,Delta_Error,Sum_Error,Output);
//	Transmit_UART_2(Answer);
  

  return Output;
}

float PID_DISTANCE(float Error) {
  static float Previous_Error = 0;
	static float Local_Error=0;
	Previous_Error=Local_Error;
	Local_Error=Error;
  float Delta_Error = Local_Error - Previous_Error; // D
  static int i=0;
	#define Integrator_Size 10
	float Sum_Error;
	static float Table_Error[Integrator_Size]={0};
	Table_Error[i]=Local_Error;
	i=(i+1)%Integrator_Size;
  for(int j=0;j<Integrator_Size;j++)
  {
    Sum_Error+=  Table_Error[j];
  }
	Sum_Error=Sum_Error/Integrator_Size;
	float Output =
      P_DISTANCE * Local_Error + I_DISTANCE * Sum_Error  +
      D_DISTANCE * Delta_Error; // On determine la commande a envoyer
//	uint8_t Answer[64];
//	sprintf((char*)Answer,"%0.2f;%0.2f;%0.2f;%0.2f\r\n",Local_Error,Delta_Error,Sum_Error,Output);
//	Transmit_UART_2(Answer);
  
  return Output;
}
float Avoidance(float *Error_Distance,float * Error_Angle_Deg)
{//0xFront Left Right Back
	//typedef enum Sensors_Letter {A, B, C,D,E,F,G,H,I,J,K, L, M}Sensors_Letter;
	//                              0 1  2 3 4 5 6 7 8 9 10 11 12
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
	int Threshold_Distance=1350;
	int Threshold_Distance_Front_Side=1250;
	int Threshold_Side_For_Front=2050;

	int Threshold_Distance_AR=1750; 
	int Threshold_Angle=1500; 

	uint16_t Threshold_Array[13]={Threshold_Distance_AR,Threshold_Distance_AR,Threshold_Distance_AR,Threshold_Distance_AR,Threshold_Angle,Threshold_Angle,Threshold_Angle,Threshold_Angle,Threshold_Angle,Threshold_Angle,Threshold_Distance,Threshold_Distance,Threshold_Distance};
	if(*Error_Distance>0 && (Result_ADC[L_Sensor]>Threshold_Distance_Front_Side ||  Result_ADC[M_Sensor]>Threshold_Distance || Result_ADC[K_Sensor]>Threshold_Distance_Front_Side 
		|| (Result_ADC[E]>Threshold_Side_For_Front && !(Result_ADC[I]>Threshold_Side_For_Front))  || (Result_ADC[G]>Threshold_Side_For_Front && !(Result_ADC[J]>Threshold_Side_For_Front))) 
		&& ((SENSOR_ENABLED & 0x0008)!=0)) 
	{//FRONT 
		*Error_Distance=0; 
	}
	
	else if(*Error_Distance<0 && (Result_ADC[A_Sensor]>Threshold_Distance_AR ||  Result_ADC[B_Sensor]>Threshold_Distance_AR || Result_ADC[C_Sensor]>Threshold_Distance_AR || Result_ADC[D_Sensor]>Threshold_Distance_AR) && ((SENSOR_ENABLED & 0x0001)!=0)) 
	{//BACK 
		*Error_Distance=0; 
	} 
	if(*Error_Angle_Deg>0 && (Result_ADC[G_Sensor]>Threshold_Angle ||  Result_ADC[J_Sensor]>Threshold_Angle||  Result_ADC[H_Sensor]>Threshold_Angle)&& ((SENSOR_ENABLED & 0x0002)!=0)) 
	{//Turn CCW -> Sensor Left 
		*Error_Angle_Deg=0; 
	} 
	else if(*Error_Angle_Deg<0 && (Result_ADC[E_Sensor]>Threshold_Angle ||  Result_ADC[I_Sensor]>Threshold_Angle ||  Result_ADC[F_Sensor]>Threshold_Angle  )&&(SENSOR_ENABLED & 0x0004)!=0) 
	{//Turn CW -> Sensor Right 
		*Error_Angle_Deg=0; 
	} 
	SENSOR_DETECTED=0;
	for(int i=A;i<=M;i++)
	{
		if(Result_ADC[i]>Threshold_Array[i])
		{
			SENSOR_DETECTED|=1<<i;
		}
	}

	return 0.0;
}
