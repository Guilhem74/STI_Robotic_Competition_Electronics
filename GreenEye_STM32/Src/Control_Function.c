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
				int16_t Delta_Encoder_Right=Encoder_Right-Encoder_Right_Past;
				int16_t Delta_Encoder_Left=Encoder_Left-Encoder_Left_Past;
				float Distance=(Delta_Encoder_Right+Delta_Encoder_Left)*TICS_2_MM/(2);
				float Angle_rad=((float)((int16_t)(Delta_Encoder_Right-Delta_Encoder_Left))*TICS_2_MM)/(SPACING_WHEELS);
				if(abs(Delta_Encoder_Right)>abs(Max_Delta_R))
					Max_Delta_R=Delta_Encoder_Right;
				if(abs(Delta_Encoder_Left)>abs(Max_Delta_L))
					Max_Delta_L=Delta_Encoder_Left;
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
				Encoder_Right_Control=TIM1->CNT;
				Encoder_Left_Control=TIM3->CNT;
				int16_t Delta_Encoder_Right=Encoder_Right_Control-Encoder_Right_Past_Control;
				int16_t Delta_Encoder_Left=Encoder_Left_Control-Encoder_Left_Past_Control;
				Encoder_Right_Past_Control=Encoder_Right_Control;
				Encoder_Left_Past_Control=Encoder_Left_Control;
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
				}
				static float TIMEOUT_COUNTER=0;
				if(UPDATE_DEST_PARAMETERS==1)
				{//Command to change destination parameters
					UPDATE_DEST_PARAMETERS=0;
					ANGLE_DES_RAD= ANGLE_DES_RAD_CACHE;
					X_DES_MM =  X_DES_MM_CACHE;
					Y_DES_MM =  Y_DES_MM_CACHE;
					SPEED_L_DES=SPEED_L_DES_CACHE;
					SPEED_R_DES=SPEED_R_DES_CACHE;
					PWM_R_DES=PWM_R_DES_CACHE;
					PWM_L_DES=PWM_L_DES_CACHE;
					TIMEOUT_MS=TIMEOUT_MS_CACHE;
					REGULATOR=REGULATOR_CACHE;
					TIMEOUT_COUNTER=0;
				}
				if(REGULATOR_CACHE!=REGULATOR)
				{
					REGULATOR=REGULATOR_CACHE;
					TIMEOUT_COUNTER=0;
				}
				int Error_X=(X_DES_MM-X_POS_MM);
				int Error_Y=(Y_DES_MM-Y_POS_MM);
				float Error_Distance=sqrt(Error_X*Error_X+Error_Y*Error_Y);
				float Error_Angle_Rad=0;
				
				if(Error_Distance<FINAL_BOOL_DISTANCE_MM)
				{
					Error_Angle_Rad=ANGLE_DES_RAD-ANGLE_POS_RAD;//atan2(Error_Y,Error_X)-ANGLE_POS_RAD;
				}
				else
				{
					Error_Angle_Rad=atan2(Error_Y,Error_X)-ANGLE_POS_RAD;

				}			
					Error_Angle_Rad=atan2(Error_Y,Error_X)-ANGLE_POS_RAD;
				while(Error_Angle_Rad>PI)
					Error_Angle_Rad-=2*PI;
				while(Error_Angle_Rad<-PI)
					Error_Angle_Rad+=2*PI;
				if(Error_Angle_Rad>PI/2||Error_Angle_Rad<-PI/2)//Backward move
					{
						Error_Distance=-Error_Distance;
						Error_Angle_Rad+=PI;
					}
				while(Error_Angle_Rad>PI)
					Error_Angle_Rad-=2*PI;
				while(Error_Angle_Rad<-PI)
					Error_Angle_Rad+=2*PI;
				Error_Distance=Error_Distance*fabs(cos(Error_Angle_Rad));
				if(fabs(Error_Distance)<FINAL_BOOL_DISTANCE_MM)
				{
					if(fabs(Error_Distance)<FINAL_BOOL2_DISTANCE_MM|| STATUS_BOOL_2==1)
					{
						STATUS_BOOL_2=1;
						Error_Distance=0;			
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
						uint8_t Answer[40];
						//sprintf((char*)Answer,"%0.2f;%0.2f;%0.2f;%0.2f\r\n",Error_Distance,Error_Angle_Rad,Output_PID_R,Output_PID_L);
						Transmit_UART(Answer);
						 break;
				}
				TIMEOUT_COUNTER++;
				if(TIMEOUT_COUNTER>=(LOOP_CONTROL_TIMING_HZ/1000*TIMEOUT_MS))
				{
					if(REGULATOR!=No_Control)
					{
					uint8_t Answer[40];
					sprintf((char*)Answer,"Timeout %0.2f\n\r",TIMEOUT_MS);
				  Transmit_UART(Answer);
					}
					REGULATOR_CACHE=No_Control;// Stall instead ?
				}
//				if(fabs(R_SPEED_TARGET)<20)
//					R_SPEED_TARGET=0;
//				if(fabs(L_SPEED_TARGET)<20)
//					L_SPEED_TARGET=0;
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
				//sprintf((char*)Answer,"%0.2f;%0.2f;%0.2f;%0.2f;%0.2f;%0.2f;%0.2f\r\n",R_SPEED_TARGET,L_SPEED_TARGET,Output_PID_R,Output_PID_L,Output_Right_Motor,Output_Left_Motor,Error_Angle_Deg);
				//

				
				
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
//	uint8_t Answer[40];
//	sprintf((char*)Answer,"%0.2f;%0.2f;%0.2f;%0.2f\r\n",Local_Error,Delta_Error,Sum_Error,Output);
//	Transmit_UART(Answer);
  

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
//	uint8_t Answer[40];
//	sprintf((char*)Answer,"%0.2f;%0.2f;%0.2f;%0.2f\r\n",Local_Error,Delta_Error,Sum_Error,Output);
//	Transmit_UART(Answer);
  

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
//	uint8_t Answer[40];
//	sprintf((char*)Answer,"%0.2f;%0.2f;%0.2f;%0.2f\r\n",Local_Error,Delta_Error,Sum_Error,Output);
//	Transmit_UART(Answer);
  

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
//	uint8_t Answer[40];
//	sprintf((char*)Answer,"%0.2f;%0.2f;%0.2f;%0.2f\r\n",Local_Error,Delta_Error,Sum_Error,Output);
//	Transmit_UART(Answer);
  
  return Output;
}
float Avoidance(float *Error_Distance,float * Error_Angle_Deg)
{
	if(*Error_Distance>0 && (Result_ADC[11]>1500 ||  Result_ADC[12]>1500 || Result_ADC[10]>1500))
	{
		*Error_Distance=0;

	}
	else if(*Error_Distance<0 && (Result_ADC[0]>1500 ||  Result_ADC[1]>1500 || Result_ADC[2]>1500 || Result_ADC[3]>1500))
	{
		*Error_Distance=0;

	}

	return 0.0;
}
