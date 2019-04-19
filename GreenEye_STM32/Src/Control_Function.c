#include "Control_Function.h"
#include "Communication_function.h"
#include <math.h>
float PID_R(float Error);
float PID_L(float Error);
void Control(void)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);


			if(UPDATE_DEST_PARAMETERS==1)
			{//Command to change destination parameters
				UPDATE_DEST_PARAMETERS=0;
				ANGLE_DES_RAD= ANGLE_DES_RAD_CACHE;
				X_DES_MM =  X_DES_MM_CACHE;
				Y_DES_MM =  Y_DES_MM_CACHE;
			}
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
				Encoder_Right=TIM5->CNT;
				Encoder_Left=TIM1->CNT;
				int16_t Delta_Encoder_Right=Encoder_Right-Encoder_Right_Past;
				int16_t Delta_Encoder_Left=Encoder_Left-Encoder_Left_Past;
				#define Integrate_Speed_Size 10
				static float Speed_R[Integrate_Speed_Size]={0};
				static float Speed_L [Integrate_Speed_Size]={0};
				static int i=0;
				Speed_R[i]=Delta_Encoder_Right*TICS_2_MM*LOOP_CONTROL_TIMING_HZ;
				Speed_L[i]=Delta_Encoder_Left*TICS_2_MM*LOOP_CONTROL_TIMING_HZ;
				i=(i+1)%Integrate_Speed_Size;
				float Speed_Average_R=0;
				float Speed_Average_L=0;
				for(int j=0;j<Integrate_Speed_Size;j++)
				{
					Speed_Average_R+=  Speed_R[j];
					Speed_Average_L+=  Speed_L[j];
				}
				Speed_Average_R=Speed_Average_R/Integrate_Speed_Size;
				Speed_Average_L=Speed_Average_L/Integrate_Speed_Size;
				if(abs(Delta_Encoder_Right)>65535/4||abs(Delta_Encoder_Left)>65535/4)
				{
				//Probably sampling issues, overflowing/ missing steps seems ineluctable at that point.
						uint8_t Answer[40];
						sprintf((char*)Answer,"ERROR: Delta encoder is high  \r\n");
						Transmit_UART(Answer);
				}
				float Distance=(Delta_Encoder_Right+Delta_Encoder_Left)*TICS_2_MM/(2);
				float Angle_rad=(Delta_Encoder_Right-Delta_Encoder_Left)*TICS_2_MM/(SPACING_WHEELS);
				ANGLE_POS_RAD+=Angle_rad;
				X_POS_MM +=  Distance * cos(ANGLE_POS_RAD);
				Y_POS_MM +=  Distance * sin(ANGLE_POS_RAD);
				
				/*END Localisaiton math*/
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
					P_DISTANCE=P_DISTANCE_CACHE;
					I_DISTANCE=I_DISTANCE_CACHE;
					D_DISTANCE=D_DISTANCE_CACHE;
					P_ANGLE=P_ANGLE_CACHE;
					I_ANGLE=I_ANGLE_CACHE;
					D_ANGLE=D_ANGLE_CACHE;
				}
				if(!CONTROL_ENABLED)
				{

					return;
				}
				int Error_X=(X_DES_MM-X_POS_MM);
				int Error_Y=(Y_DES_MM-Y_POS_MM);
				float Error_Angle_Rad=atan2(Error_Y,Error_X)-ANGLE_POS_RAD;
				float Error_Distance=sqrt(Error_X*Error_X+Error_Y*Error_Y);
				while(Error_Angle_Rad>PI)
					Error_Angle_Rad-=2*PI;
				while(Error_Angle_Rad<-PI)
					Error_Angle_Rad+=2*PI;
				Error_Distance=Error_Distance*fabs(cos(Error_Angle_Rad));
				if(Error_Angle_Rad>PI/2||Error_Angle_Rad<-PI/2)//Backward move
				{
					Error_Distance=-Error_Distance;
					Error_Angle_Rad+=PI;
				}


				while(Error_Angle_Rad>PI)
					Error_Angle_Rad-=2*PI;
				while(Error_Angle_Rad<-PI)
					Error_Angle_Rad+=2*PI;
				 float Error_Speed_Right = R_SPEED_TARGET - Delta_Encoder_Right*TICS_2_MM*LOOP_CONTROL_TIMING_HZ;
				 float Error_Speed_Left = L_SPEED_TARGET - Delta_Encoder_Left*TICS_2_MM*LOOP_CONTROL_TIMING_HZ;
				 int Output_Right_Motor=PID_R(Error_Speed_Right);
				 int Output_Left_Motor=PID_L(Error_Speed_Left);
				if(Output_Right_Motor>4019)
				{
					Output_Right_Motor=4019;
				}
				else if(Output_Right_Motor<-4019)
				{
					Output_Right_Motor=-4019;
				}
				if(Output_Left_Motor>4019)
				{
					Output_Left_Motor=4019;
				}
				else if(Output_Left_Motor<-4019)
				{
					Output_Left_Motor=-4019;
				}
				if(Output_Right_Motor>0)
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
				else
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
				if(Output_Left_Motor>0)
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
				else
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
				TIM2->CCR2=abs(Output_Right_Motor);
				TIM2->CCR1=abs(Output_Left_Motor);
				uint8_t Answer[40];
				sprintf((char*)Answer,"%0.2f;%0.2f;%0.2f;%0.2f;%d;%d\r\n",R_SPEED_TARGET,Delta_Encoder_Right*TICS_2_MM*LOOP_CONTROL_TIMING_HZ,L_SPEED_TARGET,Delta_Encoder_Left*TICS_2_MM*LOOP_CONTROL_TIMING_HZ,Output_Right_Motor,Output_Left_Motor);
				Transmit_UART(Answer);

}

float PID_R(float Error) {
  static float Previous_Error = 0;
	static float Local_Error=0;
	static float Previous_output=0;
	Previous_Error=Local_Error;
	Local_Error=Error;
  float Delta_Error = Local_Error - Previous_Error; // D
//  static int i=0;
//	#define Integrator_Size 20
//	float Sum_Error;
//	static float Table_Error[Integrator_Size]={0};
//	Table_Error[i]=Local_Error;
//	i=(i+1)%Integrator_Size;
//  for(int j=0;j<Integrator_Size;j++)
//  {
//    Sum_Error+=  Table_Error[j];
//  }
//	Sum_Error=Sum_Error/Integrator_Size;
	float Output =
      P_DISTANCE * Local_Error + I_DISTANCE * (Local_Error+Previous_output)  +
      D_DISTANCE * Delta_Error; // On determine la commande a envoyer
	Previous_output=Output;
//	uint8_t Answer[40];
//	sprintf((char*)Answer,"%0.2f;%0.2f;%0.2f;%0.2f\r\n",Local_Error,Delta_Error,Sum_Error,Output);
//	Transmit_UART(Answer);
  

  return Output;
}
float PID_L(float Error) {
  static float Previous_Error = 0;
	static float Local_Error=0;
	static float Previous_output=0;
	Previous_Error=Local_Error;
	Local_Error=Error;
  float Delta_Error = Local_Error - Previous_Error; // D
//  static int i=0;
//	#define Integrator_Size 20
//	float Sum_Error;
//	static float Table_Error[Integrator_Size]={0};
//	Table_Error[i]=Local_Error;
//	i=(i+1)%Integrator_Size;
//  for(int j=0;j<Integrator_Size;j++)
//  {
//    Sum_Error+=  Table_Error[j];
//  }
//	Sum_Error=Sum_Error/Integrator_Size;
	float Output =
      P_DISTANCE * Local_Error + I_DISTANCE * (Local_Error+Previous_output)  +
      D_DISTANCE * Delta_Error; // On determine la commande a envoyer
	Previous_output=Output;
//	uint8_t Answer[40];
//	sprintf((char*)Answer,"%0.2f;%0.2f;%0.2f;%0.2f\r\n",Local_Error,Delta_Error,Sum_Error,Output);
//	Transmit_UART(Answer);
  

  return Output;
}