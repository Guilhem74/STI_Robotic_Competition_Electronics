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
				Encoder_Right=TIM1->CNT;
				Encoder_Left=TIM3->CNT;
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
				int Output_Right_Motor=R_SPEED_TARGET;
				int Output_Left_Motor=L_SPEED_TARGET;

				if(Output_Right_Motor>1500)
				{
					Output_Right_Motor=1500;
				}
				else if(Output_Right_Motor<-1500)
				{
					Output_Right_Motor=-1500;
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
				TIM2->CCR2=abs(Output_Right_Motor);
				TIM2->CCR3=abs(Output_Left_Motor);
				uint8_t Answer[40];
				sprintf((char*)Answer,"%0.2f;%0.2f\r\n",Delta_Encoder_Right*TICS_2_MM*LOOP_CONTROL_TIMING_HZ,Delta_Encoder_Left*TICS_2_MM*LOOP_CONTROL_TIMING_HZ);
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
