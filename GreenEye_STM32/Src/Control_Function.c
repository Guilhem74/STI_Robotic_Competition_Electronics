#include "Control_Function.h"
void Control(void)
{
	static float Previous_Error_Distance=0;
			static float Previous_Error_Angle_Rad=0;
			static float Previous_Speed_Distance=0;
			static float Previous_Speed_Angle=0;
			static float Previous_Target_Distance_Speed=0;
			static float Previous_Target_Angle_Speed=0;

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
				if(abs(Delta_Encoder_Right)>65535/4||abs(Delta_Encoder_Left)>65535/4)
				{
				//Probably sampling issues, overflowing/ missing steps seems ineluctable at that point.
						uint8_t Answer[40];
						sprintf((char*)Answer,"ERROR: Delta encoder is high  \r\n");
						Transmit_UART(Answer);
				}
				volatile float Distance=(Delta_Encoder_Right+Delta_Encoder_Left)*TICS_2_MM/(2);
				volatile float Angle_rad=(Delta_Encoder_Right-Delta_Encoder_Left)*TICS_2_MM/(SPACING_WHEELS);
				ANGLE_POS_RAD+=Angle_rad;
				float ANGLE_POS_DEG=ANGLE_POS_RAD*180/PI;


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
					Previous_Error_Distance=0;
					Previous_Error_Angle_Rad=0;
					Previous_Speed_Distance=0;
					Previous_Speed_Angle=0;
				  Previous_Target_Distance_Speed=0;
			    Previous_Target_Angle_Speed=0;
					return;
				}
				volatile int Error_X=(X_DES_MM-X_POS_MM);
				volatile int Error_Y=(Y_DES_MM-Y_POS_MM);
				volatile int Error_Distance=sqrt(Error_X*Error_X+Error_Y*Error_Y);
				volatile float Error_Angle_Rad=atan2(Error_Y,Error_X);
				while(Error_Angle_Rad>PI)
					Error_Angle_Rad-=2*PI;
				while(Error_Angle_Rad<-PI)
					Error_Angle_Rad+=2*PI;
				if(Error_Angle_Rad>PI/2+0.0001 || Error_Angle_Rad<-PI/2-0.0001)//Backward move
				{
					Error_Distance=-Error_Distance;
					Error_Angle_Rad+=PI;
				}
				volatile int Target_Distance_Speed=Error_Distance;
				volatile float Target_Angle_Speed=Error_Angle_Rad;
				volatile int Speed_Distance=fabs(Previous_Error_Distance-Error_Distance)*LOOP_CONTROL_TIMING_HZ;
				volatile float Speed_Angle=fabs(Previous_Error_Angle_Rad-Error_Angle_Rad)*LOOP_CONTROL_TIMING_HZ;
				volatile int Acceleration_Distance=fabs(Previous_Speed_Distance-Speed_Distance)*LOOP_CONTROL_TIMING_HZ;
				volatile float Acceleration_Angle=fabs(Previous_Speed_Angle-Speed_Angle)*LOOP_CONTROL_TIMING_HZ;
				volatile int Distance_Braking=(Speed_Distance*Speed_Distance)/(2*BRAKING_MAX_DISTANCE_MM_S2);
				volatile float Angle_Braking=(Speed_Angle*Speed_Angle)/(2*BRAKING_MAX_ANGLE_MM_S2);
//				volatile int Target_Distance_Speed=0;
//				volatile float Target_Angle_Speed=0;
//				/* Distance Phase*/
//				if(fabs(Error_Distance)<(Distance_Braking+Distance_Braking*ANTICIPATION_PERCENTAGE))
//				{//Braking phase we reduce the speed by an increment of 1
//					Target_Distance_Speed=fabs(Previous_Target_Distance_Speed)-BRAKING_MAX_DISTANCE_MM_S2/LOOP_CONTROL_TIMING_HZ;
//				}
//				else if(fabs(Previous_Target_Distance_Speed)<SPEED_MAX_DISTANCE_MM_S)
//				{// Acceleration phase
//					Target_Distance_Speed=fabs(Previous_Target_Distance_Speed)+ACCELERATION_MAX_DISTANCE_MM_S2/LOOP_CONTROL_TIMING_HZ;
//				}
//				else
//				{//Constant speed phase
//					Target_Distance_Speed=SPEED_MAX_DISTANCE_MM_S;
//				}
//				/*END Distance Phase*/
//				
//				/* Angular Phase*/
//				if(fabs(Error_Angle_Rad)<(Angle_Braking+Angle_Braking*ANTICIPATION_PERCENTAGE))
//				{//Braking phase we reduce the speed by an increment of 1
//					Target_Angle_Speed=fabs(Previous_Target_Angle_Speed)-BRAKING_MAX_ANGLE_MM_S2/LOOP_CONTROL_TIMING_HZ;
//				}
//				else if(fabs(Previous_Target_Angle_Speed)<SPEED_MAX_ANGLE_MM_S)
//				{// Acceleration phase
//					Target_Angle_Speed=fabs(Previous_Target_Angle_Speed)+ACCELERATION_MAX_ANGLE_MM_S2/LOOP_CONTROL_TIMING_HZ;
//				}
//				else
//				{//Constant speed phase
//					Target_Angle_Speed=SPEED_MAX_ANGLE_MM_S;
//				}
//				/*END Angular Phase*/
//				if(Error_Distance<0)
//				{
//					Target_Distance_Speed=-Target_Distance_Speed;
//				}
//				if(Error_Angle_Rad<0)
//				{
//					Target_Angle_Speed=-Target_Angle_Speed;
//				}
				if(abs(Error_Distance)<FINAL_BOOL_DISTANCE_MM)
				{
					
					Target_Distance_Speed=0;
					if(fabs(Error_Angle_Rad*180/PI)<5)
						Target_Angle_Speed=0;
					//TODO Finish line
				}
				volatile int Target_DISTANCE_PID=(int)(Target_Distance_Speed*P_DISTANCE);
				volatile int Target_ANGLE_PID=(int)(Target_Angle_Speed*P_ANGLE);
				volatile int Output_Right_Motor=(int)(Target_DISTANCE_PID-Target_ANGLE_PID);
				volatile int Output_Left_Motor=(int)(Target_DISTANCE_PID+Target_ANGLE_PID);
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
				TIM2->CCR2=(Output_Right_Motor);
				TIM2->CCR1=(Output_Left_Motor);
				uint8_t Answer[40];
				sprintf((char*)Answer,"PID : D:%d G:%d EA:%0.2f ED:%d ES:%d\r\n",Output_Right_Motor,Output_Left_Motor,Error_Angle_Rad,Error_Distance,Target_Distance_Speed);
				Transmit_UART(Answer);
				Previous_Error_Distance=Error_Distance;
				Previous_Error_Angle_Rad=Error_Angle_Rad;
				Previous_Speed_Distance=Speed_Distance;
				Previous_Speed_Angle=Speed_Angle;
				Previous_Target_Distance_Speed=Target_Distance_Speed;
				Previous_Target_Angle_Speed=Target_Angle_Speed;
}
