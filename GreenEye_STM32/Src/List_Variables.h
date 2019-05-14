#ifndef __List_variables__H
#define __List_variables__H
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "math.h"
#define PI 3.1415f
/* Communication Variables*/
#define SIZE_UART 64 
uint8_t UART_RX_Analyse[SIZE_UART]=""; 
#define SIZE_BUFFER 8 
uint8_t BUFFER_RX[SIZE_BUFFER][SIZE_UART]; 
uint8_t BUFFER_TX[SIZE_BUFFER][SIZE_UART]; 
uint8_t Indice_Start_RX=0,Indice_Stop_RX=0; 
uint8_t Indice_Start_TX=0,Indice_Stop_TX=0; 
uint8_t BUFFER_RX_UART2[SIZE_BUFFER][SIZE_UART]; 
uint8_t BUFFER_TX_UART2[SIZE_BUFFER][SIZE_UART]; 
uint8_t Indice_Start_RX_UART2=0,Indice_Stop_RX_UART2=0; 
uint8_t Indice_Start_TX_UART2=0,Indice_Stop_TX_UART2=0; 

uint16_t Sensor=0;
uint32_t Result_ADC[13]={0};
/* Robot localization Variables*/
int16_t Encoder_Right_Past=0,Encoder_Right=0,Encoder_Left_Past=0,Encoder_Left=0;
float ANGLE_POS_RAD=0,X_POS_MM=0,Y_POS_MM=0;
float ANGLE_POS_RAD_CACHE=0,X_POS_MM_CACHE=0,Y_POS_MM_CACHE=0;
float ANGLE_DES_RAD=0,X_DES_MM=0,Y_DES_MM=0;
float ANGLE_DES_RAD_CACHE=0,X_DES_MM_CACHE=0,Y_DES_MM_CACHE=0;
float SPEED_R_DES=0,SPEED_R_DES_CACHE=0;
float SPEED_L_DES=0,SPEED_L_DES_CACHE=0;
float PWM_R_DES=0,PWM_R_DES_CACHE=0;
float PWM_L_DES=0,PWM_L_DES_CACHE=0;
#define DEFAULT_TIMEOUT 60000
float TIMEOUT_MS=DEFAULT_TIMEOUT,TIMEOUT_MS_CACHE=DEFAULT_TIMEOUT;

int8_t UPDATE_POS_PARAMETERS=0,UPDATE_DEST_PARAMETERS=0;
int8_t CONTROL_ENABLED=0, UPDATE_CONTROL_PARAMETERS=0;

float TICS_2_MM=0.08835,TICS_2_MM_CACHE=0.08835;
float SPACING_WHEELS=137,SPACING_WHEELS_CACHE=137;
float FINAL_BOOL_DISTANCE_MM=50,FINAL_BOOL_DISTANCE_MM_CACHE=50;
float FINAL_BOOL2_DISTANCE_MM=20,FINAL_BOOL2_DISTANCE_MM_CACHE=20;
/*Speed and acceleration profile*/
int16_t SPEED_MAX_DISTANCE_MM_S=150,SPEED_MAX_DISTANCE_MM_S_CACHE=150;
int16_t ACCELERATION_MAX_DISTANCE_MM_S2=50,ACCELERATION_MAX_DISTANCE_MM_S2_CACHE=50;
int16_t BRAKING_MAX_DISTANCE_MM_S2=100,BRAKING_MAX_DISTANCE_MM_S2_CACHE=100;
int16_t SPEED_MAX_ANGLE_MM_S=150,SPEED_MAX_ANGLE_MM_S_CACHE=150;
int16_t ACCELERATION_MAX_ANGLE_MM_S2=20,ACCELERATION_MAX_ANGLE_MM_S2_CACHE=20;
int16_t BRAKING_MAX_ANGLE_MM_S2=400,BRAKING_MAX_ANGLE_MM_S2_CACHE=400;
float ANTICIPATION_PERCENTAGE=10,ANTICIPATION_PERCENTAGE_CACHE=10;
float LOOP_CONTROL_TIMING_HZ=100;
uint8_t STATUS_BOOL_1=0,STATUS_BOOL_2=0;
float R_SPEED_TARGET=0, L_SPEED_TARGET=0;
/* PID CONTROL*/
float P_SPEED=6,P_SPEED_CACHE=6;//2
float I_SPEED=5,I_SPEED_CACHE=5;
float D_SPEED=6,D_SPEED_CACHE=6;
float P_ANGLE=5,P_ANGLE_CACHE=5;
float I_ANGLE=0,I_ANGLE_CACHE=0;
float D_ANGLE=0,D_ANGLE_CACHE=0;
float P_DISTANCE=1,P_DISTANCE_CACHE=1;
float I_DISTANCE=0,I_DISTANCE_CACHE=0;
float D_DISTANCE=0,D_DISTANCE_CACHE=0;
#endif
