#ifndef __Extern_Call_Variables_H
#define __Extern_Call_Variables_H
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#define PI 3.1415f

#define SIZE_UART 64
extern uint8_t UART_RX_Analyse[SIZE_UART];
#define SIZE_BUFFER 8
extern uint8_t BUFFER_RX[SIZE_BUFFER][SIZE_UART],BUFFER_RX_UART2[SIZE_BUFFER][SIZE_UART];
extern uint8_t BUFFER_TX[SIZE_BUFFER][SIZE_UART],BUFFER_TX_UART2[SIZE_BUFFER][SIZE_UART];
extern uint8_t Indice_Start_RX,Indice_Stop_RX;
extern uint8_t Indice_Start_RX_UART2,Indice_Stop_RX_UART2;
extern uint8_t Indice_Start_TX,Indice_Stop_TX;
extern uint8_t Indice_Start_TX_UART2,Indice_Stop_TX_UART2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern DMA_HandleTypeDef hdma_tim1_up;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart2;

extern DMA_HandleTypeDef hdma_usart6_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
/* Robot Parameters*/
extern int16_t Encoder_Right_Past,Encoder_Right,Encoder_Left_Past,Encoder_Left;

extern int8_t UPDATE_POS_PARAMETERS,UPDATE_DEST_PARAMETERS;

extern float ANGLE_POS_RAD,X_POS_MM,Y_POS_MM;
extern float ANGLE_DES_RAD,X_DES_MM,Y_DES_MM;
extern float ANGLE_POS_RAD_CACHE,X_POS_MM_CACHE,Y_POS_MM_CACHE;
extern float ANGLE_DES_RAD_CACHE,X_DES_MM_CACHE,Y_DES_MM_CACHE;
extern float TICS_2_MM, SPACING_WHEELS, TICS_2_MM_CACHE,SPACING_WHEELS_CACHE;
extern float FINAL_BOOL_DISTANCE_MM,FINAL_BOOL_DISTANCE_MM_CACHE;

extern int8_t CONTROL_ENABLED, UPDATE_CONTROL_PARAMETERS;

extern int16_t SPEED_MAX_DISTANCE_MM_S ,SPEED_MAX_DISTANCE_MM_S_CACHE ;
extern int16_t ACCELERATION_MAX_DISTANCE_MM_S2 ,ACCELERATION_MAX_DISTANCE_MM_S2_CACHE ;
extern int16_t BRAKING_MAX_DISTANCE_MM_S2 ,BRAKING_MAX_DISTANCE_MM_S2_CACHE ;
extern int16_t SPEED_MAX_ANGLE_MM_S ,SPEED_MAX_ANGLE_MM_S_CACHE ;
extern int16_t ACCELERATION_MAX_ANGLE_MM_S2 ,ACCELERATION_MAX_ANGLE_MM_S2_CACHE ;
extern int16_t BRAKING_MAX_ANGLE_MM_S2 ,BRAKING_MAX_ANGLE_MM_S2_CACHE ;
extern float ANTICIPATION_PERCENTAGE,ANTICIPATION_PERCENTAGE_CACHE;
extern float LOOP_CONTROL_TIMING_HZ;

extern float P_DISTANCE,P_DISTANCE_CACHE;
extern float I_DISTANCE,I_DISTANCE_CACHE;
extern float D_DISTANCE,D_DISTANCE_CACHE;
extern float P_ANGLE,P_ANGLE_CACHE;
extern float I_ANGLE,I_ANGLE_CACHE;
extern float D_ANGLE,D_ANGLE_CACHE;
#endif
