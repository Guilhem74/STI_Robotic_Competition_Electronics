#ifndef __Global_Variables_H
#define __Global_Variables_H
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#define SIZE_UART 64
extern uint8_t UART_TX_DMA[SIZE_UART];
extern uint8_t UART_RX_Analyse[SIZE_UART];
#define SIZE_BUFFER 8
extern uint8_t BUFFER_RX[SIZE_BUFFER][SIZE_UART];
extern uint8_t BUFFER_TX[SIZE_BUFFER][SIZE_UART];
extern uint8_t Indice_Start_RX,Indice_Stop_RX;
extern uint8_t Indice_Start_TX,Indice_Stop_TX;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern DMA_HandleTypeDef hdma_tim1_up;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;

#endif
