/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Extern_call_variable.h"
#include "Communication_function.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_tim1_up;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim10;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim10);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
	//Check if interrupt come from the timer 10
			static int16_t Previous_Error_Distance=0;
			static int16_t Previous_Error_Angle_Rad=0;
			static int16_t Previous_Speed_Distance=0;
			static int16_t Previous_Speed_Angle=0;
			static int16_t Previous_Target_Distance_Speed=0;
			static int16_t Previous_Target_Angle_Speed=0;

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
				Encoder_Right=TIM1->CCR1;
				Encoder_Left=TIM5->CCR1;
				int16_t Delta_Encoder_Right=Encoder_Right-Encoder_Right_Past;
				int16_t Delta_Encoder_Left=Encoder_Left-Encoder_Left_Past;
				if(abs(Delta_Encoder_Right)>65535/4||abs(Delta_Encoder_Left)>65535/4)
				{
				//Probably sampling issues, overflowing/ missing steps seems ineluctable at that point.
				}
				int32_t Distance=(Delta_Encoder_Right*TICS_2_MM+Delta_Encoder_Left*TICS_2_MM)/2;
				int32_t Angle_rad=(Delta_Encoder_Right*TICS_2_MM-Delta_Encoder_Left*TICS_2_MM)/SPACING_WHEELS;
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
					Previous_Error_Distance=0;
					Previous_Error_Angle_Rad=0;
					Previous_Speed_Distance=0;
					Previous_Speed_Angle=0;
				  Previous_Target_Distance_Speed=0;
			    Previous_Target_Angle_Speed=0;
					return;
				}
				int16_t Error_X=(X_DES_MM-X_POS_MM);
				int16_t Error_Y=(Y_DES_MM-Y_POS_MM);
				int16_t Error_Distance=sqrt(Error_X^2+Error_Y^2);
				int16_t Error_Angle_Rad=atan2(Error_X,Error_Y);
				if(Error_Angle_Rad>PI)
					Error_Angle_Rad-=2*PI;
				if(Error_Angle_Rad<-PI)
					Error_Angle_Rad+=2*PI;
				if(Error_Angle_Rad>PI/2 || Error_Angle_Rad<-PI/2)//Backward move
				{
					Error_Distance=-Error_Distance;
					Error_Angle_Rad+=PI;
				}
				int16_t Speed_Distance=abs(Previous_Error_Distance-Error_Distance)/LOOP_CONTROL_TIMING;
				int16_t Speed_Angle=abs(Previous_Error_Angle_Rad-Error_Angle_Rad)/LOOP_CONTROL_TIMING;
				int16_t Acceleration_Distance=abs(Previous_Speed_Distance-Speed_Distance)/LOOP_CONTROL_TIMING;
				int16_t Acceleration_Angle=abs(Previous_Speed_Angle-Speed_Angle)/LOOP_CONTROL_TIMING;
				int16_t Distance_Braking=(Speed_Distance*Speed_Distance)/(2*BRAKING_MAX_DISTANCE_MM_S2);
				int16_t Angle_Braking=(Speed_Angle*Speed_Angle)/(2*BRAKING_MAX_ANGLE_MM_S2);
				int16_t Target_Distance_Speed=0;
				int16_t Target_Angle_Speed=0;
				/* Distance Phase*/
				if(abs(Error_Distance)<(Distance_Braking+Distance_Braking*ANTICIPATION_PERCENTAGE))
				{//Braking phase we reduce the speed by an increment of 1
					Target_Distance_Speed=abs(Previous_Target_Distance_Speed)-BRAKING_MAX_DISTANCE_MM_S2*LOOP_CONTROL_TIMING;
				}
				else if(abs(Previous_Target_Distance_Speed)<SPEED_MAX_DISTANCE_MM_S)
				{// Acceleration phase
					Target_Distance_Speed=abs(Previous_Target_Distance_Speed)+ACCELERATION_MAX_DISTANCE_MM_S2*LOOP_CONTROL_TIMING;
				}
				else
				{//Constant speed phase
					Target_Distance_Speed=SPEED_MAX_DISTANCE_MM_S;
				}
				/*END Distance Phase*/
				
				/* Angular Phase*/
				if(abs(Error_Angle_Rad)<(Angle_Braking+Angle_Braking*ANTICIPATION_PERCENTAGE))
				{//Braking phase we reduce the speed by an increment of 1
					Target_Angle_Speed=abs(Previous_Target_Angle_Speed)-BRAKING_MAX_ANGLE_MM_S2*LOOP_CONTROL_TIMING;
				}
				else if(abs(Previous_Target_Angle_Speed)<SPEED_MAX_ANGLE_MM_S)
				{// Acceleration phase
					Target_Angle_Speed=abs(Previous_Target_Angle_Speed)+ACCELERATION_MAX_ANGLE_MM_S2*LOOP_CONTROL_TIMING;
				}
				else
				{//Constant speed phase
					Target_Angle_Speed=SPEED_MAX_ANGLE_MM_S;
				}
				/*END Angular Phase*/
				if(Error_Distance<0)
				{
					Target_Distance_Speed=-Target_Distance_Speed;
				}
				if(Error_Angle_Rad<0)
				{
					Target_Angle_Speed=-Target_Angle_Speed;
				}
				if(Error_Distance<FINAL_BOOL_DISTANCE_MM)
				{
					Target_Distance_Speed=0;
					//TODO Finish line
				}
				//TODO PID
				
				Previous_Error_Distance=Error_Distance;
				Previous_Error_Angle_Rad=Error_Angle_Rad;
				Previous_Speed_Distance=Speed_Distance;
				Previous_Speed_Angle=Speed_Angle;
				Previous_Target_Distance_Speed=Target_Distance_Speed;
				Previous_Target_Angle_Speed=Target_Angle_Speed;
  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
		if (__HAL_UART_GET_FLAG (&huart2, UART_FLAG_IDLE))
		{//Got some delay in the communication, time to check if the frame is full
			//Store what has been received
			//Restart DMA
			Indice_Stop_RX_UART2=(Indice_Stop_RX_UART2+1)%SIZE_BUFFER;
			HAL_UART_DMAStop(&huart2);
			HAL_UART_Receive_DMA (&huart2, BUFFER_RX_UART2[Indice_Stop_RX_UART2], SIZE_UART);
			__HAL_UART_CLEAR_IDLEFLAG (&huart2);
		}
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream5 global interrupt.
  */
void DMA2_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream5_IRQn 0 */

  /* USER CODE END DMA2_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim1_up);
  /* USER CODE BEGIN DMA2_Stream5_IRQn 1 */

  /* USER CODE END DMA2_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */
		if (__HAL_UART_GET_FLAG (&huart6, UART_FLAG_IDLE))
		{//Got some delay in the communication, time to check if the frame is full
			//Store what has been received
			//Restart DMA
			Indice_Stop_RX=(Indice_Stop_RX+1)%SIZE_BUFFER;
			HAL_UART_DMAStop(&huart6);
			HAL_UART_Receive_DMA (&huart6, BUFFER_RX[Indice_Stop_RX], SIZE_UART);
			__HAL_UART_CLEAR_IDLEFLAG (&huart6);
		}

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{//End of TX transmission
    if (huart->Instance == USART6)  // change USART instance
    {
			Indice_Start_TX=(Indice_Start_TX+1)%SIZE_BUFFER;
			if(Indice_Start_TX!=Indice_Stop_TX && huart6.gState == HAL_UART_STATE_READY)
			{
				HAL_UART_Transmit_DMA(&huart6,BUFFER_TX[Indice_Start_TX],strlen((char*)BUFFER_TX[Indice_Start_TX]));
				strcpy((char *)BUFFER_TX[Indice_Start_TX], "");
			}
    }
		if (huart->Instance == USART2)  // change USART instance
    {
			Indice_Start_TX_UART2=(Indice_Start_TX_UART2+1)%SIZE_BUFFER;
			if(Indice_Start_TX_UART2!=Indice_Stop_TX_UART2 && huart2.gState == HAL_UART_STATE_READY)
			{
				HAL_UART_Transmit_DMA(&huart2,BUFFER_TX_UART2[Indice_Start_TX_UART2],strlen((char*)BUFFER_TX_UART2[Indice_Start_TX_UART2]));
				strcpy((char *)BUFFER_TX_UART2[Indice_Start_TX_UART2], "");
			}
    }
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
