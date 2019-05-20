#ifndef __Communication_Function_H
#define __Communication_Function_H
#include "Extern_call_variable.h"
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "Communication_function.h"
void Transmit_UART(uint8_t * T) ;
void Analyse_RX_Buffer(void);
int read_double(char *line, int *char_counter, float *double_ptr);
int next_statement(char *letter, float *double_ptr, char *line, int *char_counter);
float strtod_M(const char *str, char **endptr);

void String_Analysis(uint8_t* Input);

void COMMAND_O1(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters );
void COMMAND_G0(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters );//POsition target
void COMMAND_G1(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters );//Speed target
void COMMAND_G2(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters );//Speed target
void COMMAND_G92(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters );//Set actual position
void COMMAND_M3(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters );//Activate position control
void COMMAND_M92(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters );
void COMMAND_M112(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters );// Emergency stop
void COMMAND_M135(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters );// Control loop timing interval
void COMMAND_M201(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters );//Speed acceleration and braking parameters
void COMMAND_M202(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters );//Final bool diameter;
void COMMAND_M301(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters );//Set PID parameters
#endif
