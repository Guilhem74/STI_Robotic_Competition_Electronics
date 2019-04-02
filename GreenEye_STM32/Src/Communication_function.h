#ifndef __Communication_Function_H
#define __Communication_Function_H
#include "Extern_call_variable.h"
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
void Transmit_UART(uint8_t * T) ;
void Analyse_RX_Buffer(void);
int read_double(char *line, int *char_counter, float *double_ptr);
int next_statement(char *letter, float *double_ptr, char *line, int *char_counter);
float strtod_M(const char *str, char **endptr);

void String_Analysis(uint8_t* Input);


void COMMAND_G92(uint8_t* Table_Parameters_Letter,float* Table_Parameters_Number, int8_t Number_Parameters );

#endif
