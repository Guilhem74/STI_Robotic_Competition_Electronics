#ifndef __Communication_Function_H
#define __Communication_Function_H
#include "Extern_call_variable.h"
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
void Transmit_UART(uint8_t * T) ;
void Analyse_RX_Buffer(void);
int read_double(char *line, uint8_t *char_counter, int16_t *double_ptr);
int next_statement(char *letter, int16_t *double_ptr, char *line, uint8_t *char_counter);
#endif
