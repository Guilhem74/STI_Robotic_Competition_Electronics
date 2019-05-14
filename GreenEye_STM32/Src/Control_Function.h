#ifndef __Control_Function_H
#define __Control_Function_H
#include "Extern_call_variable.h"
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
void Control(void);
float PID_ANGLE(float Error);
float PID_R(float Error);
float PID_L(float Error);
float PID_DISTANCE(float Error);

void Update_POS(void);

#endif
