/*
Copyright (c) 2016, Dust Networks.  All rights reserved.
*/

#ifndef DEVICES_H
#define DEVICES_H

#include "dn_typedef.h"
#include "dn_time.h" // to provide access to units from getSystemTime()

//=========================== defines =========================================
#define GPIO_STATE_INPUT 1
#define GPIO_STATE_OUTPUT 0
#define GPIO_OUTPUT_HIGH 1
#define GPIO_OUTPUT_LOW 0

//=========================== prototypes ======================================
void initBatADC();
INT16U readADC(INT8U adcIndex);
INT64U getSystemTime(void);
INT16S getTemp(void); 
INT16U getSupplyV(void);

void digitalWrite(int pin, INT8U output); 
INT8U digitalRead(int pin); 
void initGPIO(int pin, INT8U pinState); 
void configGPIO(int pin, INT8U pinState); 

#endif
