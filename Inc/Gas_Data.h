/*
 * Gas_Data.h
 *
 *  Created on: April 26, 2022
 *      Author: Harshita Jain
 */

#ifndef INC_Gas_Data_H_
#define INC_Gas_Data_H_

#include "stm32F0xx_hal.h"


void ADC_Select_CH0 (void);
void ADC_Select_CH1 (void);
void ADC_Select_CH2 (void);
void ADC_Select_CH3 (void);
void ADC_Select_CH4 (void);
void ADC_Select_CH5 (void);
void ADC_Select_CH6 (void);
void ADC_Select_CH7 (void);
void ADC_Select_CH8 (void);
void ADC_Select_CH12 (void);
void ADC_Select_CH13 (void);
void ADC_Select_CH14 (void);
void ADC_Select_CH15 (void);

float NO2_Read (uint16_t TEMP);
float OX_Read (uint16_t TEMP);
float CO_Read (uint16_t TEMP);
float SO2_Read (uint16_t TEMP);
void PM_Read(float *PM1, float *PM2,float *PM3, float *PM4, float *PM5 );




#endif /* INC_Gas_Data_H_ */


