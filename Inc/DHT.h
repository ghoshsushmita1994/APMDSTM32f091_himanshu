/*
 * DHT11.h
 *
 *  Created on: April 25, 2022
 *      Author: Harshita Jain
 */


#ifndef INC_DHT_H_
#define INC_DHT_H_

#include "stm32F0xx_hal.h"

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void delay (uint16_t time);
void DHT11_Start (void);
uint8_t DHT11_Check_Response (void);
uint8_t DHT11_Read (void);



#endif /* INC_DHT11_H_ */
