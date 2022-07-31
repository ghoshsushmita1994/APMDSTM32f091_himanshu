/*
 * DHT.c
 *
 *  Created on: April 25, 2022
 *      Author: Harshita Jain
 */


#include "DHT.h"
#include "stdio.h"
#include "string.h"

extern TIM_HandleTypeDef htim16;

#define PORTB GPIOB
#define DHT11_PIN GPIO_PIN_2

/*************************************************
 * delay (uint16_t time)
 * For generating micro Second delay
 */

void delay (uint16_t time)
{
		__HAL_TIM_SET_COUNTER(&htim16, 0);
		while ( __HAL_TIM_GET_COUNTER(&htim16) < time);

}

/*******************************************************
 * Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
 * set DHT pin in output mode
 */
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/*******************************************************
 * Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
 * set DHT pin in input mode
 */
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL; // changed by Harshita
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/*******************************************************
 * DHT11_Start (void)
 * MCU Sends out start signal to DHT11
 */
void DHT11_Start (void)
{
	//	/*************Modified By Harshita********************/
		HAL_GPIO_WritePin (PORTB, DHT11_PIN, 1);   // pull the pin high
		HAL_Delay(1500);


		Set_Pin_Output (PORTB, DHT11_PIN);  // set the pin as output
		HAL_GPIO_WritePin (PORTB, DHT11_PIN, 0);   // pull the pin low
		delay (18000);   // wait for 18ms
	    HAL_GPIO_WritePin (PORTB, DHT11_PIN, 1);   // pull the pin high
		delay (20);   // wait for 20us
		Set_Pin_Input(PORTB, DHT11_PIN);    // set as input
}

/**********************************************************
 * DHT11_Check_Response (void)
 * DHT Respose to MCU
 *
 */

uint8_t DHT11_Check_Response (void)
{
	uint8_t Response = 0;
	delay (40);
	if (!(HAL_GPIO_ReadPin (PORTB, DHT11_PIN)))
	{
		delay (80);
		if ((HAL_GPIO_ReadPin (PORTB, DHT11_PIN))) Response = 1;
		else Response = -1; // 255
	}
	while ((HAL_GPIO_ReadPin (PORTB, DHT11_PIN)));   // wait for the pin to go low

	return Response;


}


/*****************************************
 * DHT11_Read (void)
 * For reading Temperature and Humidity Values.
 */

uint8_t DHT11_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (PORTB, DHT11_PIN)));   // wait for the pin to go high
		delay (40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin (PORTB, DHT11_PIN)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (PORTB, DHT11_PIN)));  // wait for the pin to go low
	}
	return i;
}
