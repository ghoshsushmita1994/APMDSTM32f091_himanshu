/*
 * Gas_Data.c
 *
 *  Created on: April 26, 2022
 *      Author: Harshita Jain
 */

/*********************************************************************/
/**
 * Pin Cofiguration
 *
 * PC2 -- WE_NO2	--IN12
 * PC3 -- AE_NO2	--IN13
 * PA0 -- WE_OX		--IN0
 * PA1 -- AE_OX		--IN1
 * PA2 -- WE_CO		--IN2
 * PA3 -- AE_CO		--IN3
 * PA4 -- WE_SO2	--IN4
 * PA5 -- AE_SO2	--IN5
 * PA6 -- WE_H2S	--IN6
 * PA7 -- AE_H2S	--IN7
 * PB0 -- PM_OUT	--IN8
 * PC4 -- WE_NO		--IN14
 * PC5 -- AE_NO		--IN15
 */
/************************************************************************/

#include "Gas_Data.h"
#include "stdio.h"
#include "string.h"

extern	ADC_HandleTypeDef hadc;
extern ADC_ChannelConfTypeDef sConfig;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim16;


#define PORTA GPIOA
#define PCON_PM GPIO_PIN_15
#define PORTC GPIOC
#define PCON_AFE1 GPIO_PIN_8

/*****************************************
 * ADC_Select_CH0
 * For selecting Channel 0
 */
void ADC_Select_CH0 (void)
{
	  sConfig.Channel = ADC_CHANNEL_0; //IN0 CHANNEL 0	PIN PA0
	  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

/*****************************************
 * ADC_Select_CH1
 * For selecting Channel 1
 */

void ADC_Select_CH1 (void)
{
	  sConfig.Channel = ADC_CHANNEL_1;	//IN1 CHANNEL 1 PIN PA1
	  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

/*****************************************
 * ADC_Select_CH2
 * For selecting Channel 2
 */
void ADC_Select_CH2 (void)
{
	  sConfig.Channel = ADC_CHANNEL_2;	//IN2 CHANNEL 2	PIN PA2
	  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

/*****************************************
 * ADC_Select_CH3
 * For selecting Channel 3
 */
void ADC_Select_CH3 (void)
{
	  sConfig.Channel = ADC_CHANNEL_3;	//IN3 CHANNEL 3	PIN PA3
	  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

/*****************************************
 * ADC_Select_CH4
 * For selecting Channel 4
 */
void ADC_Select_CH4 (void)
{
	  sConfig.Channel = ADC_CHANNEL_4;	//IN4 CHANNEL 4	PIN PA4
	  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

/*****************************************
 * ADC_Select_CH5
 * For selecting Channel 5
 */
void ADC_Select_CH5 (void)
{
	  sConfig.Channel = ADC_CHANNEL_5;		//IN5 CHANNEL 5 PIN PA5
	  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

/*****************************************
 * ADC_Select_CH6
 * For selecting Channel 6
 */
void ADC_Select_CH6 (void)
{
	  sConfig.Channel = ADC_CHANNEL_6;	//IN6 CHANNEL 6	PIN PA6
	  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

/*****************************************
 * ADC_Select_CH7
 * For selecting Channel 7
 */
void ADC_Select_CH7 (void)
{
	  sConfig.Channel = ADC_CHANNEL_7;	//IN7 CHANNEL 7	PIN PA7
	  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

/*****************************************
 * ADC_Select_CH8
 * For selecting Channel 8
 */
void ADC_Select_CH8 (void)
{
	  sConfig.Channel = ADC_CHANNEL_8;	//IN7 CHANNEL 8	PIN PB0
	  sConfig.Rank =  ADC_RANK_CHANNEL_NUMBER;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

/*****************************************
 * ADC_Select_CH12
 * For selecting Channel 12
 */

void ADC_Select_CH12 (void)
{
	  sConfig.Channel = ADC_CHANNEL_12;	//IN12 CHANNEL 12	PIN PC2
	  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

/*****************************************
 * ADC_Select_CH13
 * For selecting Channel 13
 */
void ADC_Select_CH13 (void)
{
	  sConfig.Channel = ADC_CHANNEL_13;	//IN13 CHANNEL 13	PIN PC3
	  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

/*****************************************
 * ADC_Select_CH14
 * For selecting Channel 14
 */
void ADC_Select_CH14 (void)
{
	  sConfig.Channel = ADC_CHANNEL_14;	//IN12 CHANNEL 14	PIN PC4
	  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

/*****************************************
 * ADC_Select_CH15
 * For selecting Channel 15
 */
void ADC_Select_CH15 (void)
{
	  sConfig.Channel = ADC_CHANNEL_14;	//IN12 CHANNEL 15	PIN PC5
	  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}


/***************************************************************
 * float NO2_Read (uint16_t TEMP)
 * for reading NO2 ug/m^3 concentration
 * PC2 -- WE_NO2	--IN12
 * PC3 -- AE_NO2	--IN13
 */
float NO2_Read (uint16_t TEMP)
{
	uint16_t SN1_WE;
	uint16_t SN1_AE;
	float SN1_WE1;
	float SN1_AE1;

	float SN1_WE_Real;    // NO2 WE
	float SN1_AE_Real;    // NO2 Aux
	float NO2;            // NO2 ppb

//	char str[15];

	//Calibration Constants
	// for SN1 or NO2
	float SN1_TOTAL_WE = (28.8*(-0.73));            //uit is nA*(mV/nA)
	float SN1_TOTAL_AE = (8*(-0.73));
	float SN1_SENSITIVITY = (-391.6*(-0.73));       //unit is nA/ppb*(mV/nA)

	float Temperature = (float) TEMP;
	float n_NO2;
	if(Temperature <= 10){
	 n_NO2=1.090;
	}
	else if(Temperature > 10 && Temperature < 30){
	 n_NO2=1.350;
	}
	else{
	 n_NO2=3.000;
	}

	ADC_Select_CH12();
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 100);
	SN1_WE = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
	SN1_WE1 = ((float)SN1_WE * 3300 / 1023);
	printf("NO2 WE Voltage: %.4f\r\n",SN1_WE1);


	ADC_Select_CH13();
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 100);
	SN1_AE = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
	SN1_AE1 = ((float)SN1_AE * 3300 / 1023);
	printf("NO2 AE Voltage: %.4f\r\n",SN1_AE1);

	SN1_WE_Real = SN1_WE1 - SN1_TOTAL_WE;
	SN1_AE_Real = n_NO2*(SN1_AE1 - SN1_TOTAL_AE);
	NO2 = (SN1_WE_Real - SN1_AE_Real)/(SN1_SENSITIVITY);


	//NO2_final = (uint16_t) NO2;

	return NO2;
}

/***************************************************************
 * float OX_Read (uint16_t TEMP)
 * for reading OX ug/m^3 concentration
 * PA0 -- WE_OX		--IN0
 * PA1 -- AE_OX		--IN1
 */

float OX_Read (uint16_t TEMP)
{
	uint16_t SN2_WE;
	uint16_t SN2_AE;
	float SN2_WE1;
	float SN2_AE1;
	float SN2_WE_Real;    // OX WE
	float SN2_AE_Real;    // OX Aux
	float OX;
//	uint16_t OX_final;
	//float OX;             // OX ppb
//	float OX_1;           // OX ug/m3

//	char str[15];

	//Calibration Constants
	// for SN2 or OX
	float SN2_TOTAL_WE = 10.2*(-0.73);
	float SN2_TOTAL_AE = 55*(-0.73);
	float SN2_SENSITIVITY = -504.1*(-0.73);       //unit in nA/ppb*(mV/nA)

	float Temperature = (float) TEMP;
	float n_OX;
	if(Temperature <= 10){
		n_OX=0.750;
	}
	else if(Temperature > 10 && Temperature < 30){
		n_OX=1.280;
	}
	else{
		n_OX=1.360;
	}


	ADC_Select_CH0();
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 100);
	SN2_WE = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
	SN2_WE1 = ((float)SN2_WE * 3300 / 1023);
	HAL_ADC_Stop(&hadc);
	printf("OX WE Voltage: %.4f\r\n",SN2_WE1);

	ADC_Select_CH1();
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 100);
	SN2_AE = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
	SN2_AE1 = ((float)SN2_AE * 3300 / 1023);
	printf("OX AE Voltage: %.4f\r\n",SN2_AE1);

	SN2_WE_Real = SN2_WE1 - SN2_TOTAL_WE;
	SN2_AE_Real = n_OX*(SN2_AE1 - SN2_TOTAL_AE);
	OX = ((SN2_WE_Real - SN2_AE_Real)/(SN2_SENSITIVITY));

	return OX;

}

/***************************************************************
 * float CO_Read (uint16_t TEMP)
 * for reading CO ug/m^3 concentration
 * PA2 -- WE_CO		--IN2
 * PA3 -- AE_CO		--IN3
 */

float CO_Read (uint16_t TEMP)
{
	uint16_t SN3_WE;
	uint16_t SN3_AE;
	float SN3_WE1;
	float SN3_AE1;
	float SN3_WE_Real;    // CO WE
	float SN3_AE_Real;    // CO Aux
	float CO;             // CO ppb
//	float CO_1;           // CO mg/m3
//	char str[15];

	//Calibration Constants
	// for SN3 or CO
	float SN3_TOTAL_WE = -39.6*(0.8);
	float SN3_TOTAL_AE = -5*(0.8);
	float SN3_SENSITIVITY = 335.3*(0.8);        //unit in nA/ppm*(mV/nA)

	float Temperature = (float) TEMP;
	float n_CO;
	if(Temperature <= 10){
		n_CO=1.000;
	}
	else if(Temperature > 10 && Temperature < 30){
		n_CO=-1.000;
	}
	else{
		n_CO=-0.766;
	}

	ADC_Select_CH2();
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 100);
	SN3_WE = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
	SN3_WE1 = ((float)SN3_WE * 3300 / 1023);
	printf("CO WE Voltage: %.4f\r\n",SN3_WE1);

	ADC_Select_CH3();
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 100);
	SN3_AE = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
	SN3_AE1 = ((float)SN3_AE * 3300 / 1023);
	printf("CO AE Voltage: %.4f\r\n",SN3_AE1);

	SN3_WE_Real = SN3_WE1 - SN3_TOTAL_WE;
	SN3_AE_Real = n_CO*(SN3_AE1 - SN3_TOTAL_AE);
	CO = (SN3_WE_Real - SN3_AE_Real)/(SN3_SENSITIVITY);

	return CO;

}


/***************************************************************
 * float SO2_Read (uint16_t TEMP)
 * for reading SO2 ug/m^3 concentration
 * PA4 -- WE_SO2	--IN4
 * PA5 -- AE_SO2	--IN5
 */
float SO2_Read (uint16_t TEMP)
{
	uint16_t SN4_WE;
	uint16_t SN4_AE;
	float SN4_WE1;
	float SN4_AE1;
	float SN4_WE_Real;    // SO2 WE
	float SN4_AE_Real;    // SO2 Aux
	float SO2;            // SO2 ppb
//	float SO2_1;          // SO2 ug/m3
//	char str[15];

	//Calibration Constants
	// for SN4 or SO2
	float SN4_TOTAL_WE = (-9.6*0.8);
	float SN4_TOTAL_AE = (20.3*0.8);
	float SN4_SENSITIVITY = (365.6*0.8);        //unit in nA/ppb*(mV/nA)

	float Temperature = (float) TEMP;
	float n_SO2;
	if(Temperature <= 10){
		n_SO2=1.150;
	}
	else if(Temperature > 10 && Temperature < 30){
		n_SO2=1.820;
	}
	else{
		n_SO2=3.930;
	}

	ADC_Select_CH4();
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 100);
	SN4_WE = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
	SN4_WE1 = ((float)SN4_WE * 3300 / 1023);
	printf("SO2 WE Voltage %.4f\r\n",SN4_WE1);

	ADC_Select_CH5();
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 100);
	SN4_AE = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
	SN4_AE1 = ((float)SN4_AE * 3300 / 1023);
	printf("SO2 AE Voltage: %.4f\r\n",SN4_AE1);

	SN4_WE_Real = SN4_WE1 - SN4_TOTAL_WE;
	SN4_AE_Real = n_SO2*(SN4_AE1 - SN4_TOTAL_AE);
	SO2 = (SN4_WE_Real - SN4_AE_Real)/(SN4_SENSITIVITY);

	return SO2;


}

/***************************************************************
 * void PM_Read(float *PM10, float *PM2_5)
 * for reading PM10 & PM2.5 concentration
 *  * PB0 -- PM_OUT	--IN8
 */

void PM_Read(float *PM1, float *PM2,float *PM3, float *PM4,float *PM5 )
{
	float S;
	float Val[8];
	float Vadc2;
	int bin;


	S=0;
	int P1=0;  int P2=0;  int P3=0;  int P4=0;  int P5=0;  int P6=0;  int P7=0; int P8=0; int P9=0; int P10=0;

	for(int i=1; i<=900; i++)        // This gives the Counts per sec
	{
		ADC_Select_CH8();
		HAL_ADC_Start(&hadc);
		HAL_ADC_PollForConversion(&hadc, 1000);
		float Vadc = HAL_ADC_GetValue(&hadc);

		float Voltage = ((Vadc/1023)*5000);     // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V) or (0-5000mV):
		if(i<=8)
		{
		 Val[i] = Voltage;
		 S = ((S*(i-1)) + Val[i])/i;
		}

		if(i>8){
		 for(int j=1; j<=7; j++){
		 Val[j]=Val[j+1];
		 }
		 Val[8] = Voltage;
		 S=(Val[1]+Val[2]+Val[3]+Val[4]+Val[5]+Val[6]+Val[7]+Val[8])/8;
		}

		Vadc2 = 1023*(S/1200);
		bin = (500*(Vadc2/1024));          // bin = 0 -- noise; 1<= bin <=3 -- PM1; 1<= bin <=9 -- PM2.5; 1<= bin <=10 -- PM10
		if (bin > 0)
		{
		if (bin < 10)  P1=P1+1;
		if (bin < 25)  P2=P2+1;
		if (bin < 50)  P3=P3+1;
		if (bin < 100)  P4=P4+1;  //threshold for pm2.5
		if (bin < 200)  P5=P5+1;
		//if (bin < 200)  P6=P6+1;
		//if (bin < 150)  P7=P7+1;
		//if (bin < 200)  P8=P8+1;
		//if (bin < 300)  P9=P9+1;
		//if (bin < 400)  P10=P10+1;
		}
		HAL_Delay(1);

	}

	*PM1 = P1;
	*PM2 = P2;
	*PM3 = P3;
	*PM4 = P4;
	*PM5 = P5;




	HAL_Delay(100);
}


