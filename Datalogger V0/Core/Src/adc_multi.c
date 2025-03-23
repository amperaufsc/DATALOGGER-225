/**
  ******************************************************************************
  * @file           : adc_multi.c
  * @brief          : Code for dealing with the manipulations of information
  * 				  transmitted by ADC.
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc_multi.h"
#include "math.h"

/* Exported functions --------------------------------------------------------*/
float ReadPosition(float voltage){
	float position = (voltage / 3.3f)* 75.0f; // Converte Tensão em Milimetros
	return position;
}

float ReadVoltage(uint16_t Analog_Val) {
    float voltage = (Analog_Val * 3.3f) / ((1 << 12) - 1.0f); // Converte um valor analogico para tensão
    return voltage;
}
float ReadAngle(int Pedal_position){
	float interval = (ANGLE_MAX - ANGLE_MIN)/ 100.0;
	float Angle = (interval*Pedal_position) + ANGLE_MIN;
	return Angle;
}
void CleanADC_VAL(float* ADC_VAL ){
	for (int i = 0; i < READSIZE; i++) {
	    ADC_VAL[i] = 0;
	}
}
