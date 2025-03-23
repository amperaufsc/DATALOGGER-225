/**
  ******************************************************************************
  * @file           : Sensors.c
  * @brief          : Code for dealing with the manipulations of information
  * 				  transmitted by ADC.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Sensors.h"
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
float ReadAngleSTR(float voltage){
    float interval = (VOL_ANGLE_MAX - VOL_ANGLE_MIN) / 3.3f;
    float Angle = (voltage - V_CENTER) * interval;
    float AngleSTR = Angle/(STR_RATIO); // Divide pelo Steering Ratio, a cada 4.35 graus a roda gira 1 grau
    return AngleSTR;
}
