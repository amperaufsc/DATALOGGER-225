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
float ReadAngleSTR(float voltage) {

    float str_angle = ((voltage - V_CENTER) / (V_MAX / 2)) * VOL_ANGLE_MAX;
    return str_angle;
}
float ReadPressure(uint16_t Analog_Val){
    float voltage = ReadVoltage(Analog_Val);
    return ((voltage / 3.3f) * 102.0f) * 14.5038f;
}
