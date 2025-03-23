/**
  ******************************************************************************
  * @file           : adc_multi.h
  * @brief          : Header for adc_multi.h file.
  *                   This file contains prototypes for functions that deals with
  *                   the manipulations of information transmitted by ADC.
  ******************************************************************************
  */

#ifndef INC_ADC_MULTI_H_
#define INC_ADC_MULTI_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Defines declaration -------------------------------------------------------*/
#define READSIZE 1 // Amount of sensor to be read
#define ANGLE_MIN 20 // Angle when pedal is pressed
#define ANGLE_MAX 90 // Angle when pedal is inertial

/* Exported functions --	------------------------------------------------------*/
float ReadVoltage(uint16_t Analog_Val);
void CleanADC_VAL(float* ADC_VAL );
float ReadPosition(float voltage);
float ConvertAngle(float Angle);
#endif /* INC_ADC_MULTI_H_ */
