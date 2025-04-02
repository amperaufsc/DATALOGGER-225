/**
  ******************************************************************************
  * @file           : Sensors.h
  * @brief          : Header for Sensors.h file.
  *                   This file contains prototypes for functions that deals with
  *                   the manipulations of information transmitted by ADC.
  ******************************************************************************
  */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Defines declaration -------------------------------------------------------*/
#define READSIZE 7 // Numeros de Sensores a serem lidos
#define ANGLE_MIN 20 // Angulo Minimo do pedal
#define ANGLE_MAX 90 // Angulo Maximo do pedal
#define VOL_ANGLE_MIN -32.0  // Ângulo mínimo (volante todo para a esquerda)
#define VOL_ANGLE_MAX  32.0  // Ângulo máximo (volante todo para a direita)
#define V_CENTER 1.65    // Tensão no centro do volante
#define STR_RATIO 4.35 // Para cada 4.35 graus de giro no volante a roda gira 1 grau
#define V_MAX 3.3      // Tensão máxima do potenciômetro
/* Exported functions --	------------------------------------------------------*/
float ReadVoltage(uint16_t Analog_Val);
void CleanADC_VAL(float* ADC_VAL );
float ReadPosition(float voltage);
float ReadAngle(int Pedal_position);
float ReadAngleSTR(float voltage);
float ReadPressure(uint16_t Analog_Val);
#endif /* INC_SENSORS_H_ */
