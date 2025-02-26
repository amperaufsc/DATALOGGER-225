/**
  ******************************************************************************
  * @file           : spi_sd.h
  * @brief          : Header for spi_sd.c file.
  *                   This file contains prototypes for functions that deals with
  *                   the manipulations of information transmitted by SPI.
  ******************************************************************************
  */
#ifndef SPI_SD_H
#define SPI_SD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "fatfs_sd.h"
#include "String.h"
#include "stdio.h"

/* Exported functions --	------------------------------------------------------*/
void send_uart(char *string);
void sd_writeBin(float value);
void sd_writeCSV(float value);
void bufclear (void);

#endif
