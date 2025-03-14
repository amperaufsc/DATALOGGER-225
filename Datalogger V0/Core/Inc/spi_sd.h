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
#include "main.h"
#include "fatfs.h"
#include "fatfs_sd.h"
#include "String.h"
#include "stdio.h"
/* Defines ------------------------------------------------------------------*/
#define BUFFERSIZE 100

/* Exported functions --	------------------------------------------------------*/
void save_to_buffer(float position);
void sd_writeBin(float value);
void sd_writeCSV(float value);
void bufclear (void);

#endif
