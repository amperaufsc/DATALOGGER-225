/**
  ******************************************************************************
  * @file           : spi_sd.c
  * @brief          : Code for dealing with the manipulations of information
  * 				  transmitted by SPI.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "spi_sd.h"
#include "fatfs_sd.h"
#include "String.h"
#include "stdio.h"

/* Variables declaration -----------------------------------------------------*/

FATFS fs;// File System
FIL fil; // File
FRESULT fresult; // Store the result
UINT br,bw; //File read/write count
#define Buffersize 50
char buffer[Buffersize];

/* Exported functions --------------------------------------------------------*/

void sd_writeCSV(float value) {
/* Mount SD Card -------------------------------------------------------------*/
		    fresult = f_mount(&fs, "", 1);
		    if (fresult != FR_OK) {
		        send_uart("Erro ao montar o cartão SD.\n");
		        return;
		    }

/* Open/Create file ----------------------------------------------------------*/
		    fresult = f_open(&fil, "DadosFSAE.csv", FA_OPEN_ALWAYS | FA_WRITE);
		    if (fresult != FR_OK) {
		        send_uart("Erro ao abrir o arquivo.\n");
		        return;
		    }

/* Move pointer to end of file -----------------------------------------------*/
		f_lseek(&fil, f_size(&fil));
/* Float to String Conversion ------------------------------------------------*/
		sprintf(buffer, "%.2f,\n", value);

/* Write data to file --------------------------------------------------------*/
	    fresult = f_write(&fil, buffer, strlen(buffer), &bw);
	    if (fresult == FR_OK) {
	        send_uart("Valor salvo no SD em CSV!\n");
	    } else {
	        send_uart("Erro ao escrever no CSV!\n");
	    }

/* Close file ----------------------------------------------------------------*/
	    f_close(&fil);
	    bufclear();
	}

void sd_writeBin(float value){
/* Mount SD Card -------------------------------------------------------------*/
	    fresult = f_mount(&fs, "", 1);
	    if (fresult != FR_OK) {
	        send_uart("Erro ao montar o cartão SD.\n");
	        return;
	    }

/* Open/Create file ----------------------------------------------------------*/
	    fresult = f_open(&fil, "DadosFSAE.bin", FA_OPEN_ALWAYS | FA_WRITE);
	    if (fresult != FR_OK) {
	        send_uart("Erro ao abrir o arquivo.\n");
	        return;
	    }

/* Move pointer to end of file -----------------------------------------------*/
	f_lseek(&fil, f_size(&fil));

/* Write data to file --------------------------------------------------------*/
		fresult = f_write(&fil, &value, sizeof(float), &bw);
		if (fresult == FR_OK) {
			send_uart("Valor salvo no SD em binário!\n");
		} else {
			send_uart("Erro ao escrever no SD!\n");
		}

/* Close file ----------------------------------------------------------------*/
	    f_close(&fil);
	    bufclear();
	}

void bufclear (void){
/* Clean Buffer --------------------------------------------------------------*/
	for(int i = 0; i < Buffersize; i++){
		buffer[i] = '\0';
	}
}
