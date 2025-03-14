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

/* Variables declaration -----------------------------------------------------*/

FATFS fs;// File System
FIL fil; // File
FRESULT fresult; // Store the result
UINT br,bw; //File read/write count
char buffer[BUFFERSIZE];
int buffercount = 0;
int TAM = BUFFERSIZE;
float Buffer1[BUFFERSIZE] = {0};
/* Exported functions --------------------------------------------------------*/
void save_to_buffer(float position){

	for(buffercount = 0;buffercount <= BUFFERSIZE - 1; buffercount++){
	Buffer1[buffercount] = position;
	}
		}
void sd_writeCSV(float value) {
/* Mount SD Card -------------------------------------------------------------*/
		    fresult = f_mount(&fs, "", 1);
		    if (fresult != FR_OK) {
		        return;
		    }

/* Open/Create file ----------------------------------------------------------*/
		    fresult = f_open(&fil, "DadosFSAE.csv", FA_OPEN_ALWAYS | FA_WRITE);
		    if (fresult != FR_OK) {
		        return;
		    }

/* Move pointer to end of file -----------------------------------------------*/
		f_lseek(&fil, f_size(&fil));
/* Float to String Conversion ------------------------------------------------*/
		sprintf(buffer, "%.2f,\n", value);

/* Write data to file --------------------------------------------------------*/
	    fresult = f_write(&fil, buffer, strlen(buffer), &bw);
	    if (fresult != FR_OK) {
	        return;
	    }

/* Close file ----------------------------------------------------------------*/
	    f_close(&fil);
	    bufclear();
	}

void sd_writeBin(float value){
/* Mount SD Card -------------------------------------------------------------*/
	    fresult = f_mount(&fs, "", 1);
	    if (fresult != FR_OK) {
	        return;
	    }

/* Open/Create file ----------------------------------------------------------*/
	    fresult = f_open(&fil, "DadosFSAE.bin", FA_OPEN_ALWAYS | FA_WRITE);
	    if (fresult != FR_OK) {
	        return;
	    }

/* Move pointer to end of file -----------------------------------------------*/
	f_lseek(&fil, f_size(&fil));

/* Write data to file --------------------------------------------------------*/
		fresult = f_write(&fil, &value, sizeof(float), &bw);
		if (fresult == FR_OK) {
			return;
		}
/* Close file ----------------------------------------------------------------*/
	    f_close(&fil);
	    bufclear();
	}

void bufclear (void){
/* Clean Buffer --------------------------------------------------------------*/
	for(int i = 0; i < TAM; i++){
		buffer[i] = '\0';
	}
}
