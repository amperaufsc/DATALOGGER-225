/**
  ******************************************************************************
  * @file           : buildconfig.h
  * @brief          : Configures building options
  ******************************************************************************
  */
#ifndef BUILDCONFIG_H
#define BUILDCONFIG_H

/* Mode Select -----------------------------------------------------------*/

#define WriteMode
//#define ReadMode

/* SD Card save modes -----------------------------------------------------------*/
#define CSV_mode
//#define Binary_mode

/* Control types  -----------------------------------------------------------*/
#ifdef WriteMode
#ifdef CSV_mode
#define CSV_write
#endif
#ifdef Binary_mode
#define Binary_write
#endif
#endif

#ifdef ReadMode
#ifdef CSV_mode
#define CSV_read
#endif
#ifdef Binary_mode
#define Binary_read
#endif
#endif

#endif
