/**
  ******************************************************************************
  * @file           : buildconfig.h
  * @brief          : Configures building options
  ******************************************************************************
  */
#ifndef BUILDCONFIG_H
#define BUILDCONFIG_H

/* Mode Select -----------------------------------------------------------*/

#define WRITEMODE
//#define READMODE

/* SD Card save modes -----------------------------------------------------------*/
#define CSV_MODE
//#define BINARY_MODE

/* Control types  -----------------------------------------------------------*/
#ifdef WRITEMODE
#ifdef CSV_MODE
#define CSV_WRITE
#endif
#ifdef BINARY_MODE
#define BINARY_WRITE
#endif
#endif

#ifdef READMODE
#ifdef CSV_MODE
#define CSV_READ
#endif
#ifdef BINARY_MODE
#define BINARY_READ
#endif
#endif

#endif
