/******************************************************************************
 * @file    device.h
 * @author  Akshit Bhangale
 * @date    05/01/2025
 * @brief   Device Abstraction Layer for EPD Display and BME280 Sensor
 *
 * This header defines the interface for initializing and interacting with
 * peripheral components in the system, specifically the BME280 environmental
 * sensor and the 2.13" e-paper display (EPD). It includes display utility
 * functions for rendering sensor data such as temperature, pressure, humidity,
 * and altitude, as well as a general-purpose text print function.
 *
 * Features:
 * - Unified initialization for sensor and display
 * - High-level API for formatted data visualization
 * - Font and color mode abstraction for easier display handling
 *
 * Dependencies:
 * - bme280.h: Environmental sensor driver
 * - EPD_2in13_V4.h and GUI_Paint.h: E-paper display driver and graphics library
 * - Debug.h: Optional debug utilities
 * - i2c.h: STM32 HAL I2C interface
 * - Standard libraries: math.h, stdlib.h
 *
 * @note This file is part of a custom embedded application and may require
 *       additional configuration or platform-specific adaptation.
 *
 * @warning This module is under development and has not yet undergone full
 *          validation. Further testing is required before use in final systems.
 *
 * License:
 * This file is intended for academic, research, and prototyping use.
 * Users are free to modify and distribute under their own project license.
 ******************************************************************************/
#ifndef __DEVICE_H__
#define __DEVICE_H__

#include "bme280.h"
#include "EPD_2in13_V4.h"
#include "GUI_Paint.h"
#include "Debug.h"
#include "i2c.h"
#include <math.h>
#include <stdlib.h>

/* --- Global Variables --- */
extern UBYTE *BlackImage;
extern bme280_t bme;
#define BME280_ADDRESS  (0x76 << 1)
/* --- Font Color Modes --- */
typedef enum {
    FONT_BLACK_ON_WHITE,
    FONT_WHITE_ON_BLACK
} FontColorMode_t;

/* --- Functions --- */

/**
 * @brief Initialize EPD and BME280
 * @param rotation Rotation (0, 90, 180, 270 degrees)
 * @param background_color WHITE or BLACK
 */
void Device_Init(UBYTE rotation, UBYTE background_color);

/**
 * @brief General text print function.
 */
void EPD_Print(int x, int y, const char* text,sFONT* font, FontColorMode_t mode);

/**
 * @brief Specialized functions to print sensor data.
 */
void EPD_PrintTemperature(bme280_t *bme, int x, int y, FontColorMode_t mode);
void EPD_PrintPressure(bme280_t *bme, int x, int y, FontColorMode_t mode);
void EPD_PrintHumidity(bme280_t *bme, int x, int y, FontColorMode_t mode);
void EPD_PrintAltitude(bme280_t *bme, float seaLevelhPa, int x, int y, FontColorMode_t mode);

#endif /* __DEVICE_H__ */
