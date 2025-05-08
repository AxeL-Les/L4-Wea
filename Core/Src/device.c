/******************************************************************************
 * @file    device.c
 * @author  Akshit Bhangale
 * @date    05/01/2025
 * @brief   Device Initialization and Display Interface for STM32
 *
 * This file implements hardware abstraction and utility functions for
 * initializing and operating the 2.13" e-paper display (EPD) and the
 * Bosch BME280 environmental sensor. It provides user-friendly functions
 * for formatted output of sensor readings (temperature, pressure, humidity,
 * and altitude) onto the EPD display.
 *
 * The BME280 sensor is initialized via I2C using the STM32 HAL interface,
 * and the display output is managed through GUI_Paint functions.
 *
 * Features:
 * - Centralized device initialization (`Device_Init`)
 * - Font color mode control for display text
 * - Helper functions for rendering environmental sensor values
 *
 * Dependencies:
 * - bme280.h       : Sensor driver
 * - EPD_2in13_V4.h : E-paper driver
 * - GUI_Paint.h    : Graphics rendering
 * - Debug.h        : Logging/debug output
 * - i2c.h          : I2C handle for BME280
 * - math.h         : Altitude calculation
 * - stdlib.h       : Memory allocation for display buffer
 *
 * @warning This module is still in the testing phase and has not been
 *          fully validated. It may require refinement before production use.
 *
 * License:
 * This implementation is intended for educational and prototyping use.
 * Modifications are permitted as per your project requirements.
 ******************************************************************************/
#include "device.h"

UBYTE *BlackImage = NULL;
bme280_t bme; // Global BME280 sensor object

void Device_Init(UBYTE rotation, UBYTE background_color)
{
    /*** Initialize E-Paper ***/
    UWORD Imagesize = ((EPD_2in13_V4_WIDTH % 8 == 0) ?
                      (EPD_2in13_V4_WIDTH / 8) :
                      (EPD_2in13_V4_WIDTH / 8 + 1)) * EPD_2in13_V4_HEIGHT;

    BlackImage = (UBYTE *)malloc(Imagesize);
    if (BlackImage == NULL) {
        Debug("Device_Init: Failed to allocate BlackImage!\r\n");
        return;
    }

    Paint_NewImage(BlackImage, EPD_2in13_V4_WIDTH, EPD_2in13_V4_HEIGHT, rotation, background_color);
    Paint_SelectImage(BlackImage);
    Paint_Clear(background_color);

    EPD_2in13_V4_Init();
    EPD_2in13_V4_Display_Base(BlackImage);
    EPD_2in13_V4_Init_Fast();

    /*** Initialize BME280 ***/
    BME280_Init(&bme, &hi2c1, BME280_ADDRESS); // Use your hi2c1
}

void EPD_Print(int x, int y, const char* text,sFONT* font, FontColorMode_t mode)
{
    if (mode == FONT_BLACK_ON_WHITE) {
        Paint_DrawString_EN(x, y, (char*)text, font, WHITE, BLACK);
    } else {
        Paint_DrawString_EN(x, y, (char*)text, font, BLACK, WHITE);
    }
    HAL_Delay(500);
}

void EPD_PrintTemperature(bme280_t *bme, int x, int y, FontColorMode_t mode)
{
    char buffer[32];
    float temp = BME280_ReadTemperature(bme);
    sprintf(buffer, "T:%.1f C", temp);
    EPD_Print(x, y, buffer,&Font16, mode);
}

void EPD_PrintPressure(bme280_t *bme, int x, int y, FontColorMode_t mode)
{
    char buffer[32];
    float press = BME280_ReadPressure(bme);
    sprintf(buffer, "P:%.1f hPa", press);
    EPD_Print(x, y, buffer,&Font16, mode);
}

void EPD_PrintHumidity(bme280_t *bme, int x, int y, FontColorMode_t mode)
{
    char buffer[32];
    float hum = BME280_ReadHumidity(bme);
    sprintf(buffer, "H:%.1f %%", hum);
    EPD_Print(x, y, buffer,&Font16, mode);
}

void EPD_PrintAltitude(bme280_t *bme, float seaLevelhPa, int x, int y, FontColorMode_t mode)
{
    char buffer[32];
    float pressure = BME280_ReadPressure(bme);
    float altitude = 44330.0 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));
    sprintf(buffer, "A:%.1f m", altitude);
    EPD_Print(x, y, buffer,&Font16, mode);
}
