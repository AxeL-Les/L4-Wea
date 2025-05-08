/******************************************************************************
 * @file    bme280.h
 * @author  Akshit Bhangale
 * @date    04/27/2025
 * @brief   BME280 Sensor Driver Header for STM32 HAL I2C Interface
 *
 * This header defines the structures and function prototypes for interacting
 * with the Bosch BME280 environmental sensor. It enables temperature, pressure,
 * and humidity measurements using an STM32 microcontroller via I2C, and provides
 * the necessary calibration data handling as specified by the sensor datasheet.
 *
 * The driver is tailored for STM32 using the HAL library, but can be adapted
 * for other platforms with compatible I2C support.
 *
 * @note    This code is inspired by the Bosch Sensortec BME280 Sensor API and
 *          implements similar compensation algorithms for environmental readings.
 *
 * References:
 * - Bosch Sensortec BME280 Datasheet:
 *   https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
 * - Bosch BME280 Sensor API (GitHub):
 *   https://github.com/BoschSensortec/BME280_SensorAPI
 *
 * License:
 * This file is provided for educational and prototyping use.
 * If your implementation derives from or includes portions of Bosch’s code,
 * please ensure compatibility with their license terms as specified in their repository.
 ******************************************************************************/

#ifndef __BME280_H__
#define __BME280_H__

#include "stm32l4xx_hal.h"  // adjust for your STM32 series

typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
} bme280_calib_data_t;

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint8_t address;
    int32_t t_fine;
    bme280_calib_data_t calib;
} bme280_t;

void BME280_Init(bme280_t *dev, I2C_HandleTypeDef *hi2c, uint8_t address);
float BME280_ReadTemperature(bme280_t *dev);
float BME280_ReadPressure(bme280_t *dev);
float BME280_ReadHumidity(bme280_t *dev);

#endif /* __BME280_H__ */
