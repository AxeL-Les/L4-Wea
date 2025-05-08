
/******************************************************************************
 * @file    bme280.c
 * @author  Akshit Bhangale
 * @date    04/27/2025
 * @brief   BME280 Driver for STM32 using HAL I2C interface
 *
 * This driver provides initialization and data reading functions for the
 * Bosch BME280 sensor over I2C. It supports reading temperature, pressure,
 * and humidity values and compensates them using calibration coefficients
 * stored in the sensor, as specified in the BME280 datasheet.
 *
 * @note    This code is adapted for STM32 microcontrollers using the HAL library.
 *          While it is not a direct copy, it is based on the Bosch Sensortec
 *          BME280 Sensor API reference implementation and compensation formulas.
 *
 * References:
 * - Bosch Sensortec BME280 Datasheet:
 *   https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
 * - Bosch BME280 Sensor API (GitHub):
 *   https://github.com/BoschSensortec/BME280_SensorAPI
 *
 * License:
 * This implementation is provided for educational and prototyping purposes.
 * You may modify and redistribute this code in accordance with your project
 * licensing terms. If derived from Bosch's official driver, retain Bosch's
 * original license where applicable.
 ******************************************************************************/

#include "bme280.h"

#define BME280_REG_ID          0xD0
#define BME280_REG_RESET       0xE0
#define BME280_REG_CTRL_HUM    0xF2
#define BME280_REG_CTRL_MEAS   0xF4
#define BME280_REG_CONFIG      0xF5

static uint8_t BME280_Read8(bme280_t *dev, uint8_t reg);
static uint16_t BME280_Read16_LE(bme280_t *dev, uint8_t reg);
static int16_t BME280_ReadS16_LE(bme280_t *dev, uint8_t reg);
static uint32_t BME280_Read24(bme280_t *dev, uint8_t reg);
static void BME280_ReadCoefficients(bme280_t *dev);

void BME280_Init(bme280_t *dev, I2C_HandleTypeDef *hi2c, uint8_t address)
{
    dev->i2c = hi2c;
    dev->address = address;

    uint8_t id = BME280_Read8(dev, BME280_REG_ID);
    if (id != 0x60) {
        return; // wrong chip
    }

    uint8_t reset = 0xB6;
    HAL_I2C_Mem_Write(dev->i2c, dev->address, BME280_REG_RESET, 1, &reset, 1, HAL_MAX_DELAY);
    HAL_Delay(100);

    BME280_ReadCoefficients(dev);

    uint8_t ctrl_hum = 0x01; // humidity oversampling x1
    HAL_I2C_Mem_Write(dev->i2c, dev->address, BME280_REG_CTRL_HUM, 1, &ctrl_hum, 1, HAL_MAX_DELAY);

    uint8_t ctrl_meas = 0x27; // temp and pressure oversampling x1, normal mode
    HAL_I2C_Mem_Write(dev->i2c, dev->address, BME280_REG_CTRL_MEAS, 1, &ctrl_meas, 1, HAL_MAX_DELAY);

    uint8_t config = 0xA0;
    HAL_I2C_Mem_Write(dev->i2c, dev->address, BME280_REG_CONFIG, 1, &config, 1, HAL_MAX_DELAY);
}

float BME280_ReadTemperature(bme280_t *dev)
{
    int32_t var1, var2;
    uint8_t data[3];

    HAL_I2C_Mem_Read(dev->i2c, dev->address, 0xFA, 1, data, 3, HAL_MAX_DELAY);
    int32_t adc_T = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4);

    var1 = ((((adc_T >> 3) - ((int32_t)dev->calib.dig_T1 << 1))) * ((int32_t)dev->calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dev->calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)dev->calib.dig_T1))) >> 12) *
            ((int32_t)dev->calib.dig_T3)) >> 14;

    dev->t_fine = var1 + var2;

    int32_t T = (dev->t_fine * 5 + 128) >> 8;
    return T / 100.0f;
}

float BME280_ReadPressure(bme280_t *dev)
{
    int64_t var1, var2, p;
    uint8_t data[3];

    HAL_I2C_Mem_Read(dev->i2c, dev->address, 0xF7, 1, data, 3, HAL_MAX_DELAY);
    int32_t adc_P = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4);

    var1 = ((int64_t)dev->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dev->calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dev->calib.dig_P3) >> 8) + ((var1 * (int64_t)dev->calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dev->calib.dig_P1) >> 33;

    if (var1 == 0) return 0; // avoid division by zero

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dev->calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dev->calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)dev->calib.dig_P7) << 4);
    return (float)p / 25600.0f;
}

float BME280_ReadHumidity(bme280_t *dev)
{
    int32_t adc_H;
    uint8_t data[2];

    HAL_I2C_Mem_Read(dev->i2c, dev->address, 0xFD, 1, data, 2, HAL_MAX_DELAY);
    adc_H = (int32_t)((data[0] << 8) | data[1]);

    int32_t v_x1_u32r;
    v_x1_u32r = (dev->t_fine - ((int32_t)76800));

    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dev->calib.dig_H4) << 20) -
            (((int32_t)dev->calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
            (((((((v_x1_u32r * ((int32_t)dev->calib.dig_H6)) >> 10) *
            (((v_x1_u32r * ((int32_t)dev->calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
            ((int32_t)2097152)) * ((int32_t)dev->calib.dig_H2) + 8192) >> 14));

    v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dev->calib.dig_H1)) >> 4);
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    float h = (v_x1_u32r >> 12);
    return h / 1024.0f;
}

static uint8_t BME280_Read8(bme280_t *dev, uint8_t reg)
{
    uint8_t value;
    HAL_I2C_Mem_Read(dev->i2c, dev->address, reg, 1, &value, 1, HAL_MAX_DELAY);
    return value;
}

static uint16_t BME280_Read16_LE(bme280_t *dev, uint8_t reg)
{
    uint8_t data[2];
    HAL_I2C_Mem_Read(dev->i2c, dev->address, reg, 1, data, 2, HAL_MAX_DELAY);
    return (data[1] << 8) | data[0];
}

static int16_t BME280_ReadS16_LE(bme280_t *dev, uint8_t reg)
{
    return (int16_t)BME280_Read16_LE(dev, reg);
}

static uint32_t BME280_Read24(bme280_t *dev, uint8_t reg)
{
    uint8_t data[3];
    HAL_I2C_Mem_Read(dev->i2c, dev->address, reg, 1, data, 3, HAL_MAX_DELAY);
    return ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | (uint32_t)data[2];
}

static void BME280_ReadCoefficients(bme280_t *dev)
{
    dev->calib.dig_T1 = BME280_Read16_LE(dev, 0x88);
    dev->calib.dig_T2 = BME280_ReadS16_LE(dev, 0x8A);
    dev->calib.dig_T3 = BME280_ReadS16_LE(dev, 0x8C);

    dev->calib.dig_P1 = BME280_Read16_LE(dev, 0x8E);
    dev->calib.dig_P2 = BME280_ReadS16_LE(dev, 0x90);
    dev->calib.dig_P3 = BME280_ReadS16_LE(dev, 0x92);
    dev->calib.dig_P4 = BME280_ReadS16_LE(dev, 0x94);
    dev->calib.dig_P5 = BME280_ReadS16_LE(dev, 0x96);
    dev->calib.dig_P6 = BME280_ReadS16_LE(dev, 0x98);
    dev->calib.dig_P7 = BME280_ReadS16_LE(dev, 0x9A);
    dev->calib.dig_P8 = BME280_ReadS16_LE(dev, 0x9C);
    dev->calib.dig_P9 = BME280_ReadS16_LE(dev, 0x9E);

    dev->calib.dig_H1 = BME280_Read8(dev, 0xA1);
    dev->calib.dig_H2 = BME280_ReadS16_LE(dev, 0xE1);
    dev->calib.dig_H3 = BME280_Read8(dev, 0xE3);

    uint8_t e4, e5, e6;
    HAL_I2C_Mem_Read(dev->i2c, dev->address, 0xE4, 1, &e4, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(dev->i2c, dev->address, 0xE5, 1, &e5, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(dev->i2c, dev->address, 0xE6, 1, &e6, 1, HAL_MAX_DELAY);

    dev->calib.dig_H4 = (int16_t)((e4 << 4) | (e5 & 0x0F));
    dev->calib.dig_H5 = (int16_t)((e6 << 4) | (e5 >> 4));
    dev->calib.dig_H6 = (int8_t)BME280_Read8(dev, 0xE7);
}
