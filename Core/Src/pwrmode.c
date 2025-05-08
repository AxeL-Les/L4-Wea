/******************************************************************************
 * @file    pwrmode.c
 * @author  Akshit Bhangale
 * @date    04/28/2025
 * @brief   Power Mode Control Implementation for STM32L4
 *
 * This source file implements functions for managing power modes on an
 * STM32L4 microcontroller, specifically using the STM32 HAL Power APIs.
 * It includes support for automatic and user-input-based power mode
 * selection, wake-up handling, shutdown entry, and environment data
 * sampling using a BME280 sensor with display updates.
 *
 * Features:
 * - Wake-up cause detection (RTC vs. user button)
 * - Low-power data sampling with backup SRAM logging
 * - Live data display (temperature, pressure, humidity, altitude)
 * - Power-efficient shutdown transitions
 *
 * @warning This library is still under development and has not been fully
 *          tested. It may contain incomplete logic or unverified behavior,
 *          and is not recommended for production use without further validation.
 *
 * Dependencies:
 * - STM32 HAL (stm32l4xx_hal_pwr, pwr_ex, etc.)
 * - gpio.h for LED/button macros
 * - device.h for BME280 handle
 * - EPD driver for display output (EPD_Print, Paint_Clear, etc.)
 *
 * License:
 * This file is intended for research, learning, and prototyping purposes.
 * You may modify and integrate it in line with your project’s licensing terms.
 ******************************************************************************/
#include "pwrmode.h"

PWRMode_t pwrmode = AUTO_MODE;

void PWRMode_Init(void)
{
    // Enable Wakeup Pin 2 (PC13 = USER_Btn) on falling edge (button press = LOW)
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_LOW);

    // Clear any previous wakeup flags
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
}

void PWRMode_HandleWakeup(void)
{
    // Detect cause of wakeup
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_WUF2)) // Wakeup from PC13 (USER_Btn)
    {
        pwrmode = USER_INPUT_MODE;
    }
    else
    {
        pwrmode = AUTO_MODE; // Default: RTC wakeup
    }

    // Always clear wakeup flags after reading
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
}

void PWRMode_UpdateLED(void)
{
    if (pwrmode == USER_INPUT_MODE)
    {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // LD2 ON (active-low)
    }
    else
    {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);   // LD2 OFF
    }
}

void PWRMode_SampleAndDisplay(void)
{
  /* USER CODE BEGIN AUTO_MODE_INIT */
  extern bme280_t bme;                       /* sensor handle from device.c   */
  volatile uint32_t *bkp = (uint32_t *)0x40024000; /* BKPSRAM base            */
  const float seaLevel = 1013.25f;                /* hPa                       */

  uint32_t sumPa = 0;

  /* USER CODE END AUTO_MODE_INIT */

  for (uint8_t s = 0; s < 10; ++s)           /* 10 samples → 1 min           */
  {
    /* USER CODE BEGIN AUTO_MODE_LOOP */

    /* 1. Read sensor */
    float temp = BME280_ReadTemperature(&bme);     /* °C  */
    float press = BME280_ReadPressure(&bme);       /* hPa */
    float hum   = BME280_ReadHumidity(&bme);       /* %   */
    float alt   = 44330.0f * (1.0f - powf(press / seaLevel, 0.1903f));

    sumPa += (uint32_t)(press * 100);              /* accumulate in Pa         */
    bkp[s] = (uint32_t)(press * 100);              /* store raw in BKPSRAM     */

    /* 2. Update display live (1 FPS) */
    Paint_Clear(WHITE);
    char buf[32];

    sprintf(buf, "T: %.1f C", temp);
    EPD_Print(2, 10, buf,&Font16, FONT_BLACK_ON_WHITE);

    sprintf(buf, "P: %.2f hPa", press);
    EPD_Print(2, 30, buf,&Font16, FONT_BLACK_ON_WHITE);

    sprintf(buf, "H: %.1f %%", hum);
    EPD_Print(2, 50, buf,&Font16, FONT_BLACK_ON_WHITE);

    sprintf(buf, "A: %.1f m", alt);
    EPD_Print(2, 70, buf,&Font16, FONT_BLACK_ON_WHITE);

    EPD_2in13_V4_Display_Fast(BlackImage);

    /* wait 6 s before next sample */
    HAL_Delay(6000);

    /* USER CODE END AUTO_MODE_LOOP */
  }

  /* USER CODE BEGIN AUTO_MODE_AVG */

  /* Hourly average pressure (Pa) → store in BKPSRAM word 16 .. etc. */
  uint32_t avgPa = sumPa / 10u;
  bkp[16] = avgPa;          /* example: store average after raw block */

  /* Optional: flash write or other hourly tasks here */

  /* USER CODE END AUTO_MODE_AVG */
}

void PWRMode_UserModeDisplay(void)
{
    // USER_INPUT_MODE: Refresh screen 1FPS for 1 minute
    for (uint8_t i = 0; i < 60; i++) // 60 seconds
    {
        // Refresh display or data
        HAL_Delay(1000); // Wait 1 second
    }
}

void PWRMode_EnterShutdown(void)
{
    // Re-enable Wakeup Pin before entering Shutdown
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_LOW);

    // Clear any wakeup flags
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

    // Enter Shutdown mode
    HAL_PWREx_EnterSHUTDOWNMode();
}
