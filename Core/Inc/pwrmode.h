/******************************************************************************
 * @file    pwrmode.h
 * @author  Akshit Bhangale
 * @date    04/28/2025
 * @brief   Power Mode Management Interface for STM32L4
 *
 * This header defines the interface for managing power operation modes
 * on an STM32L4-based system. It includes functionality for automatic or
 * user-input-based control over power states, wake-up handling, shutdown
 * transitions, and LED/display updates related to power management.
 *
 * The module supports interaction with GPIO components such as LEDs and
 * user buttons and is designed for integration with low-power features
 * via the STM32 HAL Power and Power EX APIs.
 *
 * @warning This library is still under development and has not yet been
 *          fully tested. It is not recommended for use in production-level
 *          implementations until further validation is completed.
 *
 * Dependencies:
 * - stm32l4xx_hal.h
 * - stm32l4xx_hal_pwr.h
 * - stm32l4xx_hal_pwr_ex.h
 * - gpio.h (for user button and LED macros)
 * - device.h (application-specific functionality)
 *
 * License:
 * This file is provided for prototyping and evaluation purposes.
 * Adapt and validate as required for your specific application.
 ******************************************************************************/
#ifndef __PWRMODE_H
#define __PWRMODE_H

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_pwr.h"
#include "stm32l4xx_hal_pwr_ex.h"
#include "gpio.h" // For LD2_Pin and USER_Btn_Pin macros
#include "device.h"

// Power operation modes
typedef enum {
    AUTO_MODE = 0,
    USER_INPUT_MODE
} PWRMode_t;

// Global power mode variable
extern PWRMode_t pwrmode;

// Public function prototypes
void PWRMode_Init(void);
void PWRMode_HandleWakeup(void);
void PWRMode_UpdateLED(void);
void PWRMode_SampleAndDisplay(void);
void PWRMode_UserModeDisplay(void);
void PWRMode_EnterShutdown(void);

#endif /* __PWRMODE_H */
