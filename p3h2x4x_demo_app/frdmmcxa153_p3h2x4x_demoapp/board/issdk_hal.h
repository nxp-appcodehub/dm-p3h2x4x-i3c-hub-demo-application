/*
 * Copyright 2022, 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file issdk_hal.h
 * @brief Wrapper for Hardware Abstraction Layer (HAL)

    This file simply provides one level of indirection for the developer
    to select the particular Hardware Abstraction Layer they would like to use.
*/

#ifndef __ISSDK_HAL_H__
#define __ISSDK_HAL_H__

#include <frdm_stbi_p3h2840_shield.h> //Include appropriate sensor shield board header file
#include "fsl_i3c_cmsis.h"

#include "frdmmcxa153.h"   //Include appropriate MCU board header file

// Pin mapping and driver information for default I2C brought to shield

#define I3C_S_SCL_PIN      I3C_S1_SCL_PIN
#define I3C_S_SDA_PIN      I3C_S1_SDA_PIN
#define I3C_S_DRIVER       I3C_S1_DRIVER
#define I3C_S_SIGNAL_EVENT I3C_S1_SIGNAL_EVENT
#define I3C_S_DEVICE_INDEX I3C_S1_DEVICE_INDEX

#endif // __ISSDK_HAL_H__
