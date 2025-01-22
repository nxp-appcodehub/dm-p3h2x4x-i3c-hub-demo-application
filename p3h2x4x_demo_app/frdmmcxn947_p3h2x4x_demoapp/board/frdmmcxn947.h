/*
* Copyright 2025 NXP
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef FRDMMCXN947_H_
#define FRDMMCXN947_H_

#include "pin_mux.h"
#include "RTE_Device.h"
#include "clock_config.h"
#include "board.h"
#include "register_io_i3c.h"
#include "gpio_driver.h"

#define I3C_S1_SIGNAL_EVENT I3C_SignalEvent_t
#define I3C_S1_DRIVER       Driver_I3C
#define I3C_S1_DEVICE_INDEX I3C_INDEX


extern gpioHandleKSDK_t SDA_PIN;

extern gpioHandleKSDK_t SCL_PIN;

extern gpioHandleKSDK_t RST_PIN;

#endif /* FRDMMCXN947_H_ */
