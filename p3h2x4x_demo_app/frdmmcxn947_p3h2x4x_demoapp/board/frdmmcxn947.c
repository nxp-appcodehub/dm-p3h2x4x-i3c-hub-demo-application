/*
* Copyright 2025 NXP
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include "frdmmcxn947.h"

//GPIO Pin Handles
gpioHandleKSDK_t SDA_PIN = {.base = GPIO1, .pinNumber = 16, .mask = 1 << (16), .irq = GPIO01_IRQn , .clockName = kCLOCK_Gpio1, .portNumber = 1};

gpioHandleKSDK_t SCL_PIN = {.base = GPIO1, .pinNumber = 17, .mask = 1 << (17), .irq = GPIO01_IRQn , .clockName = kCLOCK_Gpio1, .portNumber = 1};

gpioHandleKSDK_t RST_PIN = {.base = GPIO0, .pinNumber = 28, .mask = 1 << (28), .irq = GPIO00_IRQn , .clockName = kCLOCK_Gpio0, .portNumber = 0};
