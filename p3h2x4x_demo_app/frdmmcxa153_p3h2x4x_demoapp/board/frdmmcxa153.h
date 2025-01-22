/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file frdmMCXA153.h
 * @brief The frdmMCXA153.h file defines GPIO pin mappings for frdmMCXA153 board
 */

#ifndef FRDM_MCXA153_H_
#define FRDM_MCXA153_H_

#include "pin_mux.h"
#include "RTE_Device.h"
#include "gpio_driver.h"
#include "clock_config.h"
#include "board.h"


//GPIO Handle
extern gpioHandleKSDK_t SDA_PIN;
extern gpioHandleKSDK_t SCL_PIN;
extern gpioHandleKSDK_t RST_PIN;

// I3C_S1: Pin mapping and driver information for default I2C brought to shield
#define I3C_S1_SCL_PIN      J21
#define I3C_S1_SDA_PIN      J20
#define I3C_S1_DRIVER       Driver_I3C
#define I3C_S1_DEVICE_INDEX I3C_INDEX
#define I3C_S1_SIGNAL_EVENT I3C_SignalEvent_t

/* @brief  Ask use input to resume after specified samples have been processed. */
#define ASK_USER_TO_RESUME(x)                                                          \
    static bool askResume            = true;                                           \
    static uint16_t samplesToProcess = x - 1;                                          \
    if (askResume && !samplesToProcess--)                                              \
    {                                                                                  \
        PRINTF("\r\n Specified samples processed, press any key to continue... \r\n"); \
        GETCHAR();                                                                     \
        askResume = false;                                                             \
    }

/* @brief dummy arguement to Power Mode Wait Wrapper. */
#define SMC NULL
#define I2C0 LPI2C0
#define SPI1 LPSPI1

/* @brief Kinetis style Wrapper API for Power Mode Wait (Wait for Interrupt). */
status_t SMC_SetPowerModeWait(void *arg);
/* @brief Kinetis style Wrapper API for Power Mode VLPR (Wait for Interrupt). */
status_t SMC_SetPowerModeVlpr(void *arg);
/* @brief Kinetis style Wrapper API for handling all Clock related configurations. */
void BOARD_BootClockRUN(void);
#endif /* FRDM_MCXA153_H_ */
