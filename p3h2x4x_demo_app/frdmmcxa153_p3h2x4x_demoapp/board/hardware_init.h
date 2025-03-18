/*
 * Copyright 2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _HARDWARE_INIT_H_
#define _HARDWARE_INIT_H_

#include "fsl_lpuart_cmsis.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define USART_LPUART             Driver_USART0
#define USART_LPUART_CLK_FREQ    CLOCK_GetLpuartClkFreq(0u)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_InitHardware(void);
void BOARD_UARTInitDebugConsole(void);

#endif /* _HARDWARE_INIT_H_ */
