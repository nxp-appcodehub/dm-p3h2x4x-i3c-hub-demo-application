/*
 * Copyright 2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _APP_H_
#define _APP_H_

#include "fsl_lpuart_cmsis.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define USART_LPUART           Driver_USART4
#define USART_LPUART_CLK_FREQ CLOCK_GetLPFlexCommClkFreq(4u)


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void BOARD_InitHardware(void);
void BOARD_UARTInitDebugConsole(void);

#endif /* _APP_H_ */
