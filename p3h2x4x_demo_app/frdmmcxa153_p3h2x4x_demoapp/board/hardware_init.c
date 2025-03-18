/*
 * Copyright 2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <hardware_init.h>
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console_cmsis.h"

void BOARD_UARTInitDebugConsole(void)
{
    /* attach FRO 12M to FLEXCOMM4 (debug console) */
	CLOCK_SetClockDiv(kCLOCK_DivLPUART0, 1u);
	CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    RESET_ClearPeripheralReset(BOARD_DEBUG_UART_RST);

    DebugConsole_Init();
    DebugConsole_PowerControl();
    DebugConsole_Control();

}
void BOARD_InitHardware(void)
{
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_UARTInitDebugConsole();
}

uint32_t LPUART0_GetFreq()
{
    return USART_LPUART_CLK_FREQ;
}

