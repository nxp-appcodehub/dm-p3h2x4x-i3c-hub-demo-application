/*
 * Copyright 2022, 2025 NXP
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _RTE_DEVICE_H
#define _RTE_DEVICE_H

extern void LPI3C_InitPins();
extern void LPI3C_DeinitPins();

extern void LPSPI1_InitPins();
extern void LPSPI1_DeinitPins();

extern void LPUART0_InitPins();
extern void LPUART0_deinitPins();


/* Driver name mapping. */
/* User needs to provide the implementation of LPI2CX_GetFreq/LPI2CX_InitPins/LPI2CX_DeinitPins for the enabled LPI2C
 * instance. */
#define RTE_I3C        1
#define RTE_I3C_DMA_EN 0

/* LPI2C configuration. */
#define RTE_I3C_PIN_INIT        LPI3C_InitPins
#define RTE_I3C_PIN_DEINIT      LPI3C_DeinitPins
#define RTE_I3C_DMA_TX_CH       2
#define RTE_I3C_DMA_TX_PERI_SEL (uint16_t) kDma0RequestLPI2C0Tx
#define RTE_I3C_DMA_TX_DMA_BASE DMA0
#define RTE_I3C_DMA_RX_CH       3
#define RTE_I3C_DMA_RX_PERI_SEL (uint16_t) kDma0RequestLPI2C0Rx
#define RTE_I3C_DMA_RX_DMA_BASE DMA0

/*Driver name mapping.*/
/* User needs to provide the implementation of LPSPIX_GetFreq/LPSPIX_InitPins/LPSPIX_DeinitPins for the enabled LPSPI
 * instance. */
#define RTE_SPI1        1
#define RTE_SPI1_DMA_EN 0

/* SPI configuration. */
#define RTE_SPI1_PCS_TO_SCK_DELAY       1000
#define RTE_SPI1_SCK_TO_PSC_DELAY       1000
#define RTE_SPI1_BETWEEN_TRANSFER_DELAY 1000
#define RTE_SPI1_MASTER_PCS_PIN_SEL     (kLPSPI_MasterPcs1)
#define RTE_SPI1_SLAVE_PCS_PIN_SEL      (kLPSPI_SlavePcs1)
#define RTE_SPI1_PIN_INIT               LPSPI1_InitPins
#define RTE_SPI1_PIN_DEINIT             LPSPI1_DeinitPins
#define RTE_SPI1_DMA_TX_CH              0
#define RTE_SPI1_DMA_TX_PERI_SEL        (uint16_t) kDma0RequestLPSPI1Tx
#define RTE_SPI1_DMA_TX_DMA_BASE        DMA0
#define RTE_SPI1_DMA_RX_CH              1
#define RTE_SPI1_DMA_RX_PERI_SEL        (uint16_t) kDma0RequestLPSPI1Rx
#define RTE_SPI1_DMA_RX_DMA_BASE        DMA0

/* Driver name mapping. */
/* User needs to provide the implementation of LPUARTX_GetFreq/LPUARTX_InitPins/LPUARTX_DeinitPins for the enabled
 * LPUART instance. */
#define RTE_USART0        1
#define RTE_USART0_DMA_EN 0

/* UART configuration. */
#define RTE_USART0_PIN_INIT        LPUART0_InitPins
#define RTE_USART0_PIN_DEINIT      LPUART0_deinitPins
#define RTE_USART0_DMA_TX_CH       0
#define RTE_USART0_DMA_TX_PERI_SEL (uint16_t) kDma0RequestLPUART0Tx
#define RTE_USART0_DMA_TX_DMA_BASE DMA0
#define RTE_USART0_DMA_RX_CH       1
#define RTE_USART0_DMA_RX_PERI_SEL (uint16_t) kDma0RequestLPUART0Rx
#define RTE_USART0_DMA_RX_DMA_BASE DMA0

#endif /* _RTE_DEVICE_H */
