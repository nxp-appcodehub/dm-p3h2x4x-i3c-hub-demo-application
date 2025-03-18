/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file frdmmcxn947.c
 * @brief The frdmmcxn947.c file defines GPIO pins and I2C CMSIS utilities for frdmmcxn947 board.
 */

#include <frdmmcxa153.h>

//GPIO Pin Handles
gpioHandleKSDK_t SDA_PIN = {.base = GPIO0, .pinNumber = 16, .mask = 1 << (16), .irq = GPIO0_IRQn , .clockName = kCLOCK_GateGPIO0, .portNumber = 0};
gpioHandleKSDK_t SCL_PIN = {.base = GPIO0, .pinNumber = 17, .mask = 1 << (17), .irq = GPIO0_IRQn , .clockName = kCLOCK_GateGPIO0, .portNumber = 0};
gpioHandleKSDK_t RST_PIN = {.base = GPIO3, .pinNumber = 15, .mask = 1 << (15), .irq = GPIO3_IRQn , .clockName = kCLOCK_GateGPIO3, .portNumber = 3};

/*!
 * @brief Configures the system to WAIT power mode.
 *        API name used from Kinetis family to maintain compatibility.
 *
 * @param Power peripheral base address (dummy).
 * @return Configuration error code.
 */
status_t SMC_SetPowerModeWait(void *arg)
{
    // POWER_EnterSleep();

    return kStatus_Success;
}

/*!
 * @brief Configures the system to VLPR power mode.
 *        API name used from Kinetis family to maintain compatibility.
 *
 * @param Power peripheral base address (dummy).
 * @return Configuration error code.
 */
status_t SMC_SetPowerModeVlpr(void *arg)
{
    // POWER_EnterSleep();

    return kStatus_Success;
}

/*! @brief       Determines the Clock Frequency feature.
 *  @details     The Clock Frequecny computation API required by fsl_i2c_cmsis.c.
 *  @param[in]   void
 *  @Constraints None
 *  @Reentrant   Yes
 *  @return      uint32_t Returns the clock frequency .
 */
uint32_t LPI2C0_GetFreq(void)
{
    /* Attach peripheral clock */
    CLOCK_SetClockDiv(kCLOCK_DivLPI2C0, 1u);
    CLOCK_AttachClk(kFRO12M_to_LPI2C0);

    return CLOCK_GetLpi2cClkFreq();
}

/*! @brief       Determines the Clock Frequency feature.
 *  @details     The Clock Frequecny computation API required by fsl_spi_cmsis.c.
 *  @param[in]   void
 *  @Constraints None
 *  @Reentrant   Yes
 *  @return      uint32_t Returns the clock frequency .
 */
uint32_t LPSPI1_GetFreq(void)
{
    /* Attach peripheral clock */
    CLOCK_SetClockDiv(kCLOCK_DivLPSPI1, 1u);
    CLOCK_AttachClk(kFRO12M_to_LPSPI1);

    return CLOCK_GetLpspiClkFreq(1);
}
