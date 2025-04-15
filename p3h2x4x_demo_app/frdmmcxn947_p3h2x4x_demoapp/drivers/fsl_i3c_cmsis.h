/*
* Copyright 2025 NXP
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef FSL_I3C_CMSIS_H_
#define FSL_I3C_CMSIS_H_

#include "DRIVER_I3C.h"
#include "fsl_common.h"
#include "RTE_Device.h"
#include "fsl_i3c.h"

extern ARM_DRIVER_I3C Driver_I3C;

/* I2C Driver state flags */
#define I2C_FLAG_UNINIT (0UL)
#define I2C_FLAG_INIT   (1UL << 0)
#define I2C_FLAG_POWER  (1UL << 1)

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_MASTER                  I3C1
#define EXAMPLE_I2C_BAUDRATE            100000
#define EXAMPLE_I3C_OD_BAUDRATE         625000
#define EXAMPLE_I3C_PP_BAUDRATE         1250000
#define I3C_MASTER_CLOCK_FREQUENCY      CLOCK_GetI3cClkFreq(1U)
#define WAIT_TIME                       100000
#define EXAMPLE_I3C_HDR_SUPPORT         0
#define EXAMPLE_USE_SETDASA_ASSIGN_ADDR 1
#ifndef I3C_MASTER_SLAVE_ADDR_7BIT
#define I3C_MASTER_SLAVE_ADDR_7BIT 0x1EU
#endif
#ifndef I3C_DATA_LENGTH
#define I3C_DATA_LENGTH 2U
#endif
#ifndef EXAMPLE_I3C_HDR_SUPPORT
#define EXAMPLE_I3C_HDR_SUPPORT 0
#endif

#define CCC_RSTDAA  0x06U
#define CCC_SETDASA 0x87U
#define CCC_SETNEWDA 0x88U
#define CCC_DIRECT_ENEC 0x80U
#define CCC_DIRECT_DISEC 0x81U

#endif /* FSL_I3C_CMSIS_H_ */
