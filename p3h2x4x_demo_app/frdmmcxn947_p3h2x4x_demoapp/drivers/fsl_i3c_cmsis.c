/*
* Copyright 2025 NXP
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include "fsl_i3c_cmsis.h"

#define ARM_LPI2C_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR((2), (4))
i3c_master_config_t masterConfig;
i3c_master_transfer_t masterXfer;
i3c_master_handle_t g_i3c_m_handle;
status_t result = kStatus_Success;

/*
 * ARMCC does not support split the data section automatically, so the driver
 * needs to split the data to separate sections explicitly, to reduce codesize.
 */
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
#define ARMCC_SECTION(section_name) __attribute__((section(section_name)))
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/


extern void i3c_master_callback(I3C_Type *base, i3c_master_handle_t *handle, status_t status, void *userData);

void i3c_master_ibi_callback(I3C_Type *base, i3c_master_handle_t *handle, i3c_ibi_type_t ibiType, i3c_ibi_state_t ibiState);

const i3c_master_transfer_callback_t masterCallback = {
    .slave2Master = NULL, .ibiCallback = i3c_master_ibi_callback, .transferComplete = i3c_master_callback};


/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.lpi2c_cmsis"
#endif

typedef const struct _cmsis_lpi2c_resource
{
	I3C_Type *base;          /*!< LPI2C peripheral base address. LPI2C_Type     */
    uint32_t (*GetFreq)(void); /*!< Function to get the clock frequency. */

} cmsis_lpi2c_resource_t;

typedef union _cmsis_i2c_handle
{
	i3c_master_handle_t master_handle;
	i3c_slave_handle_t slave_handle;
} cmsis_i2c_handle_t;

typedef struct _cmsis_lpi2c_interrupt_driver_state
{
    cmsis_lpi2c_resource_t *resource; /*!< Basic LPI2C resource. */
    cmsis_i2c_handle_t *handle;
    uint8_t *slave_data;            /*!< slave Transfer buffer */
    size_t slave_dataSize;          /*!< slave Transfer size */
    ARM_I3C_SignalEvent_t cb_event; /*!< call back function */
    uint8_t flags;                  /*!< Control and state flags. */
} cmsis_lpi2c_interrupt_driver_state_t;

#if (defined(FSL_FEATURE_SOC_EDMA_COUNT) && FSL_FEATURE_SOC_EDMA_COUNT)
typedef const struct _cmsis_lpi2c_edma_resource
{
    void *txEdmaBase;              /*!< EDMA peripheral base address for Tx.  */
    uint32_t txEdmaChannel;            /*!< EDMA channel for Tx                   */
    uint16_t txDmaRequest; /*!< Tx EDMA request source.               */

    void *rxEdmaBase;              /*!< EDMA peripheral base address for Rx.  */
    uint32_t rxEdmaChannel;            /*!< EDMA channel for Rx                   */
    uint16_t rxDmaRequest; /*!< Rx EDMA request source.               */
#if (defined(FSL_FEATURE_SOC_DMAMUX_COUNT) && FSL_FEATURE_SOC_DMAMUX_COUNT)
    DMAMUX_Type *txDmamuxBase; /*!< DMAMUX peripheral base address for Tx */
    DMAMUX_Type *rxDmamuxBase; /*!< DMAMUX peripheral base address for Rx */
#endif
} cmsis_lpi2c_edma_resource_t;

#endif /* FSL_FEATURE_SOC_EDMA_COUNT */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static const ARM_DRIVER_VERSION s_lpi2cDriverVersion        = {ARM_I3C_API_VERSION, ARM_LPI2C_DRV_VERSION};
static const ARM_I3C_CAPABILITIES s_lpi2cDriverCapabilities = {
    0, /* Do not support 10-bit addressing.*/
};

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Returns version information */
static ARM_DRIVER_VERSION LPI3Cx_GetVersion(void)
{
    return s_lpi2cDriverVersion;
}

/* Returns information about capabilities of this driver implementation */
static ARM_I3C_CAPABILITIES LPI3Cx_GetCapabilities(void)
{
    return s_lpi2cDriverCapabilities;
}


static int32_t LPI3C_interruptInitialize(ARM_I3C_SignalEvent_t cb_event, cmsis_lpi2c_interrupt_driver_state_t *lpi2c)
{
    if (0U == (lpi2c->flags & (uint8_t)I2C_FLAG_INIT))
    {
        lpi2c->cb_event = cb_event; /* Call back function */
        lpi2c->flags    = (uint8_t)I2C_FLAG_INIT;
    }

    return ARM_DRIVER_OK;
}

static int32_t DAA(uint16_t staticaddress, uint8_t dynamicaddress){

	uint8_t dynamicAddr = dynamicaddress;
	uint8_t staticAddr = staticaddress;

    /* Reset dynamic address before DAA */
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = 0x7EU; /* Broadcast address */
    masterXfer.subaddress     = CCC_RSTDAA;
    masterXfer.subaddressSize = 1U;
    masterXfer.direction      = kI3C_Write;
    masterXfer.busType        = kI3C_TypeI3CSdr;
    masterXfer.flags          = kI3C_TransferDefaultFlag;
    masterXfer.ibiResponse    = kI3C_IbiRespAckMandatory;

    result = I3C_MasterTransferNonBlocking(EXAMPLE_MASTER, &g_i3c_m_handle, &masterXfer);
    if (kStatus_Success != result)
    {
        return result;
    }

#if defined(EXAMPLE_USE_SETDASA_ASSIGN_ADDR) && (EXAMPLE_USE_SETDASA_ASSIGN_ADDR)
    /* Assign dynamic address. */
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = 0x7EU;
    masterXfer.subaddress     = CCC_SETDASA;
    masterXfer.subaddressSize = 1U;
    masterXfer.direction      = kI3C_Write;
    masterXfer.busType        = kI3C_TypeI3CSdr;
    masterXfer.flags          = kI3C_TransferNoStopFlag;
    masterXfer.ibiResponse    = kI3C_IbiRespAckMandatory;

    result = I3C_MasterTransferNonBlocking(EXAMPLE_MASTER, &g_i3c_m_handle, &masterXfer);
    if (kStatus_Success != result)
    {
        return result;
    }

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = staticAddr;
    masterXfer.subaddress     = dynamicAddr << 1U;
    masterXfer.subaddressSize = 1U;
    masterXfer.direction      = kI3C_Write;
    masterXfer.busType        = kI3C_TypeI3CSdr;
    masterXfer.flags          = kI3C_TransferDefaultFlag;
    masterXfer.ibiResponse    = kI3C_IbiRespAckMandatory;

    result = I3C_MasterTransferNonBlocking(EXAMPLE_MASTER, &g_i3c_m_handle, &masterXfer);
    if (kStatus_Success != result)
    {
        return result;
    }
#else
    uint8_t addressList[8] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37};
    result                 = I3C_MasterProcessDAA(EXAMPLE_MASTER, addressList, 8);
    if (result != kStatus_Success)
    {
        return -1;
    }

    i3c_device_info_t *devList;
    uint8_t devIndex;
    uint8_t devCount;
    devList = I3C_MasterGetDeviceListAfterDAA(EXAMPLE_MASTER, &devCount);
    for (devIndex = 0; devIndex < devCount; devIndex++)
    {
        if (devList[devIndex].vendorID == 0x123U)
        {
            slaveAddr = devList[devIndex].dynamicAddr;
            break;
        }
    }
    if (devIndex == devCount)
    {
        PRINTF("\r\nI3C master dynamic address assignment fails!\r\n");
        return -1;
    }
#endif
}

static int32_t DAA_without_reset(uint16_t staticaddress, uint8_t dynamicaddress, cmsis_lpi2c_interrupt_driver_state_t *lpi2c){

	uint8_t dynamicAddr = dynamicaddress;
	uint8_t staticAddr = staticaddress, data;
    uint32_t status;

	if (lpi2c->handle->master_handle.state != 0U)
	{
		return ARM_DRIVER_ERROR_BUSY; /* Master is busy */
	}

	/* Create master_handle */
	I3C_MasterTransferCreateHandle(lpi2c->resource->base, &(lpi2c->handle->master_handle),
			&masterCallback, (void *)lpi2c->cb_event);

	data = CCC_SETDASA;

	memset(&masterXfer, 0, sizeof(masterXfer));
  /* Setup the master transfer */
	masterXfer.slaveAddress   = 0x7EU;
	masterXfer.direction      = kI3C_Write;
	masterXfer.busType        = kI3C_TypeI3CSdr;
	masterXfer.subaddress     = 0;
	masterXfer.subaddressSize = 0;
	masterXfer.data           = (uint8_t *)&data;
	masterXfer.dataSize       = 1;
	masterXfer.flags          =  (uint32_t) ( kI3C_TransferDefaultFlag | kI3C_TransferNoStopFlag);
	masterXfer.ibiResponse    = kI3C_IbiRespAckMandatory;

	/* Send master non-blocking data to slave */
	status = I3C_MasterTransferNonBlocking(lpi2c->resource->base, &(lpi2c->handle->master_handle), &masterXfer);

	data = dynamicaddress << 1;

	 memset(&masterXfer, 0, sizeof(masterXfer));
  /* Setup the master transfer */
	masterXfer.slaveAddress   = staticaddress;
	masterXfer.direction      = kI3C_Write;
	masterXfer.busType        = kI3C_TypeI3CSdr;
	masterXfer.subaddress     = 0;
	masterXfer.subaddressSize = 0;
	masterXfer.data           = (uint8_t *)&data;
	masterXfer.dataSize       = 1;
	masterXfer.flags          = (uint32_t)kI3C_TransferDefaultFlag;
	masterXfer.ibiResponse    = kI3C_IbiRespAckMandatory;

	/* Send master non-blocking data to slave */
	status = I3C_MasterTransferNonBlocking(lpi2c->resource->base, &(lpi2c->handle->master_handle), &masterXfer);

	if (kStatus_Success != result)
	{
		return result;
	}

	return kStatus_Success;
#if 0
#if defined(EXAMPLE_USE_SETDASA_ASSIGN_ADDR) && (EXAMPLE_USE_SETDASA_ASSIGN_ADDR)
    /* Assign dynamic address. */
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = 0x7EU;
    masterXfer.subaddress     = CCC_SETDASA;
    masterXfer.subaddressSize = 1U;
    masterXfer.direction      = kI3C_Write;
    masterXfer.busType        = kI3C_TypeI3CSdr; //;
    masterXfer.flags          = kI3C_TransferNoStopFlag;
    masterXfer.ibiResponse    = kI3C_IbiRespAckMandatory;

    result = I3C_MasterTransferNonBlocking(EXAMPLE_MASTER, &g_i3c_m_handle, &masterXfer);
    if (kStatus_Success != result)
    {
        return result;
    }

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = staticAddr;
    masterXfer.subaddress     = dynamicAddr << 1U;
    masterXfer.subaddressSize = 1U;
    masterXfer.direction      = kI3C_Write;
    masterXfer.busType        = kI3C_TypeI3CSdr; //kI3C_TypeI3CSdr;
    masterXfer.flags          = kI3C_TransferDefaultFlag;
    masterXfer.ibiResponse    = kI3C_IbiRespAckMandatory;

    result = I3C_MasterTransferNonBlocking(EXAMPLE_MASTER, &g_i3c_m_handle, &masterXfer);
    if (kStatus_Success != result)
    {
        return result;
    }
#else
    uint8_t addressList[8] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37};
    result                 = I3C_MasterProcessDAA(EXAMPLE_MASTER, addressList, 8);
    if (result != kStatus_Success)
    {
        return -1;
    }

    i3c_device_info_t *devList;
    uint8_t devIndex;
    uint8_t devCount;
    devList = I3C_MasterGetDeviceListAfterDAA(EXAMPLE_MASTER, &devCount);
    for (devIndex = 0; devIndex < devCount; devIndex++)
    {
        if (devList[devIndex].vendorID == 0x123U)
        {
            slaveAddr = devList[devIndex].dynamicAddr;
            break;
        }
    }
    if (devIndex == devCount)
    {
        PRINTF("\r\nI3C master dynamic address assignment fails!\r\n");
        return -1;
    }
#endif
#endif
}

static int32_t CCC_Direct_ENEC_Cmd(uint16_t address, uint16_t cmd,  cmsis_lpi2c_interrupt_driver_state_t *lpi2c){

	uint8_t   data;
    uint32_t status;

	if (lpi2c->handle->master_handle.state != 0U)
	{
		return ARM_DRIVER_ERROR_BUSY; /* Master is busy */
	}

	/* Create master_handle */
	I3C_MasterTransferCreateHandle(lpi2c->resource->base, &(lpi2c->handle->master_handle),
			&masterCallback, (void *)lpi2c->cb_event);

	data = CCC_DIRECT_ENEC;

	memset(&masterXfer, 0, sizeof(masterXfer));
  /* Setup the master transfer */
	masterXfer.slaveAddress   = 0x7EU;
	masterXfer.direction      = kI3C_Write;
	masterXfer.busType        = kI3C_TypeI3CSdr;
	masterXfer.subaddress     = 0;
	masterXfer.subaddressSize = 0;
	masterXfer.data           = (uint8_t *)&data;
	masterXfer.dataSize       = 1;
	masterXfer.flags          = kI3C_TransferNoStopFlag;
	masterXfer.ibiResponse    = kI3C_IbiRespAckMandatory;

	/* Send master non-blocking data to slave */
	status = I3C_MasterTransferNonBlocking(lpi2c->resource->base, &(lpi2c->handle->master_handle), &masterXfer);

	data = (uint8_t)cmd;

	 memset(&masterXfer, 0, sizeof(masterXfer));
  /* Setup the master transfer */
	masterXfer.slaveAddress   = address;
	masterXfer.direction      = kI3C_Write;
	masterXfer.busType        = kI3C_TypeI3CSdr;
	masterXfer.subaddress     = 0;
	masterXfer.subaddressSize = 0;
	masterXfer.data           = (uint8_t *)&data;
	masterXfer.dataSize       = 1;
	masterXfer.flags          = (uint32_t)kI3C_TransferDefaultFlag;
	masterXfer.ibiResponse    = kI3C_IbiRespAckMandatory;

	/* Send master non-blocking data to slave */
	status = I3C_MasterTransferNonBlocking(lpi2c->resource->base, &(lpi2c->handle->master_handle), &masterXfer);

	if (kStatus_Success != result)
	{
		return result;
	}

	return kStatus_Success;
}

static int32_t CCC_DIRECT_DISEC_Cmd(uint16_t address, uint16_t cmd,  cmsis_lpi2c_interrupt_driver_state_t *lpi2c){

	uint8_t data;
    uint32_t status;

	if (lpi2c->handle->master_handle.state != 0U)
	{
		return ARM_DRIVER_ERROR_BUSY; /* Master is busy */
	}

	/* Create master_handle */
	I3C_MasterTransferCreateHandle(lpi2c->resource->base, &(lpi2c->handle->master_handle),
			&masterCallback, (void *)lpi2c->cb_event);

	data = CCC_DIRECT_DISEC;

	memset(&masterXfer, 0, sizeof(masterXfer));
  /* Setup the master transfer */
	masterXfer.slaveAddress   = 0x7EU;
	masterXfer.direction      = kI3C_Write;
	masterXfer.busType        = kI3C_TypeI3CSdr;
	masterXfer.subaddress     = 0;
	masterXfer.subaddressSize = 0;
	masterXfer.data           = (uint8_t *)&data;
	masterXfer.dataSize       = 1;
	masterXfer.flags          = kI3C_TransferNoStopFlag;
	masterXfer.ibiResponse    = kI3C_IbiRespAckMandatory;

	/* Send master non-blocking data to slave */
	status = I3C_MasterTransferNonBlocking(lpi2c->resource->base, &(lpi2c->handle->master_handle), &masterXfer);

	data = (uint8_t)cmd;

	 memset(&masterXfer, 0, sizeof(masterXfer));
  /* Setup the master transfer */
	masterXfer.slaveAddress   = address;
	masterXfer.direction      = kI3C_Write;
	masterXfer.busType        = kI3C_TypeI3CSdr;
	masterXfer.subaddress     = 0;
	masterXfer.subaddressSize = 0;
	masterXfer.data           = (uint8_t *)&data;
	masterXfer.dataSize       = 1;
	masterXfer.flags          = (uint32_t)kI3C_TransferDefaultFlag;
	masterXfer.ibiResponse    = kI3C_IbiRespAckMandatory;

	/* Send master non-blocking data to slave */
	status = I3C_MasterTransferNonBlocking(lpi2c->resource->base, &(lpi2c->handle->master_handle), &masterXfer);

	if (kStatus_Success != result)
	{
		return result;
	}

	return kStatus_Success;
}

static int32_t I3C_InterruptPowerControl(ARM_POWER_STATE state, cmsis_lpi2c_interrupt_driver_state_t *lpi2c)
{
    int32_t result = ARM_DRIVER_OK;
    i3c_slave_config_t slaveConfig;
    i3c_master_config_t masterConfig;
    switch (state)
    {
        case ARM_POWER_OFF:
            if ((lpi2c->flags & (uint8_t)I2C_FLAG_POWER) != 0U)
            {
                /* Disables peripheral */
                I3C_MasterDeinit(lpi2c->resource->base);
                lpi2c->flags = (uint8_t)I2C_FLAG_INIT;
            }
            break;

        case ARM_POWER_LOW:
            result = ARM_DRIVER_ERROR_UNSUPPORTED;
            break;

        case ARM_POWER_FULL:

            if (lpi2c->flags == (uint8_t)I2C_FLAG_UNINIT)
            {
                result = ARM_DRIVER_ERROR;
                break;
            }

            if ((lpi2c->flags & (uint8_t)I2C_FLAG_POWER) != 0U)
            {
                /* Driver already powered */
                break;
            }

            I3C_MasterGetDefaultConfig(&masterConfig);

            masterConfig.baudRate_Hz.i2cBaud          = EXAMPLE_I2C_BAUDRATE;
            masterConfig.baudRate_Hz.i3cPushPullBaud  = EXAMPLE_I3C_PP_BAUDRATE;
            masterConfig.baudRate_Hz.i3cOpenDrainBaud = EXAMPLE_I3C_OD_BAUDRATE;
            masterConfig.enableOpenDrainStop          = false;

            /* Initialize the LPI2C master peripheral */
            I3C_MasterInit(lpi2c->resource->base, &masterConfig, lpi2c->resource->GetFreq());

            I3C_MasterTransferCreateHandle(EXAMPLE_MASTER, &g_i3c_m_handle, &masterCallback, NULL);
            lpi2c->flags |= (uint8_t)I2C_FLAG_POWER;

            break;

        default:
            result = ARM_DRIVER_ERROR_UNSUPPORTED;
            break;
    }

    return result;
}


static int32_t I3C_Master_InterruptTransmit(
   uint32_t slaveAddr, const uint8_t *data, uint32_t num, bool xfer_pending, bool is_I3C, cmsis_lpi2c_interrupt_driver_state_t *lpi2c)
{
    int32_t status;
    int32_t ret;

    if (lpi2c->handle->master_handle.state != 0U)
    {
        return ARM_DRIVER_ERROR_BUSY; /* Master is busy */
    }

    /* Create master_handle */
    I3C_MasterTransferCreateHandle(lpi2c->resource->base, &(lpi2c->handle->master_handle),
    		&masterCallback, (void *)lpi2c->cb_event);

    /* Setup the master transfer */
    masterXfer.slaveAddress   = slaveAddr;
    masterXfer.direction      = kI3C_Write;
    if(is_I3C)
    	 masterXfer.busType        = kI3C_TypeI3CSdr;
    else
    	 masterXfer.busType        = kI3C_TypeI2C;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = (uint8_t *)data;
    masterXfer.dataSize       = num;
    masterXfer.flags          = (uint32_t)kI3C_TransferDefaultFlag;
    masterXfer.ibiResponse    = kI3C_IbiRespAckMandatory;

    if (xfer_pending)
    {
        /* Stop condition will not be generated */
        masterXfer.flags |= (uint32_t)kI3C_TransferNoStopFlag;
    }

    /* Send master non-blocking data to slave */
    status = I3C_MasterTransferNonBlocking(lpi2c->resource->base, &(lpi2c->handle->master_handle), &masterXfer);

    switch (status)
    {
        case kStatus_Success:
            ret = ARM_DRIVER_OK;
            break;

        case kStatus_I3C_Busy:
            ret = ARM_DRIVER_ERROR_BUSY;
            break;

        default:
            ret = ARM_DRIVER_ERROR;
            break;
    }

    return ret;
}

static int32_t I3C_Master_Receive(
		uint32_t slaveAddr, const uint8_t *data, uint32_t num, bool xfer_pending, bool is_I3C, cmsis_lpi2c_interrupt_driver_state_t *lpi2c)
{
    int32_t status;
    int32_t ret;

    if (lpi2c->handle->master_handle.state != 0U)
    {
        /* Master is busy */
        return ARM_DRIVER_ERROR_BUSY;
    }

    /* Setup the master transfer */
    masterXfer.slaveAddress   = slaveAddr;
    masterXfer.direction      = kI3C_Read;
    if(is_I3C)
    	 masterXfer.busType        = kI3C_TypeI3CSdr;
    else
    	 masterXfer.busType        = kI3C_TypeI2C;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0U;
    masterXfer.data           = (uint8_t *)data;
    masterXfer.dataSize       = num;
    masterXfer.flags          = (uint32_t)kI3C_TransferDefaultFlag;
    masterXfer.ibiResponse    = kI3C_IbiRespAckMandatory;

	if (result != kStatus_Success)
	{
		return -1;
	}

    if (xfer_pending)
    {
        /* Stop condition will not be generated */
        masterXfer.flags |= (uint32_t)kI3C_TransferNoStopFlag;
    }

    /* Receive blocking data from slave */

    status = I3C_MasterTransferBlocking(EXAMPLE_MASTER, &masterXfer);

    switch (status)
    {
        case kStatus_Success:
            ret = ARM_DRIVER_OK;
            break;

        case kStatus_I3C_Busy:
            ret = ARM_DRIVER_ERROR_BUSY;
            break;

        default:
            ret = ARM_DRIVER_ERROR;
            break;
    }

    return ret;
}

static int32_t I2C_Master_Transmit(
		uint32_t slaveAddr, const uint8_t *data, uint32_t num, bool xfer_pending, bool is_I3C, cmsis_lpi2c_interrupt_driver_state_t *lpi2c)
{
		int32_t status;
	    int32_t ret;

	    if (lpi2c->handle->master_handle.state != 0U)
	    {
	        return ARM_DRIVER_ERROR_BUSY; /* Master is busy */
	    }

	    /* Setup the master transfer */
	    masterXfer.slaveAddress   = slaveAddr;
	    masterXfer.direction      = kI3C_Write;
	    if(is_I3C)
	    	 masterXfer.busType        = kI3C_TypeI3CSdr;
	    else
	    	 masterXfer.busType        = kI3C_TypeI2C;
	    masterXfer.subaddress     = 0;
	    masterXfer.subaddressSize = 0;
	    masterXfer.data           = (uint8_t *)data;
	    masterXfer.dataSize       = num;
	    masterXfer.flags          = (uint32_t)kI3C_TransferDefaultFlag;
	    masterXfer.ibiResponse    = kI3C_IbiRespAckMandatory;

	    if (xfer_pending)
	    {
	        /* Stop condition will not be generated */
	        masterXfer.flags |= (uint32_t)kI3C_TransferNoStopFlag;
	    }

	    /* Send master non-blocking data to slave */
	    status = I3C_MasterTransferBlocking(EXAMPLE_MASTER, &masterXfer);

	    switch (status)
	    {
	        case kStatus_Success:
	            ret = ARM_DRIVER_OK;
	            break;

	        case kStatus_I3C_Busy:
	            ret = ARM_DRIVER_ERROR_BUSY;
	            break;

	        default:
	            ret = ARM_DRIVER_ERROR;
	            break;
	    }

	    return ret;
}

uint32_t LPI2C2_GetFreq(void)
{
    /* attach FRO 12M to FLEXCOMM2 */
    CLOCK_SetClkDiv(kCLOCK_DivFlexcom2Clk, 1u);
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM2);

    return CLOCK_GetLPFlexCommClkFreq(2U);
}

extern uint32_t LPI2C2_GetFreq(void);

static cmsis_i2c_handle_t LPI2C2_Handle;
static cmsis_lpi2c_resource_t LPI2C2_Resource = {I3C1, LPI2C2_GetFreq};

#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
ARMCC_SECTION("lpi2c2_interrupt_driver_state")
static cmsis_lpi2c_interrupt_driver_state_t LPI2C2_InterruptDriverState = {
#else
static cmsis_lpi2c_interrupt_driver_state_t LPI2C2_InterruptDriverState = {
#endif
    &LPI2C2_Resource,
    &LPI2C2_Handle,

};

static int32_t LPI3C_InterruptInitialize(ARM_I3C_SignalEvent_t cb_event)
{
#ifdef RTE_I3C_PIN_INIT
    RTE_I3C_PIN_INIT();
#endif
    return LPI3C_interruptInitialize(cb_event, &LPI2C2_InterruptDriverState);
}

static int32_t LPI3C_dynamic_address_assignment(uint8_t staticAddess, uint8_t dynamicAddress){
	return DAA(staticAddess, dynamicAddress);
}

static int32_t LPI3C_dynamic_address_assignment_without_reset(uint8_t staticAddess, uint8_t dynamicAddress){
	return DAA_without_reset(staticAddess, dynamicAddress,  &LPI2C2_InterruptDriverState);
}

static int32_t LPI3C_InterruptPowerControl(ARM_POWER_STATE state)
{
    return I3C_InterruptPowerControl(state, &LPI2C2_InterruptDriverState);
}

static int32_t LPI3C_Master_InterruptTransmit(uint32_t subaddr, const uint8_t *data, uint32_t num, bool xfer_pending, bool is_I3C)
{
    return I3C_Master_InterruptTransmit(subaddr, data, num, xfer_pending, is_I3C, &LPI2C2_InterruptDriverState);
}

static int32_t LPI3C_Master_Receive(uint32_t subaddr, const uint8_t *data, uint32_t num, bool xfer_pending, bool is_I3C)
{
    return I3C_Master_Receive(subaddr, data, num, xfer_pending, is_I3C, &LPI2C2_InterruptDriverState);
}

static int32_t LPI3C_Direct_ENEC(uint16_t address, uint16_t cmd)
{
	return CCC_Direct_ENEC_Cmd(address, cmd, &LPI2C2_InterruptDriverState);
}

static int32_t LPI3C_Direct_DISEC(uint16_t address, uint16_t cmd)
{
	return CCC_DIRECT_DISEC_Cmd(address, cmd, &LPI2C2_InterruptDriverState);
}

static int32_t LPI2C_Master_Transmit(uint32_t slaveAddr, const uint8_t *data, uint32_t num, bool xfer_pending, bool is_I3C)
{
    return I2C_Master_Transmit(slaveAddr, data, num, xfer_pending, is_I3C, &LPI2C2_InterruptDriverState);
}

ARM_DRIVER_I3C Driver_I3C =   {LPI3Cx_GetVersion,
                              LPI3Cx_GetCapabilities,
#if defined(RTE_I3C_DMA_EN) && RTE_I3C_DMA_EN
                              LPI2C2_Master_EdmaInitialize,
                              LPI2C2_Master_EdmaUninitialize,
                              LPI2C2_Master_EdmaPowerControl,
                              LPI2C2_Master_EdmaTransmit,
                              LPI2C2_Master_EdmaReceive,
                              NULL,
                              NULL,
                              LPI2C2_Master_EdmaGetDataCount,
                              LPI2C2_Master_EdmaControl,
                              LPI2C2_Master_EdmaGetStatus
#else
                              LPI3C_InterruptInitialize,
                              LPI3C_InterruptPowerControl,
							  LPI3C_dynamic_address_assignment,
							  LPI3C_dynamic_address_assignment_without_reset,
                              LPI3C_Master_InterruptTransmit,
                              LPI3C_Master_Receive,
							  LPI3C_Direct_ENEC,
							  LPI3C_Direct_DISEC,
							  LPI2C_Master_Transmit,
#endif
};
