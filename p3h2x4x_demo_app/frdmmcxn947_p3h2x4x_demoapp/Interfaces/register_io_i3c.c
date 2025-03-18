/*
* Copyright 2025 NXP
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/* Standard C Includes */
#include <string.h>
/* ISSDK Includes */
#include "issdk_hal.h"
#include "register_io_i3c.h"
#include "fsl_debug_console_cmsis.h"

/*******************************************************************************
 * Types
 ******************************************************************************/
#define I2C_COUNT (sizeof(i2cBases) / sizeof(void *))

/*******************************************************************************
 * Variables
 ******************************************************************************/
#if defined(CPU_MCXN947VDF_cm33_core0) || defined(CPU_MCXN547VDF_cm33_core0)
LPI2C_Type *const i2cBases[] = LPI2C_BASE_PTRS;
#else
I2C_Type *const i2cBases[] = I2C_BASE_PTRS;
#endif
volatile bool b_I2C_CompletionFlag[I2C_COUNT] = {false};
volatile uint32_t g_I2C_ErrorEvent[I2C_COUNT] = {ARM_I3C_EVENT_TRANSFER_DONE};
volatile bool g_masterCompletionFlag = false;
volatile bool g_ibiWonFlag = false;
volatile status_t g_completionStatus = kStatus_Success;

extern void i3c_master_callback(I3C_Type *base, i3c_master_handle_t *handle, status_t status, void *userData);
/*******************************************************************************
 * Code
 ******************************************************************************/

/* The I2C2 Signal Event Handler function. */
void I3C_SignalEvent_t(uint32_t event){

    if (event != ARM_I3C_EVENT_TRANSFER_DONE)
    {
        g_I2C_ErrorEvent[2] = event;
    }
    b_I2C_CompletionFlag[2] = true;
}

/*! The interface function to block write sensor registers. */
int32_t Register_I3C_BlockWrite(ARM_DRIVER_I3C *pCommDrv,
                                registerDeviceInfo_t *devInfo,
                                uint16_t slaveAddress,
                                uint8_t offset,
                                const uint8_t *pBuffer,
                                uint8_t bytesToWrite,
								bool is_i3c_mode)
{
    int32_t status;
    uint8_t buffer[SENSOR_MAX_REGISTER_COUNT];

    buffer[0] = offset;
    if(bytesToWrite >= SENSOR_MAX_REGISTER_COUNT)
    	return ARM_DRIVER_ERROR;
    memcpy(buffer + 1, pBuffer, bytesToWrite);

    g_masterCompletionFlag = false;
    g_I2C_ErrorEvent[devInfo->deviceInstance] = ARM_I3C_EVENT_TRANSFER_DONE;
    status = pCommDrv->MasterTransmit(slaveAddress, buffer, bytesToWrite + 1, false, is_i3c_mode);
    if (ARM_DRIVER_OK == status)
    {
        /* Wait for completion */
    	while ((!g_masterCompletionFlag) && (g_completionStatus == kStatus_Success)){

    		if (devInfo->idleFunction)
				{
					devInfo->idleFunction(devInfo->functionParam);
				}
			else
				{
					__NOP();
				}
    	}

        if (g_I2C_ErrorEvent[devInfo->deviceInstance] != ARM_I3C_EVENT_TRANSFER_DONE)
        {
            status = ARM_DRIVER_ERROR;
        }
    }

    return status;

}

/*! The interface function to write a sensor register. */
int32_t Register_I3C_Write(ARM_DRIVER_I3C *pCommDrv,
                           registerDeviceInfo_t *devInfo,
                           uint16_t slaveAddress,
                           uint8_t offset,
                           uint8_t value,
                           uint8_t mask,
                           bool repeatedStart,
						   bool is_i3c_mode)
{
    int32_t status;
    uint8_t config[] = {offset, 0x00};

    /*! Set the register based on the values in the register value pair configuration.*/
    if (mask)
    {
    	g_masterCompletionFlag = false;
        g_I2C_ErrorEvent[devInfo->deviceInstance] = ARM_I3C_EVENT_TRANSFER_DONE;
        /*! Send the register address to read from.*/
        status = pCommDrv->MasterTransmit(slaveAddress, &config[0], 1, true, is_i3c_mode);
        if (ARM_DRIVER_OK == status)
        {
        	while ((!g_masterCompletionFlag) && (g_completionStatus == kStatus_Success)){

        		if (devInfo->idleFunction)
    				{
    					devInfo->idleFunction(devInfo->functionParam);
    				}
    			else
    				{
    					__NOP();
    				}
        	}

            if (g_I2C_ErrorEvent[devInfo->deviceInstance] != ARM_I3C_EVENT_TRANSFER_DONE)
            {
                return ARM_DRIVER_ERROR;
            }
        }
        else
        {
            return status;
        }

        b_I2C_CompletionFlag[devInfo->deviceInstance] = false;
        g_I2C_ErrorEvent[devInfo->deviceInstance] = ARM_I3C_EVENT_TRANSFER_DONE;

        /*! Read the value.*/
        status = pCommDrv->MasterReceive(slaveAddress, &config[1], 1, false, is_i3c_mode);

        if (status != kStatus_Success)
		{
			return -1;
		}

        /*! 'OR' in the requested values to the current contents of the register */
        config[1] = (config[1] & ~mask) | value;
    }
    else
    {
        /*! Overwrite the register with specified value.*/
        config[1] = value;
    }
    g_masterCompletionFlag = false;
    g_I2C_ErrorEvent[devInfo->deviceInstance] = ARM_I3C_EVENT_TRANSFER_DONE;
    /*!  Write the updated value. */
    status = pCommDrv->MasterTransmit(slaveAddress, config, 2, repeatedStart, is_i3c_mode);
    if (ARM_DRIVER_OK == status)
    {
        /* Wait for completion */

    	while ((!g_masterCompletionFlag) && (g_completionStatus == kStatus_Success)){

		if (devInfo->idleFunction)
			{
				devInfo->idleFunction(devInfo->functionParam);
			}
		else
		{
			__NOP();
		}
    	}

        if (g_I2C_ErrorEvent[devInfo->deviceInstance] != ARM_I3C_EVENT_TRANSFER_DONE)
        {
            status = ARM_DRIVER_ERROR;
        }
    }
    return status;
}

/*! The interface function to read a sensor register. */
int32_t Register_I3C_Read(ARM_DRIVER_I3C *pCommDrv,
                          registerDeviceInfo_t *devInfo,
                          uint16_t slaveAddress,
                          uint8_t offset,
                          uint8_t length,
                          uint8_t *pOutBuffer,
						  bool is_i3c_mode)
{
    int32_t status;
    uint8_t config[] = {offset, 0x00};

    g_masterCompletionFlag = false;
    g_ibiWonFlag = false;
    g_I2C_ErrorEvent[devInfo->deviceInstance] = ARM_I3C_EVENT_TRANSFER_DONE;

    status = pCommDrv->MasterTransmit(slaveAddress, &offset, 1, true, is_i3c_mode);
    if (ARM_DRIVER_OK == status)
    {
        /* Wait for completion without calling idle function. */
		while ((!g_masterCompletionFlag) && (g_completionStatus == kStatus_Success))
	   {
		   __NOP();
	   }

		if (g_I2C_ErrorEvent[devInfo->deviceInstance] != ARM_I3C_EVENT_TRANSFER_DONE)
		{
			return ARM_DRIVER_ERROR;
		}
    }
    else
    {
        return status;
    }

    /*! Read and update the value.*/
    status = pCommDrv->MasterReceive(slaveAddress, pOutBuffer, length, false, is_i3c_mode);

	if (status != kStatus_Success)
	{
		return -1;
	}
    return status;
}

/*! The interface function to write a sensor register. */
int32_t Register_I2C_Write(ARM_DRIVER_I3C *pCommDrv,
                           registerDeviceInfo_t *devInfo,
                           uint16_t slaveAddress,
                           uint8_t offset,
                           uint8_t value,
                           uint8_t mask,
                           bool repeatedStart,
						   bool is_i3c_mode)
{
    int32_t status;
    uint8_t config[] = {offset, 0x00};

    /*! Set the register based on the values in the register value pair configuration.*/
    if (mask)
    {
    	g_masterCompletionFlag = false;
        g_I2C_ErrorEvent[devInfo->deviceInstance] = ARM_I3C_EVENT_TRANSFER_DONE;
        /*! Send the register address to read from.*/
        status = pCommDrv->I2C_MasterTransmit(slaveAddress, &config[0], 1, true, is_i3c_mode);

    	if (status != kStatus_Success)
    	{
    		return -1;
    	}

        return status;

        b_I2C_CompletionFlag[devInfo->deviceInstance] = false;
        g_I2C_ErrorEvent[devInfo->deviceInstance] = ARM_I3C_EVENT_TRANSFER_DONE;

        /*! Read the value.*/
        status = pCommDrv->MasterReceive(slaveAddress, &config[1], 1, false, is_i3c_mode);

        if (status != kStatus_Success)
		{
			return -1;
		}

        /*! 'OR' in the requested values to the current contents of the register */
        config[1] = (config[1] & ~mask) | value;
    }
    else
    {
        /*! Overwrite the register with specified value.*/
        config[1] = value;
    }
    g_masterCompletionFlag = false;
    g_I2C_ErrorEvent[devInfo->deviceInstance] = ARM_I3C_EVENT_TRANSFER_DONE;
    /*!  Write the updated value. */
    status = pCommDrv->I2C_MasterTransmit(slaveAddress, config, 2, repeatedStart, is_i3c_mode);

	if (status != kStatus_Success)
	{
		return -1;
	}

    return status;
}
