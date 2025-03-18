/*
* Copyright 2025 NXP
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include <P3H2x4x.h>
#include <P3H2x4x_drv.h>
#include <string.h>
/*  SDK Included Files */
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console_cmsis.h"
#include "fsl_i3c.h"

//-----------------------------------------------------------------------
// CMSIS Includes
//-----------------------------------------------------------------------
#include "issdk_hal.h"
#include "Driver_Common.h"
#include "fsl_i3c_cmsis.h"
#include "systick_utils.h"
#include "P3H2x4x_Temp_sensor_drv.h"


int32_t P3T1755_I2C_GetTempRegValue(float fTemp, uint8_t * pBuffer, uint8_t is_i3c)
{
    int32_t negative = 0;
    uint16_t value = 0;

    if (fTemp < 0){
    	 negative = 1;
    	 fTemp *= -1;
    }

    value = (uint16_t)(fTemp / P3T1755DP_CELCIUS_CONV_VAL);
    value = value & 0x0fff;
    if (negative)
        value = ~(value - 1);
    value = value << P3T1755DP_TEMP_IGNORE_SHIFT;

    if(is_i3c) {
    	pBuffer[1] = (uint8_t)value;
    	pBuffer[0] = (uint8_t)(value >> 8);
    } else {
		pBuffer[2] = (uint8_t)value;
		pBuffer[1] = (uint8_t)(value >> 8);
    }

    return SENSOR_ERROR_NONE;
}

int32_t P3T1755_GetTempFloatValue(uint8_t * pBuffer, float * fTemp)
{
    int32_t negative = 0;
    uint16_t value = 0, temp = 0;

    temp = (uint16_t)((uint16_t)pBuffer[0] << 8) | (uint16_t)pBuffer[1];
    negative = (temp & P3T1755DP_TEMP_NEGPOS_MASK) != 0;

    temp >>= P3T1755DP_TEMP_IGNORE_SHIFT;
    if (negative) {
        value = ~(temp) + 1;
        value = (value & 0x0fff);
        *fTemp = -1 * P3T1755DP_CELCIUS_CONV_VAL * value;
    } else
        *fTemp = P3T1755DP_CELCIUS_CONV_VAL * temp;

    return SENSOR_ERROR_NONE;
}


int32_t P3T1755_Initialize(p3t1755_sensorhandle_t *pSensorHandle, ARM_DRIVER_I3C *pBus, uint8_t index, uint16_t sAddress)
{
    int32_t status;
    uint8_t reg = 0;

    /*! Check the input parameters. */
    if ((pSensorHandle == NULL) || (pBus == NULL))
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    pSensorHandle->deviceInfo.deviceInstance = index;
    pSensorHandle->deviceInfo.functionParam = NULL;
    pSensorHandle->deviceInfo.idleFunction = NULL;

    /*! Initialize the sensor handle. */
    pSensorHandle->pCommDrv = pBus;
    pSensorHandle->slaveAddress = sAddress;
    pSensorHandle->isInitialized = true;

    return SENSOR_ERROR_NONE;
}

int32_t P3T1755_GetTemp(D_P3H2x4x_Handle *P3H2x4xDriver, p3t1755_sensorhandle_t *pSensorHandle, int target_port, uint8_t target_address, bool in_I2C_mode, float *pBuffer)
{
    uint8_t reg[P3T1755DP_REG_SIZE_BYTES];
    int32_t result;

    /*! Validate for the correct handle and register write list.*/
    if (pSensorHandle == NULL)
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before applying configuration.*/
    if (pSensorHandle->isInitialized != true)
    {
	return SENSOR_ERROR_INIT;
    }

    if(in_I2C_mode){

		result = P3H2x4x_Target_Smbus_Read(P3H2x4xDriver, target_port, target_address, 2, P3T1755DP_GET_TEMP, &reg[0]);

		if (SENSOR_ERROR_NONE != result)
		{
			return -1;
		}
	}
	else{

		result = P3H2x4x_Target_I3C_read(P3H2x4xDriver, target_address, 2, P3T1755DP_GET_TEMP, reg);

		if (SENSOR_ERROR_NONE != result)
		{
			return -1;
		}
	}
    P3T1755_GetTempFloatValue(&reg[0], pBuffer);
    return SENSOR_ERROR_NONE;
}

int32_t P3T1755_GetTHigh(D_P3H2x4x_Handle *P3H2x4xDriver, p3t1755_sensorhandle_t *pSensorHandle, int target_port, uint8_t target_address, bool in_I2C_mode, float *pBuffer)
{
    int32_t result;
    uint8_t reg[P3T1755DP_REG_SIZE_BYTES];

    /*! Validate for the correct handle and register write list.*/
    if (pSensorHandle == NULL)
    {
	return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before applying configuration.*/
    if (pSensorHandle->isInitialized != true)
    {
	return SENSOR_ERROR_INIT;
    }

    if(in_I2C_mode){

		//Get current T-high
    	result = P3H2x4x_Target_Smbus_Read(P3H2x4xDriver, target_port, target_address, 2, P3T1755DP_GET_HIGH_TEMP, &reg[0]);

		if (SENSOR_ERROR_NONE != result)
		{
			PRINTF("\r\n T-high read failed! \r\n");
			return -1;
		}
	}
	else{

		//Get current T-high
		result = P3H2x4x_Target_I3C_read(P3H2x4xDriver, target_address, 2, P3T1755DP_GET_HIGH_TEMP, &reg[0]);

		if (SENSOR_ERROR_NONE != result)
		{
			PRINTF("\r\n T-high read failed! \r\n");
			return -1;
		}
	}
    P3T1755_GetTempFloatValue(&reg[0], pBuffer);

    return SENSOR_ERROR_NONE;
}

int32_t P3T1755_GetTLow(D_P3H2x4x_Handle *P3H2x4xDriver, p3t1755_sensorhandle_t *pSensorHandle, int target_port, uint8_t target_address, bool in_I2C_mode, float *pBuffer)
{
    int32_t result;
    uint8_t reg[P3T1755DP_REG_SIZE_BYTES];

    /*! Validate for the correct handle and register write list.*/
    if (pSensorHandle == NULL)
    {
	return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before applying configuration.*/
    if (pSensorHandle->isInitialized != true)
    {
	return SENSOR_ERROR_INIT;
    }

	if(in_I2C_mode){

		//Get current T-low
		result = P3H2x4x_Target_Smbus_Read(P3H2x4xDriver, target_port, target_address, 2, P3T1755DP_GET_LOW_TEMP, &reg[0]);

		if (SENSOR_ERROR_NONE != result)
		{
			PRINTF("\r\n T-low read failed! \r\n");
			return -1;
		}

	}
	else{

		//Get current T-low
		result = P3H2x4x_Target_I3C_read(P3H2x4xDriver, target_address, 2, P3T1755DP_GET_LOW_TEMP, &reg[0]);

		if (SENSOR_ERROR_NONE != result)
		{
			PRINTF("\r\n  T-low read failed! \r\n");
			return -1;
		}
	}

    P3T1755_GetTempFloatValue(&reg[0], pBuffer);

    return SENSOR_ERROR_NONE;
}

int32_t P3T1755_SetTHigh(D_P3H2x4x_Handle *P3H2x4xDriver, p3t1755_sensorhandle_t *pSensorHandle, int target_port, uint8_t target_address, bool in_I2C_mode, float fTemp)
{
    int32_t result, negative = 0;
    uint8_t buff[2];

    /*! Validate for the correct handle and register write list.*/
    if (pSensorHandle == NULL)
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before applying configuration.*/
    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

    if(in_I2C_mode){

		//set current T-high
    	buff[0] = 0x03;			//register to write T-high temperature
		P3T1755_I2C_GetTempRegValue(fTemp, &buff[0], 0); //Thigh temperature value

		result = P3H2x4x_Target_Smbus_Write(P3H2x4xDriver,target_port, target_address, 3, &buff[0]);
		if (SENSOR_ERROR_NONE != result)
		{
			PRINTF("\r\n T-high write failed! \r\n");
			return -1;
		}
	}
	else{
		P3T1755_I2C_GetTempRegValue(fTemp, &buff[0], 1); //THIGH temperature value

		//set current T-high
		result = P3H2x4x_Target_I3C_block_write(P3H2x4xDriver, target_address, 2, P3T1755DP_GET_HIGH_TEMP, buff, 0x00, 0x00);

		if (SENSOR_ERROR_NONE != result)
		{
			PRINTF("\r\n T-high write failed! \r\n");
			return -1;
		}

	}

    return SENSOR_ERROR_NONE;
}

int32_t P3T1755_SetTLow(D_P3H2x4x_Handle *P3H2x4xDriver, p3t1755_sensorhandle_t *pSensorHandle, int target_port, uint8_t target_address, bool in_I2C_mode, float fTemp)
{
	int32_t result, negative = 0;
	uint8_t buff[3];

	/*! Validate for the correct handle and register write list.*/
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before applying configuration.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	if(in_I2C_mode){
		//Get current T-low
		buff[0] = 0x02;			//register to write T-low temperature
		P3T1755_I2C_GetTempRegValue(fTemp, &buff[0], 0); //TLOW temperature value

		result = P3H2x4x_Target_Smbus_Write(P3H2x4xDriver,target_port, target_address, 3, &buff[0]);
		if (SENSOR_ERROR_NONE != result)
		{
			PRINTF("\r\n T-low write failed! \r\n");
			return -1;
		}
	}
	else{
		//Get current T-low
		P3T1755_I2C_GetTempRegValue(fTemp, &buff[0], 1); //TLOW temperature value

		result =  P3H2x4x_Target_I3C_block_write(P3H2x4xDriver, target_address, 2, P3T1755DP_GET_LOW_TEMP, buff, 0x00, 0x00);
		if (SENSOR_ERROR_NONE != result)
		{
			PRINTF("\r\n T-low write failed! \r\n");
			return -1;
		}
	}
    return SENSOR_ERROR_NONE;
}
