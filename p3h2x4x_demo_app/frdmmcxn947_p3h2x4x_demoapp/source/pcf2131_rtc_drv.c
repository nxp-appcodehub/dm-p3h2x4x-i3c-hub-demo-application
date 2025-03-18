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
#include "pcf2131_rtc_drv.h"

//-----------------------------------------------------------------------
// CMSIS Includes
//-----------------------------------------------------------------------
#include "issdk_hal.h"
#include "Driver_Common.h"
#include "fsl_i3c_cmsis.h"
#include "systick_utils.h"
#include "P3H2x4x_Temp_sensor_drv.h"


uint8_t DecimaltoBcd(uint8_t val)
{
	return (((val/10) << 4) | (((val) % 10) & 0x0f));
}

uint8_t BcdToDecimal(uint8_t val)
{
	return (((val) >> 4) * 10 + ((val) & 0x0f)) ;
}

int32_t PCF2131_Initialize(pcf2131_sensorhandle_t *pSensorHandle, ARM_DRIVER_I3C *pBus, uint8_t index, uint16_t sAddress)
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

/*! The interface function to read register data from a sensor. */
int32_t Sensor_I2C_Read(D_P3H2x4x_Handle *P3H2x4xDriver,
                        uint16_t slaveAddress,
                        const registerreadlist_t *pReadList,
                        uint8_t *pOutBuffer)
{
    int32_t status;
    uint8_t *pBuf;

    /*! Validate for the correct handle.*/
    if (pReadList == NULL || pOutBuffer == NULL)
    {
        return SENSOR_ERROR_BAD_ADDRESS;
    }
    const registerreadlist_t *pCmd = pReadList;

    /*! Traverse the read list and read the registers one by one unless the register read list numBytes is zero*/
    for (pBuf = pOutBuffer; pCmd->numBytes != 0; pCmd++)
    {

        status = P3H2x4x_Target_I2C_read(P3H2x4xDriver, PCF2131_I2C_ADDR, pCmd->numBytes, pCmd->readFrom,
        		 pBuf);

        if (ARM_DRIVER_OK != status)
        {
            return SENSOR_ERROR_READ;
        }
        pBuf += pCmd->numBytes;
    }
    return SENSOR_ERROR_NONE;
}

int32_t PCF2131_Rtc_Start(pcf2131_sensorhandle_t *pSensorHandle, D_P3H2x4x_Handle *P3H2x4xDriver)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/*! Start RTC source clock */
	status = P3H2x4x_Target_I2C_write(P3H2x4xDriver, PCF2131_I2C_ADDR, PCF2131_CTRL1, rtcStart, PCF2131_CTRL1_START_STOP_SHIFT, PCF2131_CTRL1_START_STOP_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCF2131_Rtc_Stop(pcf2131_sensorhandle_t *pSensorHandle, D_P3H2x4x_Handle *P3H2x4xDriver)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/*! Stop RTC source clock */
	status = P3H2x4x_Target_I2C_write(P3H2x4xDriver, PCF2131_I2C_ADDR, PCF2131_CTRL1, rtcStop, PCF2131_CTRL1_START_STOP_SHIFT, PCF2131_CTRL1_START_STOP_MASK);

	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCF2131_ReadData(D_P3H2x4x_Handle *P3H2x4xDriver, pcf2131_sensorhandle_t *pSensorHandle,
		const registerreadlist_t *pReadList,uint8_t *pBuffer)
{
	int32_t status;

	/*! Validate for the correct handle and register read list.*/
	if ((pSensorHandle == NULL) || (pReadList == NULL) || (pBuffer == NULL))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before reading sensor data.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/*! Parse through the read list and read the data one by one. */
	status = Sensor_I2C_Read(P3H2x4xDriver, PCF2131_I2C_ADDR,
			pReadList, pBuffer);


	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_READ;
	}
	return SENSOR_ERROR_NONE;
}

int32_t PCF2131_12h_24h_Mode_Set(D_P3H2x4x_Handle *P3H2x4xDriver, pcf2131_sensorhandle_t *pSensorHandle, Mode12h_24h is_mode12h)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/*! Set 12/24 mode */

	status = P3H2x4x_Target_I2C_write(P3H2x4xDriver, PCF2131_I2C_ADDR, PCF2131_CTRL1,
			(uint8_t)((is_mode12h == mode12H) ? mode12H : mode24H ), PCF2131_CTRL1_12_HOUR_24_HOUR_MODE_SHIFT, PCF2131_CTRL1_12_HOUR_24_HOUR_MODE_MASK);


	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCF2131_12h_24h_Mode_Get(D_P3H2x4x_Handle *P3H2x4xDriver, pcf2131_sensorhandle_t *pSensorHandle, Mode12h_24h *pmode_12_24)
{
	int32_t status;
	PCF2131_CTRL_1 Ctrl1_Reg;

	/*! Validate for the correct handle */
	if ((pSensorHandle == NULL) || (pmode_12_24 == NULL))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/*! Get 12/24 mode */

    status = P3H2x4x_Target_I2C_read(P3H2x4xDriver, PCF2131_I2C_ADDR, 1, PCF2131_CTRL1,
    		(uint8_t *)&Ctrl1_Reg);

	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_READ;
	}
	*pmode_12_24 = Ctrl1_Reg.b.mode_12_24;

	return SENSOR_ERROR_NONE;
}

int32_t PCF2131_Sec100TH_Mode(D_P3H2x4x_Handle *P3H2x4xDriver, pcf2131_sensorhandle_t *pSensorHandle, S100thMode is_s100h)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/*! 100th Second mode Enable/Disable */

	status = P3H2x4x_Target_I2C_write(P3H2x4xDriver, PCF2131_I2C_ADDR, PCF2131_CTRL1,
			(uint8_t)((is_s100h == s100thEnable) ? s100thEnable : s100thDisable), PCF2131_CTRL1_100TH_S_DIS_SHIFT, PCF2131_CTRL1_100TH_S_DIS_MASK);

	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCF2131_Sec100TH_Mode_Get(D_P3H2x4x_Handle *P3H2x4xDriver, pcf2131_sensorhandle_t *pSensorHandle, S100thMode *s100_mode)
{
	int32_t status;
	PCF2131_CTRL_1 Ctrl1_Reg;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/*! Get 100th Second mode */

    status = P3H2x4x_Target_I2C_read(P3H2x4xDriver, PCF2131_I2C_ADDR, 1, PCF2131_CTRL1,
    		(uint8_t *)&Ctrl1_Reg);

	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}
	*s100_mode = Ctrl1_Reg.b.s_dis_100th;

	return SENSOR_ERROR_NONE;
}

int32_t PCF2131_SetTime(D_P3H2x4x_Handle *P3H2x4xDriver, pcf2131_sensorhandle_t *pSensorHandle, pcf2131_timedata_t *time)
{
	int32_t status;
	Mode12h_24h mode12_24;

	/*! Validate for the correct handle and time read variable.*/
	if ((pSensorHandle == NULL) || (time == NULL))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/*! Set 100th Second.*/

	status = P3H2x4x_Target_I2C_write(P3H2x4xDriver, PCF2131_I2C_ADDR, PCF2131_100TH_SECOND,
			DecimaltoBcd(time->second_100th & PCF2131_SECONDS_100TH_MASK), 0x00, PCF2131_SECONDS_100TH_MASK);

	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	/*! Set Second.*/

	status = P3H2x4x_Target_I2C_write(P3H2x4xDriver, PCF2131_I2C_ADDR, PCF2131_SECOND,
			DecimaltoBcd(time->second & PCF2131_SECONDS_MASK), 0x00, PCF2131_SECONDS_MASK);

	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	/*! Set Minutes.*/

	status = P3H2x4x_Target_I2C_write(P3H2x4xDriver, PCF2131_I2C_ADDR, PCF2131_MINUTE,
			DecimaltoBcd(time->minutes & PCF2131_MINUTES_MASK), 0x00, PCF2131_MINUTES_MASK);

	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	/*! Update AM/PM Bit.*/
	if(time->ampm == AM)
	{
		time->hours = DecimaltoBcd(time->hours & PCF2131_HOURS_MASK_12H);
		time->hours = ((time->hours) & (~(PM << PCF2131_AM_PM_SHIFT)));
	}
	else if(time->ampm == PM)
	{
		time->hours = DecimaltoBcd(time->hours & PCF2131_HOURS_MASK_12H);
		time->hours = (time->hours | (PM << PCF2131_AM_PM_SHIFT));
	}
	else
		time->hours = DecimaltoBcd(time->hours & PCF2131_HOURS_MASK);

	/*! Set Hour.*/

	status = P3H2x4x_Target_I2C_write(P3H2x4xDriver, PCF2131_I2C_ADDR, PCF2131_HOUR,
			time->hours, 0x00, PCF2131_HOURS_MASK);


	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	/*! Set Day.*/

	status = P3H2x4x_Target_I2C_write(P3H2x4xDriver, PCF2131_I2C_ADDR, PCF2131_DAY,
			DecimaltoBcd(time->days & PCF2131_DAYS_MASK), 0x00, PCF2131_DAYS_MASK);

	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	/*! Set WeekDay.*/

	status = P3H2x4x_Target_I2C_write(P3H2x4xDriver, PCF2131_I2C_ADDR, PCF2131_WEEKEND,
			DecimaltoBcd(time->weekdays &  PCF2131_WEEKDAYS_MASK), 0x00, PCF2131_WEEKDAYS_MASK);

	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	/*! Set Months.*/

	status = P3H2x4x_Target_I2C_write(P3H2x4xDriver, PCF2131_I2C_ADDR, PCF2131_MONTH,
			DecimaltoBcd(time->months & PCF2131_MONTHS_MASK), 0x00, PCF2131_MONTHS_MASK);

	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	/*! Set Year.*/

	status = P3H2x4x_Target_I2C_write(P3H2x4xDriver, PCF2131_I2C_ADDR, PCF2131_YEAR,
			DecimaltoBcd(time->years & PCF2131_YEARS_MASK), 0x00, PCF2131_YEARS_MASK);

	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCF2131_GetTime(D_P3H2x4x_Handle *P3H2x4xDriver, pcf2131_sensorhandle_t *pSensorHandle, const registerreadlist_t *pcf2131timedata, pcf2131_timedata_t *time )
{
	int32_t status;
	Mode12h_24h mode12_24;

	/*! Validate for the correct handle and time read variable.*/
	if ((pSensorHandle == NULL) || (time == NULL))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/*! Get time.*/
	status = PCF2131_ReadData(P3H2x4xDriver, pSensorHandle, pcf2131timedata, ( uint8_t *)time );
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	/*! after read convert BCD to Decimal */
	time->second_100th = BcdToDecimal(time->second_100th);
	time->second = BcdToDecimal(time->second & PCF2131_SECONDS_MASK) ;
	time->minutes = BcdToDecimal(time->minutes & PCF2131_MINUTES_MASK);

	PCF2131_12h_24h_Mode_Get(P3H2x4xDriver, pSensorHandle, &mode12_24);
	if(mode12_24 ==  mode24H)
	{
		time->hours = BcdToDecimal(time->hours & PCF2131_HOURS_MASK) ;
		time->ampm = h24;
	}
	else   /* Set AM/PM */
	{
		if( (time->hours >> 5) & 0x01 )
			time->ampm = PM;
		else
			time->ampm = AM;
		time->hours =  BcdToDecimal(time->hours & PCF2131_HOURS_MASK_12H);
	}
	time->days = BcdToDecimal(time->days & PCF2131_DAYS_MASK);
	time->weekdays = BcdToDecimal(time->weekdays & PCF2131_WEEKDAYS_MASK);
	time->months = BcdToDecimal(time->months & PCF2131_MONTHS_MASK);
	time->years = BcdToDecimal(time->years) ;

	return SENSOR_ERROR_NONE;
}
