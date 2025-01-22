/*
* Copyright 2025 NXP
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef PCF2131_RTC_DRV_H_
#define PCF2131_RTC_DRV_H_

#include <P3H2x4x_drv.h>
#include "register_io_i3c.h"

#define PCF2131_TIME_SIZE_BYTE    (8)

/*External RTC pcf2131 Configurations*/
#define PCF2131_CTRL1 						 0x00
#define PCF2131_CTRL1_START_STOP_MASK       ((uint8_t)0x20)
#define PCF2131_CTRL1_START_STOP_SHIFT      ((uint8_t)5)

//for Testing external I2C device-RTC(PCF2131) I2C address
#define PCF2131_I2C_ADDR  0x53U

#define PCF2131_CTRL1  					0x00
#define PCF2131_CTRL1_100TH_S_DIS_MASK ((uint8_t)0x10)
#define PCF2131_CTRL1_100TH_S_DIS_SHIFT ((uint8_t)4)

#define PCF2131_100TH_SECOND  				0x06
 /* 100th_Seconds - Bit field mask definitions*/
#define PCF2131_SECONDS_100TH_MASK    ((uint8_t)0xFF)
#define PCF2131_SECONDS_100TH_SHIFT    ((uint8_t)0)

#define	PCF2131_SECOND 						0x07
 /* seconds - Bit field mask definitions*/
#define PCF2131_SECONDS_MASK    ((uint8_t)0x7F)
#define PCF2131_SECONDS_SHIFT    ((uint8_t)0)

#define	PCF2131_MINUTE  					0x08
 /* Minutes - Bit field mask definitions*/
#define PCF2131_MINUTES_MASK    ((uint8_t)0x7F)
#define PCF2131_MINUTES_SHIFT    ((uint8_t)0)

#define	PCF2131_HOUR   						0x09
/* Hours - Bit field mask definitions*/
#define PCF2131_HOURS_MASK        ((uint8_t)0x3F)
#define PCF2131_HOURS_MASK_12H    ((uint8_t)0x1F)
#define PCF2131_HOURS_SHIFT       ((uint8_t)0)
#define PCF2131_AM_PM_SHIFT       ((uint8_t)5)

#define PCF2131_CTRL1_12_HOUR_24_HOUR_MODE_MASK ((uint8_t)0x04)
#define PCF2131_CTRL1_12_HOUR_24_HOUR_MODE_SHIFT ((uint8_t)2)

#define	PCF2131_DAY  						0x0A
/* Days - Bit field mask definitions*/
#define PCF2131_DAYS_MASK    ((uint8_t)0x3F)
#define PCF2131_DAYS_SHIFT    ((uint8_t)0)

#define	PCF2131_WEEKEND 					0x0B
/*
 * Weekdays - Bit field mask definitions
 */
#define PCF2131_WEEKDAYS_MASK    ((uint8_t)0x07)
#define PCF2131_WEEKDAYS_SHIFT    ((uint8_t)0)

#define	PCF2131_MONTH  						0x0C
/*
 * Months - Bit field mask definitions
 */
#define PCF2131_MONTHS_MASK    ((uint8_t)0x1F)
#define PCF2131_MONTHS_SHIFT    ((uint8_t)0)

#define	PCF2131_YEAR  						0x0D
/*
 * Years - Bit field mask definitions
 */
#define PCF2131_YEARS_MASK    ((uint8_t)0xFF)
#define PCF2131_YEARS_SHIFT    ((uint8_t)0)

/*--------------------------------
 ** Register: Control_1
 ** Enum: PCF2131_CTRL1
 ** --
 ** Offset : 0x00 control and status register 1.
 ** ------------------------------*/
typedef union
{
	struct
	{
		uint8_t si : 1; /*  Second interrupt 0-disabled 1-enabled */

		uint8_t mi : 1; /*  Minute interrupt 0- disabled 1-enabled */

		uint8_t mode_12_24 : 1;/*  0-24 hour mode selected 1- 12 hour mode selected */

		uint8_t por_ovrd : 1; /*  Power on reset override 0-disabled 1-enabled */

		uint8_t s_dis_100th : 1; /*  0- 100th seconds counter enabled 1- 100th seconds counter disabled */

		uint8_t stop : 1;     /*  0- RTC source Clock runs 1- RTC clock is stopped */

		uint8_t tc_dis : 1;   /*  0- Temperature Compensation Enabled 1- Temperature Compensation disabled */

		uint8_t ext_test : 1; /*  0- Normal Mode 1- External Clock Test Mode. */
	} b;
	uint8_t w;
} PCF2131_CTRL_1;

typedef enum RTCONOFF
{
	rtcStart = 0x00,        /* Start Real Time Clock */
	rtcStop = 0x01,         /* Stop Real Time Clock */
}RtcOnOff;

typedef enum MODE12H_24H
{
	mode24H = 0x00,         /* 24 Hour Mode Set*/
	mode12H = 0x01,         /* 12 Hour Mode Set */
}Mode12h_24h;

typedef enum AMPM
{
	AM = 0x00,             /* AM in 12H Mode */
	PM = 0x01,             /* PM in 12H Mode */
	h24 = 0x02,            /* 24H Mode*/
}AmPm;
/*--------------------------------
 ** Enum: S100thMode
 ** @brief 100th Second Mode Enable/
 ** 		  Disable
 ** ------------------------------*/
typedef enum S100THMODE
{
	s100thEnable = 0x00,     /* 100th Second Mode Enabled */
	s100thDisable = 0x01,    /* 100th Second Mode Disabled */
}S100thMode;

typedef struct
{
	registerDeviceInfo_t deviceInfo;      /*!< SPI device context. */
	ARM_DRIVER_I3C *pCommDrv;        /*!< Pointer to the i2c driver. */
	bool isInitialized;                   /*!< Whether sensor is intialized or not.*/
	uint16_t slaveAddress;           /*!< slave address.*/
}  pcf2131_sensorhandle_t;

/*! @brief This structure defines the Time Data in 100th Seconds to Years.*/
typedef struct
{
	uint8_t  second_100th;
	uint8_t  second;
	uint8_t  minutes;
	uint8_t  hours;
	uint8_t  days;
	uint8_t  weekdays;
	uint8_t  months;
	uint8_t  years;
	AmPm     ampm;
} pcf2131_timedata_t;

/*! @brief       Sets an idle task for the PCF2131 RTC.
 *  @details     Sets a function to be called when the sensor is in idle state.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   idleTask  			Function Pointer to the idle task.
 *  @param[in]   userParam     		Pointer to user defined parameter for the idle task.
 *  @constraints This can be called any number of times only after PCF2131_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 */
int32_t PCF2131_Initialize(pcf2131_sensorhandle_t *pSensorHandle, ARM_DRIVER_I3C *pBus, uint8_t index, uint16_t sAddress);

/*! @brief       Reads the data of RTC from a read list.
 *  @return      ::Sensor_I2C_Read() returns the status.
 */
int32_t Sensor_I2C_Read(D_P3H2x4x_Handle *P3H2x4xDriver, uint16_t slaveAddress, const registerreadlist_t *pReadList, uint8_t *pOutBuffer);

/*! @brief       Start the real time clock (RTC)  for the PCF2131 RTC.
 *  @details     Start the real time clock (RTC) of the RTC.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCF2131_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return      ::PCF2131_Rtc_Start() returns the status.
 */
int32_t PCF2131_Rtc_Start(pcf2131_sensorhandle_t *pSensorHandle, D_P3H2x4x_Handle *P3H2x4xDriver);

/*! @brief       Stops the real time clock (RTC) for the PCF2131 RTC.
 *  @details     Stops the real time clock (RTC) of the RTC.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCF2131_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return      ::PCF2131_Rtc_Stop() returns the status.
 */
int32_t PCF2131_Rtc_Stop(pcf2131_sensorhandle_t *pSensorHandle, D_P3H2x4x_Handle *P3H2x4xDriver);

/*! @brief       read the data
 *  @details     Print the real time data.
 *  @return      ::PCF2131_ReadData() returns the status.
 */
int32_t PCF2131_ReadData(D_P3H2x4x_Handle *P3H2x4xDriver, pcf2131_sensorhandle_t *pSensorHandle, const registerreadlist_t *pReadList,uint8_t *pBuffer);

/*! @brief       Gets the current time mode from the PCF2131 RTC.
 *  @details     Retrieves the current time mode (12-hour or 24-hour) from the PCF2131 RTC.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[out]  pmode_12_24   		Pointer to store the current time mode.
 *  @constraints This can be called any number of times only after PCF2131_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return      ::PCF2131_12h_24h_Mode_Get() returns the status.
 */
int32_t PCF2131_12h_24h_Mode_Get(D_P3H2x4x_Handle *P3H2x4xDriver, pcf2131_sensorhandle_t *pSensorHandle, Mode12h_24h *pmode_12_24);

/*! @brief       Sets the time mode for the PCF2131 RTC.
 *  @details     Sets the 12-hour or 24-hour mode the PCF2131 RTC.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   is_mode12h   		Mode Selection: 12 hour/24 hour.
 *  @constraints This can be called any number of times only after PCF2131_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return      ::PCF2131_12h_24h_Mode_Set() returns the status.
 */
int32_t PCF2131_12h_24h_Mode_Set(D_P3H2x4x_Handle *P3H2x4xDriver, pcf2131_sensorhandle_t *pSensorHandle, Mode12h_24h is_mode12h);

/*! @brief       Sets the 100th second mode for the PCF2131 RTC.
 *  @details     Sets the 100th second mode(Enable/Disable) for the PCF2131 RTC.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[out]  is_s100h   		100th Second Mode: Enable or Disable.
 *  @constraints This can be called any number of times only after PCF2131_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return      ::PCF2131_Sec100TH_Mode() returns the status.
 */
int32_t PCF2131_Sec100TH_Mode(D_P3H2x4x_Handle *P3H2x4xDriver, pcf2131_sensorhandle_t *pSensorHandle, S100thMode is_s100h);

/*! @brief       Gets the 100th second mode for the PCF2131 RTC.
 *  @details     Gets the 100th second mode(Enable/Disable) for the PCF2131 RTC.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[out]  is_s100h   		100th Second Mode: Enable or Disable.
 *  @constraints This can be called any number of times only after PCF2131_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return      ::PCF2131_Sec100TH_Mode_Get() returns the status.
 */
int32_t PCF2131_Sec100TH_Mode_Get(D_P3H2x4x_Handle *P3H2x4xDriver, pcf2131_sensorhandle_t *pSensorHandle, S100thMode *s100_mode);

/*! @brief       Sets the time from the PCF2131 RTC.
 *  @details     Sets the current time in the RTC registers.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   time    			Pointer to the time data to be set.
 *  @constraints This can be called any number of times only after PCF2131_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return      ::PCF2131_SetTime() returns the status.
 */
int32_t PCF2131_SetTime(D_P3H2x4x_Handle *P3H2x4xDriver, pcf2131_sensorhandle_t *pSensorHandle, pcf2131_timedata_t *time);

/*! @brief       Get the current time from the PCF2131 RTC.
 *  @details     Reads the current time from sensor registers.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   pcf2131timedata    Pointer to the list of registers read operationsfor time data.
 *  @param[out]  time   			Pointer to store the read time data.
 *  @constraints This can be called any number of times only after PCF2131_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return      ::PCF2131_GetTime() returns the status.
 */
int32_t PCF2131_GetTime(D_P3H2x4x_Handle *P3H2x4xDriver, pcf2131_sensorhandle_t *pSensorHandle, const registerreadlist_t *pcf2131timedata, pcf2131_timedata_t *time );


#endif /* PCF2131_RTC_DRV_H_ */
