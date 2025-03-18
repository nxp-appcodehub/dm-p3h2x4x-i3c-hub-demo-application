/*
 * Copyright 2019, 2022-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*  Standard C Included Files */
#include <P3H2x4x.h>
#include <P3H2x4x_drv.h>
#include <string.h>
/*  SDK Included Files */
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console_cmsis.h"
#include "fsl_i3c.h"
#include "hardware_init.h"
//-----------------------------------------------------------------------
// CMSIS Includes
//-----------------------------------------------------------------------
#include "issdk_hal.h"
#include "Driver_Common.h"
#include "fsl_i3c_cmsis.h"
#include "systick_utils.h"
#include "P3H2x4x_Temp_sensor_drv.h"
#include "pcf2131_rtc_drv.h"

uint8_t g_master_txBuff[I3C_DATA_LENGTH];
uint8_t g_master_rxBuff[I3C_DATA_LENGTH];

pcf2131_timedata_t timeData;
p3h2x4x_manual_config Set_manual_config;
bool checkIBI = false;
bool in_I2C_mode = true;
bool is_i2c_only = false;
bool is_smbus_only = false;
bool is_i3c_only = false;

int target_port =0;
float temperature = 0;
extern uint8_t payload_byte_one, payload_byte_two;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*! @brief Address of Second Register for Time. */
const registerreadlist_t pcf2131timedata[] = {{.readFrom = PCF2131_100TH_SECOND, .numBytes = PCF2131_TIME_SIZE_BYTE}, __END_READ_DATA__};


/*******************************************************************************
 * Variables
 ******************************************************************************/

void user_ibi_callback_data_rcv(uint8_t *buf, uint8_t len)
{
	uint8_t i;

	PRINTF("\r\n IBI user callback \r\n");

	for( i=0; i < len; ++i )
		PRINTF("\r\n data  :- %d  \r\n", *(buf + i));

}
/*******************************************************************************
 * Code
 ******************************************************************************/
uint32_t EnDis_IBI(D_P3H2x4x_Handle *P3H2x4xDriver){

	uint32_t status;
	uint8_t options;

	PRINTF("\r\n 1. Enable IBI \r\n");
	PRINTF("\r\n 2. Disable IBI \r\n");
	PRINTF("\r\n 3. Check IBI \r\n");

	PRINTF("\r\n Enter your choice :- ");
	SCANF("%d",&options);
	PRINTF("%d\r\n",options);

	if(options < 1 || options > 3){
		PRINTF("\r\n Invalid Value, Please enter correct value\r\n");
		return -1;
	}

	if(options == 1){

		status = P3H2x4x_Enable_Disable_IBI(P3H2x4xDriver);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n \033[32m IBI Enabled Failed \033[37m\r\n");
			return -1;
		}
		P3H2x4xDriver->usercallbackibi = user_ibi_callback_data_rcv;

		PRINTF("\r\n \033[32m IBI Enabled \033[37m\r\n");

	}else if(options == 2){

		status = P3H2x4x_Enable_Disable_IBI(P3H2x4xDriver);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n \033[32m IBI Disabled Failed \033[37m\r\n");
			return -1;
		}
		PRINTF("\r\n \033[32m IBI Disabled \033[37m\r\n");
		P3H2x4xDriver->usercallbackibi = NULL;
	}
	else{

		if(P3H2x4xDriver->ibi_info.is_ibi){
			PRINTF("\r\n \033[32m IBI is generated \033[37m\r\n");
			P3H2x4x_Read_IBI_data(P3H2x4xDriver, &payload_byte_one, &payload_byte_two);
			P3H2x4xDriver->ibi_info.is_ibi = false;
		}
		else
			PRINTF("\r\n \033[32m No IBI \033[37m\r\n");
	}
}

int SetCntrlHubNw(D_P3H2x4x_Handle *P3H2x4xDriver, i3c_master_transfer_t *transfer){

	uint32_t status;
	uint8_t options, P3H2840_SLAVE_ADDR;
	uint8_t DYNAMIC_ADDR = 0x00;
	static bool once_done = 0;
	status_t result;

	PRINTF("\r\n Select \033[32mController-Hub\033[37m connection mode! \r\n");
	PRINTF("\r\n 1. I2C Mode \r\n");
	PRINTF("\r\n 2. I3C mode \r\n");

	do{
		PRINTF("\r\n Enter your choice :- ");
		SCANF("%d",&options);
		PRINTF("\r\n %d\r\n",options);
		if(options < 1 || options > 2){
			PRINTF("\r\n Invalid Value, Please enter correct value\r\n");
		}
	}while(options < 1 || options > 2);

	if(options == 1){

		PRINTF("\r\n \033[32m Only I2C and SMBUS Target devices are supported!!! \033[37m \r\n");

		PRINTF("\r\n I2C static slave address will be used.\r\n");

		P3H2840_SLAVE_ADDR = STATIC_ADDR;

		status = P3H2x4x_Initialize(P3H2x4xDriver, &I3C_S_DRIVER, I3C_S_DEVICE_INDEX, P3H2840_SLAVE_ADDR, 0);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n P3H2840 I3C HUB Initialization Failed\r\n");
			return -1;
		}
		PRINTF("\r\n P3H2840 I3C HUB Initialization completed\r\n");

		if((P3H2441) || (P3H2841))
			P3H2x41_config(P3H2x4xDriver);

		once_done = 1;
	}
	else{
		if(once_done){

			PRINTF("\r\n I3C Dynamic Address Assignment will occur!\r\n");

			PRINTF("\r\n \033[32m Only SMBUS and I3C Target devices are supported!!! \033[37m \r\n");

			PRINTF("\r\n Assign I3C Dynamic Address to Hub!\r\n");

			uint8_t hexValue;
			PRINTF("\r\n Enter a hexadecimal value(eg. 15, 20 etc.)\r\n :- ");
			SCANF("%x\r\n",&hexValue);
			PRINTF("0x%x\r\n",hexValue);

			DYNAMIC_ADDR = hexValue;

			if(SILICON_A0 == 0){

				status = P3H2x4x_Initialize(P3H2x4xDriver, &I3C_S_DRIVER, I3C_S_DEVICE_INDEX, DYNAMIC_ADDR, 1);
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n P3H2840 I3C HUB Initialization Failed\r\n");
					return -1;
				}
			}

			status = P3H2x4x_Dynamic_addr_assgmt_without_rest(STATIC_ADDR, DYNAMIC_ADDR);
			if (SENSOR_ERROR_NONE != status)
			{
				PRINTF("\r\n Dynamic address assignment Failed\r\n");
				return -1;
			}
			PRINTF("\r\nI3C dynamic address assigned to Hub\r\n");

			if(SILICON_A0 == 1){

				status = P3H2x4x_Initialize(P3H2x4xDriver, &I3C_S_DRIVER, I3C_S_DEVICE_INDEX, DYNAMIC_ADDR, 1);
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n P3H2840 I3C HUB Initialization Failed\r\n");
					return -1;
				}
				PRINTF("\r\n P3H2840 I3C HUB Initialization completed\r\n");
			}
		}

		else{

			PRINTF("\r\n I3C Dynamic Address Assignment will occur!\r\n");

			PRINTF("\r\n \033[32m Only SMBUS and I3C Target devices are supported!!! \033[37m \r\n");

			PRINTF("\r\n Assign I3C Dynamic Address to Hub!\r\n");

			uint8_t hexValue;
			PRINTF("\r\n Enter a hexadecimal value(eg. 15, 20 etc.)\r\n :- ");
			SCANF("%x\r\n",&hexValue);
			PRINTF("0x%x\r\n",hexValue);

			DYNAMIC_ADDR = hexValue;

			if(SILICON_A0 == 0){

				status = P3H2x4x_Initialize(P3H2x4xDriver, &I3C_S_DRIVER, I3C_S_DEVICE_INDEX, DYNAMIC_ADDR, 1);
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n P3H2840 I3C HUB Initialization Failed\r\n");
					return -1;
				}
				P3H2x4xDriver->ibi_info.is_ibi = false;
			}

			status = P3H2x4x_Dynamic_addr_assgmt_with_rest(STATIC_ADDR, DYNAMIC_ADDR);
			if (SENSOR_ERROR_NONE != status)
			{
				PRINTF("\r\n Dynamic address assignment Failed\r\n");
				return -1;
			}
			PRINTF("\r\nI3C dynamic address assigned to Hub\r\n");

			if(SILICON_A0 == 1){

				status = P3H2x4x_Initialize(P3H2x4xDriver, &I3C_S_DRIVER, I3C_S_DEVICE_INDEX, DYNAMIC_ADDR, 1);
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n P3H2840 I3C HUB Initialization Failed\r\n");
					return -1;
				}
				PRINTF("\r\n P3H2840 I3C HUB Initialization completed\r\n");
			}
			once_done = 1;
		}
	}
	return 0;
}

int SetManualConfig(D_P3H2x4x_Handle *P3H2x4xDriver, p3t1755_sensorhandle_t *p3t1755Driver, p3h2x4x_manual_config *Set_manual_config, bool is_i2c_only, bool is_smbus_only, bool is_i3c_only){

	int32_t status;
	uint32_t temp;

	do{
		PRINTF("\r\n Enter number of bytes to write(eg. 1, 3)- ");
		SCANF("%d",&temp);
		PRINTF("%d\r\n",temp);

	}
	while(temp <= 0);
	Set_manual_config->bytesToWrite = temp;


	do{
		PRINTF("\r\n Enter register number to write- ");
		SCANF("%d",&temp);
		PRINTF("0x%d\r\n",temp);

	}
	while(temp < 0);
	Set_manual_config->regsiterToWrite = temp;


	do{
		for(int i =0; i<(Set_manual_config->bytesToWrite); i++){

			PRINTF("\r\n Enter data to write- ");
			SCANF("%d",&temp);
			if(is_smbus_only)
				Set_manual_config->dataToWrite[i+1] = (uint16_t)temp;
			else
				Set_manual_config->dataToWrite[i] = (uint16_t)temp;
			PRINTF("%d\r\n", temp);
		}
	}
	while(temp < 0);

	if(is_smbus_only){

		do{
			PRINTF("\r\n Enter target port number (0-7):- ");
			SCANF("%d",&temp);
			PRINTF("%d\r\n",temp);
			if(temp <0 || temp >7){
				PRINTF("\r\n Invalid input \r\n");
			}

		}
		while(temp <0 || temp >7);
		Set_manual_config->target_port = temp;
		Set_manual_config->bytesToWrite += 1;
		Set_manual_config->dataToWrite[0] = Set_manual_config->regsiterToWrite;
	}

	if(is_i2c_only || is_i3c_only){


		do{
			PRINTF("\r\n Enter number of bits you want to shift by(eg. 0, 1, 2):- ");
			SCANF("%d",&temp);
			PRINTF("%d\r\n",temp);

		}
		while(temp < 0);
		Set_manual_config->shiftBy = temp;

		do{
			PRINTF("\r\n Enter masking value :- ");
			SCANF("%x",&temp);
			PRINTF("0x%x\r\n",temp);

		}
		while(temp < 0);
		Set_manual_config->mask = temp;
	}

	return 0;

}

/*! -----------------------------------------------------------------------
 *  @brief       Take input from User to enter Thigh value in celcius
 *  @details     This static function sets THigh Value as provided by user and print it back
 *  @return      void  There is no return value.
 *  -----------------------------------------------------------------------*/
void fModifyThighValue(D_P3H2x4x_Handle *P3H2x4xDriver, p3t1755_sensorhandle_t *p3t1755Driver, int target_port, uint8_t target_address,  bool in_I2C_mode)
{
	float fTemp, fTempLow;

	P3T1755_GetTHigh(P3H2x4xDriver, p3t1755Driver, target_port,
						target_address, in_I2C_mode, &fTemp);

	PRINTF("\r\nCurrent T-High(in degree celsius) = %f,\r\nEnter T-High value in Celcius (should be less than 125 C)\r\n->", fTemp);
	SCANF("%f", &fTemp);

	if (fTemp > P3T1755DP_MAX_THIGH_VALUE_CEL || fTemp < P3T1755DP_MIN_TLOW_VALUE_CEL)
	{
		PRINTF("\r\nT-High should be in range from -40 C to 125 C\r\n");
		return;
	}

	P3T1755_GetTLow(P3H2x4xDriver, p3t1755Driver, target_port,
						target_address, in_I2C_mode, &fTempLow);
	if (fTemp <= fTempLow)
	{
		PRINTF("\r\nT-High should be greater than T-Low\r\n");
		return;

	}
	P3T1755_SetTHigh(P3H2x4xDriver, p3t1755Driver, target_port,
						target_address, in_I2C_mode, fTemp);

	P3T1755_GetTHigh(P3H2x4xDriver, p3t1755Driver, target_port,
						target_address, in_I2C_mode, &fTemp);

	PRINTF("\r\n \033[32m Set T-High(in degree celsius) = %f  \033[37m\r\n", fTemp);

}

/*! -----------------------------------------------------------------------
 *  @brief       Take input from User to enter TLow value in celcius
 *  @details     This static function sets TLow Value as provided by user and print it back
 *  @return      void  There is no return value.
 *  -----------------------------------------------------------------------*/
void fModifyTlowValue(D_P3H2x4x_Handle *P3H2x4xDriver, p3t1755_sensorhandle_t *p3t1755Driver, int target_port, uint8_t target_address,  bool in_I2C_mode)
{
	float fTemp, fTempHigh;

	P3T1755_GetTLow(P3H2x4xDriver, p3t1755Driver, target_port,
						target_address, in_I2C_mode, &fTemp);


	PRINTF("\r\nCurrent T-Low(in degree celsius) = %f,\r\nEnter T-Low value in Celsius (should not be less than -40 C)\r\n->", fTemp);
	SCANF("%f", &fTemp);
	if (fTemp > P3T1755DP_MAX_THIGH_VALUE_CEL || fTemp < P3T1755DP_MIN_TLOW_VALUE_CEL)
	{
		PRINTF("\r\nT-Low should be in range from -40 C to 125 C\r\n");
		return;
	}

	P3T1755_GetTHigh(P3H2x4xDriver, p3t1755Driver, target_port,
						target_address, in_I2C_mode, &fTempHigh);
	if (fTemp >= fTempHigh)
	{
		PRINTF("\r\nT-Low should be less than T-High\r\n");
		return;

	}

	P3T1755_SetTLow(P3H2x4xDriver, p3t1755Driver, target_port,
							target_address, in_I2C_mode, fTemp);
	P3T1755_GetTLow(P3H2x4xDriver, p3t1755Driver, target_port,
							target_address, in_I2C_mode, &fTemp);

	PRINTF("\033[32m Set TLow(in degree celsius) = %f \033[37m\r\n", fTemp);

}

void checkAlert(D_P3H2x4x_Handle *P3H2x4xDriver, p3t1755_sensorhandle_t *p3t1755Driver, int target_port, uint8_t target_address,  bool in_I2C_mode)
{
	float fTemp, fTempHigh, fTempLow;

	P3T1755_GetTemp(P3H2x4xDriver, p3t1755Driver, target_port,
							target_address, in_I2C_mode, &temperature);
	P3T1755_GetTHigh(P3H2x4xDriver, p3t1755Driver, target_port,
						target_address, in_I2C_mode, &fTempHigh);
	P3T1755_GetTLow(P3H2x4xDriver, p3t1755Driver, target_port,
						target_address, in_I2C_mode, &fTempLow);

	if(temperature <= fTempLow || temperature >= fTempHigh){

		PRINTF("\r\n \033[32m Alert Generated! \033[37m \r\n");

	}
	else{

		PRINTF("\r\n \033[32m No Alert Generated! \033[37m \r\n");
	}
}

int SetGet_TempSensor(D_P3H2x4x_Handle *P3H2x4xDriver, p3t1755_sensorhandle_t *p3t1755Driver, uint8_t target_port, uint8_t target_address,  bool in_I2C_mode){

	int options;
	float fTemp, fTempHigh, fTemplow;
	uint8_t buff[2];
	uint32_t status;
	int TS=1;

	do{
		PRINTF("\r\n Test Temperature Sensor Configurations!\r\n");

		PRINTF("\r\n 1. Read current temperature \r\n");
		PRINTF("\r\n 2. Get/Set T-low \r\n");
		PRINTF("\r\n 3. Get/Set T-high \r\n");
		PRINTF("\r\n 4. Check alert \r\n");
		PRINTF("\r\n 5. Target Reset and Exit\r\n");

		PRINTF("\r\n Enter your choice :- ");
		SCANF("%d",&options);
		PRINTF("%d\r\n",options);

		if(options < 1 || options > 5){
			PRINTF("\r\n Invalid Value, Please enter correct value\r\n");
			return -1;
		}

		switch (options)
		{
			case 1: //read temperature
				status = P3T1755_GetTemp(P3H2x4xDriver, p3t1755Driver, target_port,
						target_address, in_I2C_mode, &temperature);

				PRINTF("\r\n \033[32m Current Temperature(in degree celsius) = %f \033[37m\r\n", temperature);
				break;
			case 2: //get/set tlow
				fModifyTlowValue(P3H2x4xDriver, p3t1755Driver, target_port,
						target_address, in_I2C_mode);
				break;
			case 3: //Get/set thigh
				fModifyThighValue(P3H2x4xDriver, p3t1755Driver, target_port,
						target_address, in_I2C_mode);
				break;
			case 4: //check alert
				checkAlert(P3H2x4xDriver, p3t1755Driver, target_port,
						target_address, in_I2C_mode);
				break;
			case 5: //exit
				if(P3H2x4xDriver->in_i3c_mode) {
					P3H2x4x_Target_device_reset();
					PRINTF("Target Reset Completed!\r\n");
				}
				else
					PRINTF("No Target Reset Needed in I2C/SMBUS!\r\n");
				PRINTF("Bye....!");
				TS = 0;
				break;
			default:
				PRINTF("Invalid option selected");
				break;
		}
	}while(TS);

	return 0;
}

/*!@brief        Get Time data.
 *  @details     Get Time and date from corresponding Registers and
 *  				store back in internal structure.
 *  @param[in]   pcf2131Driver   Pointer to sensor handle structure.
 *  @param[out]  timeData   		Structure holding time data.
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      Status of the operation.
 */
int32_t getTime(D_P3H2x4x_Handle *P3H2x4xDriver, pcf2131_sensorhandle_t *pcf2131Driver, pcf2131_timedata_t *timeData)
{
	int32_t status;

	/* Get Time */
	status = PCF2131_GetTime(P3H2x4xDriver, pcf2131Driver, (const registerreadlist_t *)&pcf2131timedata , timeData);

	if (SENSOR_ERROR_NONE != status)
	{
		PRINTF("\r\n Get Time Failed\r\n");
		return -1;
	}
	return SENSOR_ERROR_NONE;
}

/*!@brief        Print Time.
 *  @details     Print Time set by user.
 *  @param[in]   timeAlarm   Structure holding time data.
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No
 */
void printTime(pcf2131_timedata_t timeData)
{
	PRINTF("\r\n TIME :- %02d:%02d:%02d:%02d", timeData.hours, timeData.minutes, timeData.second, timeData.second_100th );

	if(timeData.ampm == AM)
		PRINTF(" AM\r\n");
	else if(timeData.ampm == PM)
		PRINTF(" PM\r\n");
	else
		PRINTF(" 24H Mode\r\n");

	PRINTF("\r\n DATE [DD/MM/YY]:- %02d/%02d/%02d\r\n",timeData.days, timeData.months, timeData.years );

	switch(timeData.weekdays)
	{
	case 0:
		PRINTF("\r\n SUNDAY\r\n");
		break;
	case 1:
		PRINTF("\r\n MONDAY\r\n");
		break;
	case 2:
		PRINTF("\r\n TUESDAY\r\n");
		break;
	case 3:
		PRINTF("\r\n WEDNESDAY\r\n");
		break;
	case 4:
		PRINTF("\r\n THURSDAY\r\n");
		break;
	case 5:
		PRINTF("\r\n FRIDAY\r\n");
		break;
	case 6:
		PRINTF("\r\n SATURDAY\r\n");
		break;
	default:
		break;
	}
}

/*!@brief        Set mode (12h/24h).
 *  @details     set 12 hour / 24 hour format.
 *  @param[in]   pcf2131Driver   Pointer to sensor handle structure.
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No
 */
void setmode12h_24h(D_P3H2x4x_Handle *P3H2x4xDriver, pcf2131_sensorhandle_t *pcf2131Driver)
{
	uint8_t temp;
	int32_t status;

	PRINTF("\r\n 1. 12H mode\r\n");
	PRINTF("\r\n 2. 24H mode\r\n");

	PRINTF("\r\n Enter your choice :- ");
	do{
		SCANF("%d",&temp);
		PRINTF("%d\r\n",temp);
		if(temp < 0 || temp > 2)
			PRINTF("\r\n Invalid Value, Please enter correct value\r\n");
	}
	while(temp < 0 || temp > 2);

	switch(temp)
	{
	case 1: /* Set 12h clock format */
		status = PCF2131_12h_24h_Mode_Set(P3H2x4xDriver, pcf2131Driver, mode12H);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n 12h clock format Set Failed\r\n");
		}
		else
			PRINTF("\r\n 12h mode is set\r\n");
		break;

	case 2: /* Set 24h clock format */
		status = PCF2131_12h_24h_Mode_Set(P3H2x4xDriver, pcf2131Driver, mode24H);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n 24h clock format Set Failed\r\n");
		}
		else
			PRINTF("\r\n 24h mode is set\r\n");
		break;
	default:
		PRINTF("\r\n Invalid choice\r\n");
		break;
	}
}

void  set100thSecond(D_P3H2x4x_Handle *P3H2x4xDriver, pcf2131_sensorhandle_t *pcf2131Driver)
{
	uint8_t temp;
	int32_t status;
	PRINTF("\r\n 1. Enable centi-seconds granularity\r\n");
	PRINTF("\r\n 2. Disable centi-seconds granularity\r\n");

	PRINTF("\r\n Enter your choice :- ");
	do{
		SCANF("%d",&temp);
		PRINTF("%d\r\n",temp);
		if(temp < 0 || temp > 2)
			PRINTF("\r\n Invalid Value, Please enter correct value\r\n");
	}
	while(temp < 0 || temp > 2);

	switch(temp)
	{
	case 1:
		status = PCF2131_Sec100TH_Mode(P3H2x4xDriver,pcf2131Driver,s100thEnable);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n centi seconds Enabled Failed\r\n");
		}
		else
			PRINTF("\r\n centi seconds Enabled\r\n");
		break;
	case 2:
		status = PCF2131_Sec100TH_Mode(P3H2x4xDriver,pcf2131Driver,s100thDisable);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n centi seconds Disabled Failed\r\n");
		}
		else
			PRINTF("\r\n centi seconds Disabled\r\n");
		break;
	default:
		PRINTF("\r\nInvalid Option\r\n");
		break;

	}
}

/*!@brief        Set Time.
 *  @details     Set Time by taking input from user.
 *  @param[in]   pcf2131Driver   Pointer to spi sensor handle structure.
 *  @param[in]   timeData        Time to be set by user.
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      Status of the operation.
 */
int32_t setTime(D_P3H2x4x_Handle *P3H2x4xDriver, pcf2131_sensorhandle_t *pcf2131Driver, pcf2131_timedata_t *timeData)
{
	int32_t status;
	uint8_t temp;
	Mode12h_24h mode12_24;
	S100thMode s100thmode;

	/* Get Days from User and update its internal Time Structure */
	do{
		PRINTF("\r\n Enter Day value between 1 to 31 :- ");
		SCANF("%d",&temp);
		PRINTF("%d\r\n",temp);
		if(temp < 0 || temp > 31)
			PRINTF("\r\nInvalid Value, Please enter correct value\r\n");
	}
	while(temp < 1 || temp > 31);
	timeData->days = temp;

	/* Get Months from User and update its internal Time Structure */
	do{
		PRINTF("\r\n Enter Month value between 1 to 12 :- ");
		SCANF("%d",&temp);
		PRINTF("%d\r\n",temp);
		if(temp < 1 || temp > 12)
			PRINTF("\r\n Invalid Value, Please enter correct value\r\n");
	}
	while(temp < 1 || temp > 12);
	timeData->months = temp;

	/* Get Years from User and update its internal Time Structure */
	do{
		PRINTF("\r\n Enter Year value between 0 to 99 :- ");
		SCANF("%d",&temp);
		PRINTF("%d\r\n",temp);
		if(temp < 0 || temp > 99)
			PRINTF("\r\n Invalid Value, Please enter correct value\r\n");
	}
	while(temp < 0 || temp > 99);
	timeData->years = temp;

	/* Get Weekdays from User and update its internal Time Structure */
	/* 0 - Sunday
	 * 1 - Monday
	 * 2 - Tuesday
	 * 3 - Wednesday
	 * 4 - Thursday
	 * 5 - Friday
	 * 6 - Saturday
	 */
	do{

		PRINTF("\r\n Enter Weekday value between 0 to 6 (0 - Sunday.....6 - Saturday)-");
		SCANF("%d",&temp);
		PRINTF("%d\r\n",temp);


		if(temp < 0 || temp > 6)
			PRINTF("\r\n Invalid Value, Please enter correct value\r\n");
	}
	while(temp < 0 || temp > 6);
	timeData->weekdays = temp;
	/* Get hour from User and update its internal Time Structure */
	do
	{
		/* Get 12h/24h hour format */
		PCF2131_12h_24h_Mode_Get(P3H2x4xDriver, pcf2131Driver, &mode12_24);
		if(mode12_24 ==  mode24H)
			PRINTF("\r\n Enter Hour value between 0 to 23 :- ");
		else
			PRINTF("\r\n Enter Hour value between 1 to 12 :- ");
		SCANF("%d",&temp);
		PRINTF("%d\r\n",temp);

		if((mode12_24 ==  mode24H) && (temp < 0 || temp > 23))
			PRINTF("\r\nInvalid Value\r\n");

		if((mode12_24 ==  mode12H) && (temp < 1 || temp > 12))
			PRINTF("\r\n Invalid Value, Please enter correct value\r\n");
	}
	while((mode12_24 ==  mode24H) && (temp < 0 || temp > 23) ||
			(mode12_24 ==  mode12H) && (temp < 1 || temp > 12));
	timeData->hours = temp;

	/* Get Minutes from User and update its internal Time Structure */
	do{
		PRINTF("\r\n Enter Minute value between 0 to 59 :- ");
		SCANF("%d",&temp);
		PRINTF("%d\r\n",temp);
		if(temp < 0 || temp > 59)
			PRINTF("\r\n Invalid Value, Please enter correct value\r\n");
	}
	while(temp < 0 || temp > 59);
	timeData->minutes = temp;

	/* Get Second from User and update its internal Time Structure */
	do{
		PRINTF("\r\n Enter Second value between 0 to 59 :- ");
		SCANF("%d",&temp);
		PRINTF("%d\r\n",temp);
		if(temp < 0 || temp > 59)
			PRINTF("\r\n Invalid Value, Please enter correct value\r\n");
	}
	while(temp < 0 || temp > 59);
	timeData->second = temp;

	/* Get 100th Second mode*/
	status = PCF2131_Sec100TH_Mode_Get(P3H2x4xDriver, pcf2131Driver, &s100thmode);
	if (SENSOR_ERROR_NONE != status)
	{
		PRINTF("\r\n Get 100th Second Failed\r\n");
		return -1;
	}
	if(s100thmode == s100thEnable)
	{
		/* Get 1/100th Second from User and update its internal Time Structure */
		do
		{
			PRINTF("\r\n Enter 1/100th Second value between 0 to 99 :- ");
			SCANF("%d",&temp);
			PRINTF("%d\r\n",temp);
			if(temp < 0 || temp > 99)
				PRINTF("\r\n Invalid Value, Please enter correct value\r\n");
		}
		while(temp < 0 || temp > 99);
		timeData->second_100th = temp;
	}


	/* Get 12h/24h hour format */
	PCF2131_12h_24h_Mode_Get(P3H2x4xDriver, pcf2131Driver, &mode12_24);
	if(mode12_24 !=  mode24H)
	{
		PRINTF("\r\n 1. AM \r\n");
		PRINTF("\r\n 2. PM \r\n");

		PRINTF("\r\n Enter Your choice :- ");
		do
		{
			SCANF("%d",&temp);
			PRINTF("%d\r\n",temp);
			if(temp < 0 || temp > 2)
				PRINTF("\r\n Invalid Value, Please enter correct value\r\n");
		}
		while(temp < 0 || temp > 2);

		switch(temp)  /* Update AM/PM */
		{
		case 1:
			timeData->ampm =  AM;
			break;
		case 2:
			timeData->ampm = PM;
			break;
		default:
			PRINTF("\r\nInvalid choice\r\n");
			break;
		}
	}
	else
		timeData->ampm = h24;

	/* Stop RTC */
	status = PCF2131_Rtc_Stop(pcf2131Driver, P3H2x4xDriver);
	if (SENSOR_ERROR_NONE != status)
	{
		PRINTF("\r\n RTC Stop Failed\r\n");
		return -1;
	}

	/* Set Time */
	status = PCF2131_SetTime(P3H2x4xDriver, pcf2131Driver, timeData);
	if (SENSOR_ERROR_NONE != status)
	{
		PRINTF("\r\n Set Time Failed\r\n");
		return -1;
	}

	/* Start RTC */
	status = PCF2131_Rtc_Start(pcf2131Driver, P3H2x4xDriver);
	if (SENSOR_ERROR_NONE != status)
	{
		PRINTF("\r\n RTC Start Failed\r\n");
		return -1;
	}
	return 0;
}

int SetGetRTC(D_P3H2x4x_Handle *P3H2x4xDriver, pcf2131_sensorhandle_t *pcf2131Driver, int target_port){

	uint32_t status;
	uint8_t options, suboptions;
	int rtc = 1;

	do{
		PRINTF("\r\n 1. RTC Start \r\n");
		PRINTF("\r\n 2. RTC Stop \r\n");
		PRINTF("\r\n 3. Get Time \r\n");
		PRINTF("\r\n 4. Set Time \r\n");
		PRINTF("\r\n 5. Exit \r\n");

		PRINTF("\r\n Enter your choice :- ");
		SCANF("%d",&options);
		PRINTF("%d\r\n",options);

		if((options < 1) || (options > 5)){
			PRINTF("\r\n Invalid Value, Please enter correct value\r\n");
			return -1;
		}
		switch (options)
		{
			case 1: //RTC Start
				status = PCF2131_Rtc_Start(pcf2131Driver, P3H2x4xDriver);
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n \033[32m RTC Start Failed \033[37m\r\n");
					continue;
				}
				PRINTF("\r\n \033[32m RTC Started \033[37m\r\n");
				break;
			case 2: //RTC Stop
				status = PCF2131_Rtc_Stop(pcf2131Driver, P3H2x4xDriver);
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n \033[32m RTC Stop Failed \033[37m\r\n");
					continue;
				}
				PRINTF("\r\n \033[32m RTC Stopped \033[37m\r\n");
				break;
			case 3: //Get time

				memset(&timeData, '0', sizeof(pcf2131_timedata_t));
				status = getTime(P3H2x4xDriver, pcf2131Driver, &timeData);

				if (SENSOR_ERROR_NONE  == status)
				{
					printTime(timeData);  // print time data
					break;
				}
				PRINTF("Get time failed \r\n");
			case 4: //Set time

				setmode12h_24h(P3H2x4xDriver, pcf2131Driver);  /* Set 12h/24h mode */
				set100thSecond(P3H2x4xDriver, pcf2131Driver);  /* Set centi-seconds mode */
				memset(&timeData, '0',sizeof(pcf2131_timedata_t));
				setTime(P3H2x4xDriver, pcf2131Driver, &timeData);    // set time data
				break;
			case 5: //exit
				PRINTF("Bye....!");
				rtc = 0;
				break;
			default:
				PRINTF("Invalid option selected");
				break;
		}
	}while(rtc == 1);

	return 0;
}

int Set_TS_working_mode(D_P3H2x4x_Handle *P3H2x4xDriver , p3t1755_sensorhandle_t *p3t1755Driver, int target_port){

	uint32_t options,suboptions, status;
	uint8_t TEMP_SENSOR_DYNAMIC_ADDR, hexValue;

	PRINTF("\r\n Please Make sure, default configurations match with input selected below by checking <P3H2X4X_config.h> \r\n");

	PRINTF("\r\n 1. SMBus \r\n");
	PRINTF("\r\n 2. I3C \r\n");

	PRINTF("\r\n Enter your choice :- ");
	SCANF("%d",&options);
	PRINTF("%d\r\n",options);

	if((options < 1) || (options > 2)){
		PRINTF("\r\n Invalid Value, Please enter correct value\r\n");
		return -1;
	}

	if(options == 1)
	{
		/*! Initialize P3T1755DP sensor driver. */
		status = P3T1755_Initialize(p3t1755Driver, &I3C_S_DRIVER, 2, TEMP_SENSOR_STATIC_ADDR);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n Sensor Initialization Failed\r\n");
			return -1;
		}
		PRINTF("\r\n P3T1755 Sensor Initialization completed\r\n");

		in_I2C_mode = true;

		SetGet_TempSensor(P3H2x4xDriver, p3t1755Driver, target_port, TEMP_SENSOR_STATIC_ADDR, in_I2C_mode);
	}
	else if(options == 2 && (P3H2x4xDriver->in_i3c_mode == true))
	{

		PRINTF("\r\n Assign Dynamic address to Temperature sensor ");

		PRINTF("\r\n Enter a hexadecimal value(eg. 15, 20 etc.)\r\n :- ");
		SCANF("%x\r\n",&hexValue);
		PRINTF("0x%x\r\n",hexValue);

		TEMP_SENSOR_DYNAMIC_ADDR = hexValue;

		if(P3H2x4xDriver->slaveAddress == TEMP_SENSOR_DYNAMIC_ADDR){

			PRINTF("\r\n Wrong address entered, select unique dynamic address \r\n");
			return -1;
		}

		status = P3H2x4x_Dynamic_addr_assgmt_without_rest(TEMP_SENSOR_STATIC_ADDR, TEMP_SENSOR_DYNAMIC_ADDR);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n Temperature sensor Dynamic address assignment Failed\r\n");
			return -1;
		}
		PRINTF("\r\n Temperature sensor Dynamic address assignment completed\r\n");

		status = P3T1755_Initialize(p3t1755Driver, &I3C_S_DRIVER, 2, TEMP_SENSOR_DYNAMIC_ADDR);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n P3T1755 Temperature Sensor Initialization Failed\r\n");
			return -1;
		}
		PRINTF("\r\n P3T1755 Temperature Sensor Initialization completed\r\n");

		in_I2C_mode = false;

		SetGet_TempSensor(P3H2x4xDriver, p3t1755Driver, target_port, TEMP_SENSOR_DYNAMIC_ADDR, in_I2C_mode);
	}
	else{
		PRINTF("\r\n Wrong input selected! \r\n");
	}

	return status;
}

int SetTSmode(D_P3H2x4x_Handle *P3H2x4xDriver, p3t1755_sensorhandle_t *p3t1755Driver){

	uint32_t status;
	status_t result = kStatus_Success;
    i3c_master_transfer_t masterXfer;
    int32_t TARGET_DYNAMIC_ADDR , HUB_DYNAMIC_ADDR, TEMP_SENSOR_DYNAMIC_ADDR  = 0x00;
	uint8_t options, suboptions, P3H2840_SLAVE_ADDR;

	uint8_t status_test;
	uint8_t target_port_status = I3C_HUB_TP0_SMBUS_AGNT_STS + target_port;

	PRINTF("\r\n \033[32m Two temperature sensors (P3T1755) are present on I3C HUB on two different target ports!\033[37m  \r\n");

	PRINTF("\r\n Please Make sure, default configurations match with input selected below by checking <P3H2X4X_config.h> \r\n");

	PRINTF("\r\n 1.  Temperature sensor 1 on port 0 (By default, In SMBUS mode) \r\n");

	PRINTF("\r\n 2.  Temperature sensor 2 on port 2 (By default, In I3C mode) \r\n");

	PRINTF("\r\n Enter your choice :- ");
	SCANF("%d",&options);
	PRINTF("%d\r\n",options);

	if((options < 1) || (options > 2)){

		PRINTF("\r\n Invalid Value, Please enter correct value\r\n");
		return -1;
	}

    if(options == 1){
    	Set_TS_working_mode(P3H2x4xDriver, p3t1755Driver, 0);
    }
	else{
    	Set_TS_working_mode(P3H2x4xDriver, p3t1755Driver, 2);
	}

    return 0;
}

int SetExtlDeviceConfig(D_P3H2x4x_Handle *P3H2x4xDriver, p3t1755_sensorhandle_t *p3t1755Driver, pcf2131_sensorhandle_t *pcf2131Driver){

	uint32_t status;
	uint8_t options, suboptions, static_addr;
	uint8_t TEMP_SENSOR_DYNAMIC_ADDR, target_port, hexValue;
	uint8_t receive_buff[100];
	int external_rd_wr = 0;

	PRINTF("\r\n \033[32m Select any option to test external devices: - \033[37m  \r\n");

	PRINTF("\r\n 1. Test external RTC device (PCF2131-ARD) in I2C mode\r\n");
	PRINTF("\r\n 2. Test external Temperature sensor(P3T1755) in I3C mode \r\n");
	PRINTF("\r\n 3. Read/write any externally connected devices \r\n");

	PRINTF("\r\n Enter your choice :- ");
	SCANF("%d",&options);
	PRINTF("%d\r\n",options);

	if(options < 1 || options > 3){

		PRINTF("\r\n Invalid Value, Please enter correct value\r\n");
		return -1;
	}

	if(options==1){

		/*! Initialize PCF2131 rtc driver. */
		status = PCF2131_Initialize(pcf2131Driver, &I3C_S_DRIVER, I3C_S_DEVICE_INDEX, PCF2131_I2C_ADDR);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n RTC Initialization Failed\r\n");
			return -1;
		}
		PRINTF("\r\n PCF2131 RTC Initialization completed\r\n");

		SetGetRTC(P3H2x4xDriver, pcf2131Driver, PCF2131_I2C_ADDR);

	}
	else if(options ==2){

		PRINTF("\r\n Assign Dynamic address to Temperature sensor ");

		PRINTF("\r\n Enter a hexadecimal value(eg. 15, 20 etc.)\r\n :- ");
		SCANF("%x\r\n",&hexValue);
		PRINTF("0x%x\r\n",hexValue);

		TEMP_SENSOR_DYNAMIC_ADDR = hexValue;

		PRINTF("\r\n Enter target port number on which P3T1755 is connected :- \r\n :- ");
		SCANF("%d\r\n",&hexValue);
		PRINTF("%d\r\n",hexValue);

		target_port = hexValue;

		if(P3H2x4xDriver->slaveAddress == TEMP_SENSOR_DYNAMIC_ADDR){

			PRINTF("\r\n Wrong address entered, select unique dynamic address \r\n");
			return -1;
		}

		status = P3H2x4x_Dynamic_addr_assgmt_without_rest(TEMP_SENSOR_STATIC_ADDR, TEMP_SENSOR_DYNAMIC_ADDR);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n Temperature sensor Dynamic address assignment Failed\r\n");
			return -1;
		}
		PRINTF("\r\n Temperature sensor Dynamic address assignment completed\r\n");

		status = P3T1755_Initialize(p3t1755Driver, &I3C_S_DRIVER, 2, TEMP_SENSOR_DYNAMIC_ADDR);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n P3T1755 Temperature Sensor Initialization Failed\r\n");
			return -1;
		}
		PRINTF("\r\n P3T1755 Temperature Sensor Initialization completed\r\n");

		SetGet_TempSensor(P3H2x4xDriver, p3t1755Driver, target_port, TEMP_SENSOR_DYNAMIC_ADDR, false);
	}
	else{

		do{
			PRINTF("\r\n Enter Static Target Address :- ");
			SCANF("%x",&static_addr);
			PRINTF("0x%x\r\n",static_addr);
		}
		while(static_addr < 0);
		Set_manual_config.static_target_address = static_addr;

		PRINTF("\r\n 1. Read/write in I2C mode\r\n");
		PRINTF("\r\n 2. Read/write in I3C mode \r\n");
		PRINTF("\r\n 3. Read/write in SMbus mode \r\n");

		PRINTF("\r\n Enter your choice :- ");
		SCANF("%d",&options);
		PRINTF("%d\r\n",options);

		if(options < 1 || options > 3){

			PRINTF("\r\n Invalid Value, Please enter correct value\r\n");
			return -1;
		}

		if(options == 1){

			is_i2c_only = true;
			do{
				PRINTF("\r\n 1.Write\r\n");
				PRINTF("\r\n 2.Read\r\n");
				PRINTF("\r\n 3.Exit \r\n");

				PRINTF("\r\n Enter your choice :- ");
				SCANF("%d",&suboptions);
				PRINTF("%d\r\n",suboptions);

				if(suboptions < 1 || suboptions > 3){

					PRINTF("\r\n Invalid Value, Please enter correct value\r\n");
					return -1;
				}
				switch(suboptions){

				case 1:
					SetManualConfig(P3H2x4xDriver, p3t1755Driver, &Set_manual_config,
							is_i2c_only, is_smbus_only, is_i3c_only);
					status = P3H2x4x_Target_I2C_block_write(P3H2x4xDriver, Set_manual_config.static_target_address, Set_manual_config.bytesToWrite,
							Set_manual_config.regsiterToWrite, Set_manual_config.dataToWrite, Set_manual_config.shiftBy, Set_manual_config.mask);
						if (ARM_DRIVER_OK != status)
						{
							PRINTF("\r\n I2C Write failed! \r\n");
							return -1;
						}
						PRINTF("\r\n I2C Write complete! \r\n");
						break;

				case 2:
					//uint8_t receive_buff[&Set_manual_config.bytesToWrite];
					status = P3H2x4x_Target_I2C_read(P3H2x4xDriver, Set_manual_config.static_target_address, Set_manual_config.bytesToWrite,
							Set_manual_config.regsiterToWrite, &receive_buff[0]);
						if (ARM_DRIVER_OK != status)
						{
							PRINTF("\r\n I2C Read failed! \r\n");
							return -1;
						}
						for(int i=0;i<Set_manual_config.bytesToWrite;i++){

							PRINTF("\r\n I2C Read Data:- %d\r\n",receive_buff[i]);
						}
						break;
				case 3:
					external_rd_wr = 1;
					break;
				default:
					PRINTF("\r\n Invalid options \r\n");
					break;
				}

			}while(external_rd_wr == 0);
		}

		else if(options == 2){

			is_i3c_only = true;
			external_rd_wr = 0;


			PRINTF("\r\n Assign Dynamic address to Temperature sensor ");

			PRINTF("\r\n Enter a hexadecimal value(eg. 15, 20 etc.)\r\n :- ");
			SCANF("%x\r\n",&hexValue);
			PRINTF("0x%x\r\n",hexValue);

			if(P3H2x4xDriver->slaveAddress == hexValue){

				PRINTF("\r\n Wrong address entered, select unique dynamic address \r\n");
				return -1;
			}

			status = P3H2x4x_Dynamic_addr_assgmt_without_rest(Set_manual_config.static_target_address, hexValue);
			if (SENSOR_ERROR_NONE != status)
			{
				PRINTF("\r\n Dynamic address assignment Failed\r\n");
				return -1;
			}
			PRINTF("\r\n Dynamic address assignment completed\r\n");

			do{
				PRINTF("\r\n 1.Write\r\n");
				PRINTF("\r\n 2.Read\r\n");
				PRINTF("\r\n 3.Target Reset and Exit \r\n");

				PRINTF("\r\n Enter your choice :- ");
				SCANF("%d",&suboptions);
				PRINTF("%d\r\n",suboptions);

				if(suboptions < 1 || suboptions > 3){

					PRINTF("\r\n Invalid Value, Please enter correct value\r\n");
					return -1;
				}
				switch(suboptions){

				case 1:
					SetManualConfig(P3H2x4xDriver, p3t1755Driver, &Set_manual_config,
							is_i2c_only, is_smbus_only, is_i3c_only);

					status = P3H2x4x_Target_I3C_block_write(P3H2x4xDriver, hexValue, Set_manual_config.bytesToWrite,
							Set_manual_config.regsiterToWrite, Set_manual_config.dataToWrite, Set_manual_config.shiftBy, Set_manual_config.mask);
						if (ARM_DRIVER_OK != status)
						{
							PRINTF("\r\n Write failed! \r\n");
							return -1;
						}
						PRINTF("\r\n Write complete! \r\n");
						break;
				case 2:
					status = P3H2x4x_Target_I3C_read(P3H2x4xDriver, hexValue, Set_manual_config.bytesToWrite,
							Set_manual_config.regsiterToWrite, &receive_buff[0]);
						if (ARM_DRIVER_OK != status)
						{
							PRINTF("\r\n Read failed! \r\n");
							return -1;
						}
						for(int i=0;i<Set_manual_config.bytesToWrite;i++){

							PRINTF("\r\n Read Data:- %d\r\n",receive_buff[i]);
						}
						break;
				case 3:
					P3H2x4x_Target_device_reset();
					PRINTF("Target Reset\r\n");
					external_rd_wr = 1;
					break;

				default:
					PRINTF("\r\n Invalid options \r\n");
					break;
				}

			}while(external_rd_wr == 0);

		}
		else{
			is_smbus_only = true;
			external_rd_wr =0;
			do{

				PRINTF("\r\n 1.Write\r\n");
				PRINTF("\r\n 2.Read\r\n");
				PRINTF("\r\n 3.Exit \r\n");

				PRINTF("\r\n Enter your choice :- ");
				SCANF("%d",&suboptions);
				PRINTF("%d\r\n",suboptions);

				if(suboptions < 1 || suboptions > 3){

					PRINTF("\r\n Invalid Value, Please enter correct value\r\n");
					return -1;
				}
				switch(suboptions){

				case 1:
					SetManualConfig(P3H2x4xDriver, p3t1755Driver, &Set_manual_config,
							is_i2c_only, is_smbus_only, is_i3c_only);
					status = P3H2x4x_Target_Smbus_Write(P3H2x4xDriver,Set_manual_config.target_port, Set_manual_config.static_target_address, Set_manual_config.bytesToWrite
							, Set_manual_config.dataToWrite);
						if (ARM_DRIVER_OK != status)
						{
							PRINTF("\r\n SMBUS Write failed! \r\n");
							return -1;
						}
						PRINTF("\r\n SMBUS Write complete! \r\n");
						break;

				case 2:

					status = P3H2x4x_Target_Smbus_Read(P3H2x4xDriver,Set_manual_config.target_port, Set_manual_config.static_target_address, (Set_manual_config.bytesToWrite -1)
							, Set_manual_config.regsiterToWrite, &receive_buff[0]);
						if (ARM_DRIVER_OK != status)
						{
							PRINTF("\r\n Read failed! \r\n");
							return -1;
						}
						for(int i=0;i<Set_manual_config.bytesToWrite -1 ;i++){

							PRINTF("\r\n Read Data:- %d\r\n",receive_buff[i]);
						}
						break;
				case 3:
					external_rd_wr = 1;
					break;

				default:
					PRINTF("\r\n Invalid Options\r\n");
					break;
				}
			}while(external_rd_wr == 0);
		}
	}
	is_i2c_only, is_smbus_only, is_i3c_only = 0;
	return 0;
}

/*!
 * @brief Main function
 */
int main(void)
{

    status_t result = kStatus_Success;
    uint32_t status;
    int ret;
    D_P3H2x4x_Handle P3H2x4xDriver;
    p3t1755_sensorhandle_t p3t1755Driver;
	pcf2131_sensorhandle_t pcf2131Driver;
    I3C_Hub_Configuration i3c_hub_config;
    char dummy;
    uint8_t charecter;
    i3c_master_transfer_t masterXfer;

    /* Attach PLL0 clock to I3C, 150MHz / 6 = 25MHz. */
    CLOCK_SetClkDiv(kCLOCK_DivI3c1FClk, 6U);
    CLOCK_AttachClk(kPLL0_to_I3C1FCLK);

    /* enable clock for GPIO*/
    CLOCK_EnableClock(kCLOCK_Gpio0);

    BOARD_InitHardware();
    BOARD_SystickEnable();

reinitialise:
    P3H2x4x_Cp_sel_pin_init();
    P3H2x4x_Reset_Device(&P3H2x4xDriver);
    P3H2x4x_set_handle(&P3H2x4xDriver);

#if SILICON_A0
	/*Errata fix*/
	status = P3H2x4x_Initialize(&P3H2x4xDriver, &I3C_S_DRIVER, I3C_S_DEVICE_INDEX, STATIC_ADDR, 0);

	if (SENSOR_ERROR_NONE != status)
	{
		PRINTF("\r\n P3H2x4x I3C HUB Initialization Failed\r\n");
		return -1;
	}
	PRINTF("\r\n P3H2x4x I3C HUB Initialization completed\r\n");

	status = P3H2x4x_Errata_fix(&P3H2x4xDriver);
	if (ARM_DRIVER_OK != status)
	{
		PRINTF("\r\n Driver errata fix failed\r\n");
		return -1;
	}
	P3H2x4xDriver.isInitialized = false;
#endif

	if((P3H2840 + P3H2841 + P3H2440 + P3H2441) != 1){

		PRINTF("\r\n \033[32m Select only one version of P3H2x4x Shield board. \033[37m \r\n");
		return -1;
	}

	/*Set controller-hub network connection mode*/
	status = SetCntrlHubNw(&P3H2x4xDriver, &masterXfer);

	if (ARM_DRIVER_OK != status)
	{
		PRINTF("\r\n Controller-HUB network connection failed\r\n");
		return -1;
	}

	//unlock protected register
	status = P3H2x4x_UnlockPrtcdReg(&P3H2x4xDriver);
	if (ARM_DRIVER_OK != status)
	{
		PRINTF("\r\n Unlocking of protected registers failed! \r\n");
		return -1;
	}

	 /*Set HUB network configurations*/
	P3H2x4x_default_Configuration(&i3c_hub_config);
	P3H2x4x_set_Configuration(&i3c_hub_config);
	status = P3H2x4x_Configure(&P3H2x4xDriver, &i3c_hub_config);

	if (ARM_DRIVER_OK != status)
	{
		PRINTF("\r\n HUB configuration setup failed\r\n");
		return -1;
	}

	 do
		{
			PRINTF("\r\n");
			PRINTF("\r\n *********** Main Menu ***************\r\n");
			PRINTF("\r\n 1. Check On-board Temperature Sensor Device (P3T1755) \r\n");
			PRINTF("\r\n 2. Check External Devices \r\n");
			PRINTF("\r\n 3. Enable/Disable/Check IBI \r\n");
			PRINTF("\r\n 4. Hub Interface Reset \r\n");
			PRINTF("\r\n 5. Device Reset \r\n");

			PRINTF("\r\n Enter your choice :- ");
			SCANF("%d",&charecter);
			PRINTF("%d\r\n",charecter);

			switch (charecter)
			{
				case 1: //on board temperature sensor
					SetTSmode(&P3H2x4xDriver,&p3t1755Driver);
					break;
				case 2: //read/write external device
					SetExtlDeviceConfig(&P3H2x4xDriver, &p3t1755Driver, &pcf2131Driver);
					break;
				case 3: //enable/disable IBI
					EnDis_IBI(&P3H2x4xDriver);
					break;
				case 4: //software reset
					status = P3H2x4x_Interface_Reset(&P3H2x4xDriver);
					if(status == SENSOR_ERROR_NONE)
					{
						PRINTF("\r\n \033[32m Hub Interface Reset completed \033[37m \r\n");
						goto reinitialise;
					}
					else{
						PRINTF("\r\n \033[32m Hub Interface Reset Failed \033[37m \r\n");
					}					
					break;
				case 5:
					P3H2x4x_Reset_Device(&P3H2x4xDriver);
					PRINTF("\r\n \033[32m Device reset completed\033[37m \r\n");
					goto reinitialise;
					break;
				default:
					PRINTF("Invalid option selected");
					break;
			}
		}
	    while(1);

	 return 0;
}
