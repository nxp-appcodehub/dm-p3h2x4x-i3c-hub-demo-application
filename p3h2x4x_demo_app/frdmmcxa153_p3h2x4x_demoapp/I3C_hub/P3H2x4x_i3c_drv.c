/*
* Copyright 2025 NXP
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include <P3H2x4x.h>
#include <P3H2x4x_drv.h>
#include "P3H2x4x_config.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"
#include "fsl_i3c_cmsis.h"
#include "register_io_i3c.h"
#include "systick_utils.h"
#include "issdk_hal.h"
#include "gpio_driver.h"
#include "Driver_GPIO.h"
//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------
bool repeatedStart = 1;
bool is_of = false;
extern bool g_masterCompletionFlag;
extern status_t g_completionStatus;
extern bool g_ibiWonFlag;
uint8_t payload_byte_one, payload_byte_two;

GENERIC_DRIVER_GPIO *pGpioDriver = &Driver_GPIO_KSDK;
ARM_DRIVER_I3C *pdriver = &Driver_I3C;
D_P3H2x4x_Handle *P3H2x4xDriver_handle;

void P3H2x4x_set_handle(D_P3H2x4x_Handle *pSensorHandle)
{
	P3H2x4xDriver_handle = pSensorHandle;
}

D_P3H2x4x_Handle * P3H2x4x_get_handle(void)
{
	return P3H2x4xDriver_handle;
}


void i3c_master_callback(I3C_Type *base, i3c_master_handle_t *handle, status_t status, void *userData)
{
     //Signal transfer success when received success status.
    if (status == kStatus_Success)
    {
        g_masterCompletionFlag = true;
    }

    if (status == kStatus_I3C_IBIWon)
    {
        g_ibiWonFlag = true;
    }

    g_completionStatus = status;
}

void i3c_master_ibi_callback(I3C_Type *base, i3c_master_handle_t *handle, i3c_ibi_type_t ibiType, i3c_ibi_state_t ibiState)
{
	D_P3H2x4x_Handle *P3H2x4xDriver_ptr = P3H2x4x_get_handle();

	switch (ibiType)
	{
		case kI3C_IbiNormal:
			if (ibiState == kI3C_IbiDataBuffNeed)
			{
				handle->ibiBuff = (P3H2x4xDriver_ptr->ibi_info.ibi_buff);  //g_master_ibiBuff
			}
			else
			{
				memcpy(P3H2x4xDriver_ptr->ibi_info.ibi_buff, (void *)handle->ibiBuff, handle->ibiPayloadSize);
			}
			break;

		default:
			assert(false);
			break;
	}

	payload_byte_one = (*(uint8_t *)handle->ibiBuff);
    payload_byte_two = (*(uint8_t *)(handle->ibiBuff + 1));

	P3H2x4xDriver_ptr->ibi_info.is_ibi = true;
}

void P3H2x4x_read_smbus_agent_rx_buf(D_P3H2x4x_Handle *P3H2x4xDriver, enum i3c_hub_rcv_buf rfbuf,
		enum i3c_hub_tp tp, bool is_of){

	uint8_t target_buffer_page, flag_clear, rx_data, temp, i;
	uint8_t slave_rx_buffer[I3C_HUB_SMBUS_TARGET_PAYLOAD_SIZE] = { 0 };
	uint8_t packet_len, slave_address;
    uint32_t ret;

	target_buffer_page = (((rfbuf) ? I3C_HUB_TARGET_BUFF_1_PAGE : I3C_HUB_TARGET_BUFF_0_PAGE)
							+  (4 * tp));

	ret = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, PAGE_REG_POINTER_addr,
			target_buffer_page, 0x00, false, P3H2x4xDriver->in_i3c_mode);
	if(ret){
		PRINTF("\r\n IBI error \r\n");
		goto ibi_err;
		return;
	}

	//read buff length
	ret = Register_I3C_Read(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_CONTROLLER_AGENT_BUFF,
			1, &packet_len, P3H2x4xDriver->in_i3c_mode);

	if (ret){
		PRINTF("\r\n read buff error \r\n");
		goto ibi_err;
	}

	if (packet_len)
		packet_len = packet_len - 1;

	if (packet_len > I3C_HUB_SMBUS_TARGET_PAYLOAD_SIZE) {
		PRINTF(" \r\n Received message too big for hub buffer \r\n");
		goto ibi_err;
	}

	//read slave address
	ret = Register_I3C_Read(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_TARGET_BUFF_ADDRESS,
			1, &slave_address, P3H2x4xDriver->in_i3c_mode);
	if (ret){
		PRINTF("\r\n read slave addr error \r\n");
		goto ibi_err;
	}

	//read data
	if (packet_len){
		ret = Register_I3C_Read(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_TARGET_BUFF_DATA,
				packet_len, slave_rx_buffer, P3H2x4xDriver->in_i3c_mode);
		if (ret) {
			PRINTF("\r\n IBI read data error \r\n");
			goto ibi_err;
		}
	}

	if (is_of)
		flag_clear = BUF_RECEIVED_FLAG_TF_MASK;
	else
		flag_clear = (((rfbuf == RCV_BUF_0) ? I3C_HUB_TARGET_BUF_0_RECEIVE :
					I3C_HUB_TARGET_BUF_1_RECEIVE));

ibi_err:
		Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, PAGE_REG_POINTER_addr,
				0x00, 0x00, 0, P3H2x4xDriver->in_i3c_mode);
		Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_TP0_SMBUS_AGNT_STS + tp,
				flag_clear, 0x00, 0, P3H2x4xDriver->in_i3c_mode);

		if(P3H2x4xDriver->usercallbackibi != NULL)
			P3H2x4xDriver->usercallbackibi(slave_rx_buffer, packet_len);
		else
			PRINTF("\r\n No user IBI callback registered  \r\n");
}

int P3H2x4x_Read_IBI_data(D_P3H2x4x_Handle *P3H2x4xDriver, uint8_t *payload_byte_one, uint8_t *payload_byte_two){

	uint8_t target_port_status;
	uint8_t i;
	int  ret;

	if (!((*payload_byte_one) & SMBUS_AGENT_EVENT_FLAG_STATUS))
		return -1 ;

	ret = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, TARGET_SMBUS_IBI_CONF_addr,
			I3C_HUB_IBI_DISABLED, 0x00, 0, P3H2x4xDriver->in_i3c_mode);

	if (ret){
		PRINTF("\r\n Failed to Disable IBI \r\n");
		return -1 ;
	}

	for(int i = 0; i< I3C_HUB_TP_MAX_COUNT; ++i){
		if((((*payload_byte_two) >> i) & 0x01)){
			ret = Register_I3C_Read(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_TP0_SMBUS_AGNT_STS + i ,
										1, &target_port_status, P3H2x4xDriver->in_i3c_mode);
			if (ret) {
				PRINTF(" \r\n target port read status failed %d\n \r\n", ret);
				goto err;
			}

		    switch(target_port_status & BUF_RECEIVED_FLAG_MASK){
					case I3C_HUB_TARGET_BUF_CA_TF:
						break;

					case I3C_HUB_TARGET_BUF_0_RECEIVE:
						P3H2x4x_read_smbus_agent_rx_buf(P3H2x4xDriver, RCV_BUF_0, i, false);
						break;

					case I3C_HUB_TARGET_BUF_1_RECEIVE:
						P3H2x4x_read_smbus_agent_rx_buf(P3H2x4xDriver, RCV_BUF_1, i, false);
						break;

					case I3C_HUB_TARGET_BUF_0_1_RECEIVE:
						P3H2x4x_read_smbus_agent_rx_buf(P3H2x4xDriver, RCV_BUF_0, i, false);
						P3H2x4x_read_smbus_agent_rx_buf(P3H2x4xDriver, RCV_BUF_1, i, false);
						break;

					case I3C_HUB_TARGET_BUF_OVRFL:
						P3H2x4x_read_smbus_agent_rx_buf(P3H2x4xDriver, RCV_BUF_0, i, false);
						P3H2x4x_read_smbus_agent_rx_buf(P3H2x4xDriver, RCV_BUF_1, i, true);
						PRINTF("\r\n Overflow, reading buffer zero and one \r\n");
						break;

					default:
						Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_TP0_SMBUS_AGNT_STS + i,
								BUF_RECEIVED_FLAG_TF_MASK, 0x00, 0, P3H2x4xDriver->in_i3c_mode);
							break;
		    }
		}
	}
err:
		Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, TARGET_SMBUS_IBI_CONF_addr,
				 0x01 , 0x00, 0, P3H2x4xDriver->in_i3c_mode);

		return 0;
}

int P3H2x4x_Errata_fix(D_P3H2x4x_Handle *P3H2x4xDriver){

	int32_t status;

	status = Register_I2C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, 0x10,
					0x69, 0x00, 0, false);

	status = Register_I2C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, 0x16,
			0xFF, 0x00, 0, false);

	status = Register_I2C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, 0x19,
			0xA0, 0x00, 0, false);

    if (status != kStatus_Success)
    {
        PRINTF("\r\nErrata fix failed\r\n");
        return -1;
    }
    return 0;
}

int32_t P3H2x4x_Initialize(D_P3H2x4x_Handle *pSensorHandle, ARM_DRIVER_I3C *pBus, uint8_t index, uint16_t slaveAddress, uint8_t in_i3c_mode)
{
	int32_t status;
	uint8_t reg = 0;


	if(pSensorHandle->isInitialized)
		return SENSOR_ERROR_NONE;

/*! Initialize the driver. */
	status = pdriver->Initialize(I3C_S_SIGNAL_EVENT);

	if (ARM_DRIVER_OK != status)
	{
		PRINTF("\r\n I3C Initialization Failed\r\n");
		return -1;
	}

	/*! Set the Power mode. */
	status = pdriver->PowerControl(ARM_POWER_FULL);
	if (ARM_DRIVER_OK != status)
	{
		PRINTF("\r\n Driver Power Mode setting Failed\r\n");
		return -1;
	}

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
	pSensorHandle->slaveAddress = slaveAddress;
	pSensorHandle->isInitialized = true;

	if(in_i3c_mode){
		pSensorHandle->in_i3c_mode = true;
	}
	else{
		pSensorHandle->in_i3c_mode = false;
	}
	return SENSOR_ERROR_NONE;
}

int P3H2x4x_read_transaction_status(D_P3H2x4x_Handle *P3H2x4xDriver, uint8_t target_port_status, uint8_t *status, uint8_t data_length){

	uint8_t status_read;
	unsigned long time_to_timeout = 0;
	int ret, retry_err = 0;

	SDK_DelayAtLeastUs(I3C_HUB_SMBUS_400kHz_TRANSFER_TIMEOUT(data_length),  CLOCK_GetFreq(kCLOCK_CoreSysClk ));

	ret = Register_I3C_Read(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress,target_port_status ,
					1, &status_read, P3H2x4xDriver->in_i3c_mode);
	if (ret)
		return ret;

	*status = (uint8_t) status_read;

	if ((*status & I3C_HUB_TP_BUFFER_STATUS_MASK) == I3C_HUB_XFER_SUCCESS)
		return 0;

	PRINTF("\r\n Status read timeout reached!\r\n");
	return 0;

}

int32_t P3H2x4x_Target_Smbus_Write(D_P3H2x4x_Handle *P3H2x4xDriver, uint8_t target_port, uint8_t Target_address, uint8_t BytesToWrite, uint8_t *dataToWrite){

	 uint32_t status;
	 uint8_t ret_status;

	/*! Validate for the correct handle */
	if (P3H2x4xDriver == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (P3H2x4xDriver->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	//check target port status
	uint8_t target_port_status = I3C_HUB_TP0_SMBUS_AGNT_STS + target_port;


	//Disable IBI
		status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress,  TARGET_SMBUS_IBI_CONF_addr,
					0X00, 0x00, 0, P3H2x4xDriver->in_i3c_mode);


	status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, target_port_status,
			I3C_HUB_TP_BUFFER_STATUS_MASK, 0x00, 0, P3H2x4xDriver->in_i3c_mode);


	//write data on page pointer
	uint8_t controller_buffer_page = I3C_HUB_CONTROLLER_BUFFER_PAGE + 4 * target_port;

	status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, PAGE_REG_POINTER_addr,
			controller_buffer_page, 0x00, 0, P3H2x4xDriver->in_i3c_mode);

	if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n P3H2840 I3C HUB write Failed on target port 0\r\n");
			return -1;
		}

	//update the buffer for write only operation
	uint8_t buff[5];
	buff[0] = 2*Target_address; //address of target device
	buff[1] = 4;      			//transaction_type
	buff[2] = BytesToWrite;     // write_length
	buff[3] = 0;      			// read_length

	status = Register_I3C_BlockWrite(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_CONTROLLER_AGENT_BUFF,
			buff, 4, P3H2x4xDriver->in_i3c_mode);

	if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n write Failed on target port 0\r\n");
			return -1;
		}

	status = Register_I3C_BlockWrite(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_CONTROLLER_AGENT_BUFF_DATA,
			dataToWrite, BytesToWrite, P3H2x4xDriver->in_i3c_mode);

	if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n Failed on target port 0\r\n");
			return -1;
		}

	//start the transaction
	status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_TP_SMBUS_AGNT_TRANS_START,
			(uint8_t)(1 << target_port), 0x00, 0, P3H2x4xDriver->in_i3c_mode);

	if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n Transaction Failed on target port 0\r\n");
			return -1;
		}

	//Check the transaction status
	status = P3H2x4x_read_transaction_status(P3H2x4xDriver, target_port_status, &ret_status, BytesToWrite);
	if ((SENSOR_ERROR_NONE != status) || (ret_status != I3C_HUB_XFER_SUCCESS))
	{
		PRINTF("\r\n WRITE TRANSACTION STATUS failed\r\n");
		return -1;
	}

	//Reset Page pointer
	status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, PAGE_REG_POINTER_addr,
							0X00, 0x00, 0, P3H2x4xDriver->in_i3c_mode);

	if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n Page Pointer Reset failed \r\n");
			return -1;
		}


	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	//Enable IBI
		status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress,  TARGET_SMBUS_IBI_CONF_addr,
				(uint8_t)(1 << target_port), 0x00, 0, P3H2x4xDriver->in_i3c_mode);

	return SENSOR_ERROR_NONE;

}

int P3H2x4x_Target_Smbus_Read(D_P3H2x4x_Handle *P3H2x4xDriver, uint8_t target_port, uint8_t Target_address, uint8_t BytesToRead, uint8_t RegisterToRead, uint8_t *recieve_buff){

	uint8_t status, ret_status;
	uint8_t controller_buffer_page = I3C_HUB_CONTROLLER_BUFFER_PAGE + 4 * target_port;
	uint8_t target_port_status = I3C_HUB_TP0_SMBUS_AGNT_STS + target_port;

	//Disable IBI
	status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress,  TARGET_SMBUS_IBI_CONF_addr,
				0X00, 0x00, 0, P3H2x4xDriver->in_i3c_mode);

	if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n Page Pointer Reset failed \r\n");
			return -1;
		}

	//Reset Page pointer
	status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, PAGE_REG_POINTER_addr,
							0X00, 0x00, 0, P3H2x4xDriver->in_i3c_mode);

	if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n Page Pointer Reset failed \r\n");
			return -1;
		}

	status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, target_port_status,
			I3C_HUB_TP_BUFFER_STATUS_MASK, 0x00, 0, P3H2x4xDriver->in_i3c_mode);


	//Assign value to page pointer
	status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, PAGE_REG_POINTER_addr,
			controller_buffer_page, 0x00, 0, P3H2x4xDriver->in_i3c_mode);

	if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n Page pointer assignment failed! \r\n");
			return -1;
		}

	//Fill and assign values to buffer

	uint8_t buff[4];
	buff[0] = 2*Target_address; //address of target device
	buff[1] = 5;      			//transaction_type
	buff[2] = 1;     			// write_length
	buff[3] = BytesToRead;      // read_length


	status = Register_I3C_BlockWrite(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_CONTROLLER_AGENT_BUFF,
			buff, 4, P3H2x4xDriver->in_i3c_mode);

	if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n Buffer assignment Failed \r\n");
			return -1;
		}

	//write the register, we want to read
	status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_CONTROLLER_AGENT_BUFF_DATA,
			RegisterToRead, 0x00, 0, P3H2x4xDriver->in_i3c_mode);

	if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n Register to read failed\r\n");
			return -1;
		}

	//start the transaction 0x01 (uint8_t)(1 << target_port)
	status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_TP_SMBUS_AGNT_TRANS_START,
			(uint8_t)(1 << target_port), 0x00, 0, P3H2x4xDriver->in_i3c_mode);

	if (SENSOR_ERROR_NONE != status)
	{
		PRINTF("\r\n Transaction start Failed\r\n");
		return -1;
	}

	//check transaction status
	status = P3H2x4x_read_transaction_status(P3H2x4xDriver, target_port_status, &ret_status, (BytesToRead + 1));
	if ((SENSOR_ERROR_NONE != status) || (ret_status != I3C_HUB_XFER_SUCCESS))
	{
		PRINTF("\r\n transaction status failed\r\n");
		return -1;
	}

	// read register
	status = Register_I3C_Read(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_CONTROLLER_AGENT_BUFF_DATA + 1,
			BytesToRead, recieve_buff, P3H2x4xDriver->in_i3c_mode);

	if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n Read Failed\r\n");
			return -1;
		}

	//start transaction status
	status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, target_port_status,
			I3C_HUB_TP_BUFFER_STATUS_MASK, 0x00, 0, P3H2x4xDriver->in_i3c_mode);

	if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n transaction status failed\r\n");
			return -1;
		}

	//reset page pointer
	status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, PAGE_REG_POINTER_addr,
				0X00, 0x00, 0, P3H2x4xDriver->in_i3c_mode);

	if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n reset page pointer failed\r\n");
			return -1;
		}

	if (ARM_DRIVER_OK != status)
		{
			return SENSOR_ERROR_WRITE;
		}

	//Enable IBI 0
		status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress,  TARGET_SMBUS_IBI_CONF_addr,
				(uint8_t)(1<< target_port), 0x00, 0, P3H2x4xDriver->in_i3c_mode);

	return 0;
}

int P3H2x4x_Target_I3C_read(D_P3H2x4x_Handle *P3H2x4xDriver, uint8_t Target_address, uint8_t BytesToRead, uint8_t RegisterToRead, uint8_t *recieve_buff){

	uint32_t status;

	status = Register_I3C_Read(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, Target_address, RegisterToRead,
			BytesToRead, recieve_buff, true);

	if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n I3C data read failed\r\n");
			return -1;
		}
	return 0;
}

int P3H2x4x_Target_I3C_write(D_P3H2x4x_Handle *P3H2x4xDriver, uint8_t Target_address, uint8_t RegisterToWrite, uint8_t dataToWrite, uint8_t shiftBy, uint8_t mask){

	uint32_t status;

	status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, Target_address, RegisterToWrite,
			(uint8_t)(dataToWrite << shiftBy), mask, 0, true);

	if (SENSOR_ERROR_NONE != status)
	{
		PRINTF("\r\n I3C data write failed\r\n");
		return -1;
	}

	return 0;
}

int P3H2x4x_Target_I3C_block_write(D_P3H2x4x_Handle *P3H2x4xDriver, uint8_t Target_address, uint8_t BytesToWrite, uint8_t RegisterToWrite, uint8_t *dataToWrite, uint8_t shiftBy, uint8_t mask){

	uint32_t status;

	status = Register_I3C_BlockWrite(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, Target_address, RegisterToWrite,
			dataToWrite, BytesToWrite, true);

	if (SENSOR_ERROR_NONE != status)
	{
		PRINTF("\r\n I3C data write failed\r\n");
		return -1;
	}

	return 0;
}


int P3H2x4x_Target_I2C_read(D_P3H2x4x_Handle *P3H2x4xDriver, uint8_t Target_address, uint8_t BytesToRead, uint8_t RegisterToRead, uint8_t *recieve_buff){

	uint32_t status;

	status = Register_I3C_Read(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, Target_address, RegisterToRead,
			BytesToRead, recieve_buff, false);

	if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n I2C data read failed\r\n");
			return -1;
		}
	return 0;

}

int P3H2x4x_Target_I2C_write(D_P3H2x4x_Handle *P3H2x4xDriver, uint8_t Target_address, uint8_t RegisterToWrite, uint8_t dataToWrite, uint8_t shiftBy, uint8_t mask){

	uint32_t status;


	status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, Target_address, RegisterToWrite,
			(uint8_t)(dataToWrite << shiftBy), mask, 0, false);

	if (SENSOR_ERROR_NONE != status)
	{
		PRINTF("\r\n I2C data write failed\r\n");
		return -1;
	}

	return 0;
}


int P3H2x4x_Target_I2C_block_write(D_P3H2x4x_Handle *P3H2x4xDriver, uint8_t Target_address, uint8_t BytesToWrite, uint8_t RegisterToWrite, uint8_t *dataToWrite, uint8_t shiftBy, uint8_t mask){

	uint32_t status;


	status = Register_I3C_BlockWrite(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, Target_address, RegisterToWrite,
			dataToWrite, BytesToWrite, false);

	if (SENSOR_ERROR_NONE != status)
	{
		PRINTF("\r\n I2C data write failed\r\n");
		return -1;
	}

	return 0;
}

int P3H2x4x_Enable_Disable_IBI(D_P3H2x4x_Handle *P3H2x4xDriver){

	uint32_t status;

	if(enable_ibi){

		status = pdriver->DirectENEC(P3H2x4xDriver->slaveAddress, IBI_CMD);
		if (ARM_DRIVER_OK != status)
		{
			return -1;
		}
	}
	else if(disable_ibi){
		status = pdriver->DirectDISEC(P3H2x4xDriver->slaveAddress, IBI_CMD);
		if (ARM_DRIVER_OK != status)
		{
			return -1;
		}
	}
	return 0;
}

int P3H2x4x_Dynamic_addr_assgmt_without_rest(uint8_t static_addr, uint8_t dynamic_addr){

	uint32_t status;

	status = pdriver->LPI3C_dynamic_address_assignment_without_reset(static_addr, dynamic_addr);
	if (ARM_DRIVER_OK != status)
	{
		return -1;
	}
	return 0;
}

int P3H2x4x_Dynamic_addr_assgmt_with_rest(uint8_t static_addr, uint8_t dynamic_addr){

	uint32_t status;

	status = pdriver->LPI3C_dynamic_address_assignment(static_addr, dynamic_addr);
	if (ARM_DRIVER_OK != status)
	{
		return -1;
	}
	return 0;
}
void P3H2x4x_default_Configuration(I3C_Hub_Configuration *i3c_hub_config)
{
	uint8_t tp_count;

	i3c_hub_config->cp0_ldo_en = I3C_HUB_LDO_NOT_DEFINED;
	i3c_hub_config->cp1_ldo_en = I3C_HUB_LDO_NOT_DEFINED;
	i3c_hub_config->tp0145_ldo_en = I3C_HUB_LDO_NOT_DEFINED;
	i3c_hub_config->tp2367_ldo_en = I3C_HUB_LDO_NOT_DEFINED;
	i3c_hub_config->tp0145_pullup = I3C_HUB_TP_PULLUP_NOT_SET;
	i3c_hub_config->tp2367_pullup = I3C_HUB_TP_PULLUP_NOT_SET;
	i3c_hub_config->cp0_ldo_volt = I3C_HUB_LDO_VOLT_NOT_SET;
	i3c_hub_config->cp1_ldo_volt = I3C_HUB_LDO_VOLT_NOT_SET;
	i3c_hub_config->tp0145_ldo_volt = I3C_HUB_LDO_VOLT_NOT_SET;
	i3c_hub_config->tp2367_ldo_volt = I3C_HUB_LDO_VOLT_NOT_SET;
	i3c_hub_config->cp0_io_strength = I3C_HUB_IO_STRENGTH_NOT_SET;
	i3c_hub_config->cp1_io_strength = I3C_HUB_IO_STRENGTH_NOT_SET;
	i3c_hub_config->tp0145_io_strength = I3C_HUB_IO_STRENGTH_NOT_SET;
	i3c_hub_config->tp2367_io_strength = I3C_HUB_IO_STRENGTH_NOT_SET;

	for (tp_count = 0; tp_count < I3C_HUB_TP_MAX_COUNT; ++tp_count) {

		i3c_hub_config->target_port[tp_count].mode =  I3C_HUB_TP_MODE_NOT_SET;
		i3c_hub_config->target_port[tp_count].pullup_en = I3C_HUB_LDO_NOT_DEFINED;
		i3c_hub_config->target_port[tp_count].ibi_config = I3C_HUB_IBI_DISABLED;
	}
}

void P3H2x4x_set_Configuration(I3C_Hub_Configuration *i3c_hub_config)
{
	uint8_t tp_count;

	i3c_hub_config->cp0_ldo_en = I3C_HUB_CP0_LDO_CONFIG;
	i3c_hub_config->cp1_ldo_en = I3C_HUB_CP1_LDO_CONFIG;
	i3c_hub_config->tp0145_ldo_en = I3C_HUB_TP0145_LDO_CONFIG;
	i3c_hub_config->tp2367_ldo_en = I3C_HUB_TP2367_LDO_CONFIG;
	i3c_hub_config->cp0_ldo_volt = I3C_HUB_CP0_LDO_VOLT;
	i3c_hub_config->cp1_ldo_volt = I3C_HUB_CP1_LDO_VOLT;
	i3c_hub_config->tp0145_ldo_volt = I3C_HUB_TP0145_LDO_VOLT;
	i3c_hub_config->tp2367_ldo_volt = I3C_HUB_TP2367_LDO_VOLT;
	i3c_hub_config->tp0145_pullup = I3C_HUB_TP0145_PULLUP;
	i3c_hub_config->tp2367_pullup = I3C_HUB_TP2367_PULLUP;
	i3c_hub_config->cp0_io_strength = I3C_HUB_CP0_IO_STRENGTH;
	i3c_hub_config->cp1_io_strength = I3C_HUB_CP1_IO_STRENGTH;
	i3c_hub_config->tp0145_io_strength = I3C_HUB_TP0145_IO_STRENGTH;
	i3c_hub_config->tp2367_io_strength = I3C_HUB_TP2367_IO_STRENGTH;

	for (tp_count = 0; tp_count < I3C_HUB_TP_MAX_COUNT; ++tp_count) {
		i3c_hub_config->target_port[tp_count].mode =  I3C_HUB_TP_MODE(tp_count);
		i3c_hub_config->target_port[tp_count].pullup_en = I3C_HUB_TP_PULLUP(tp_count);
		i3c_hub_config->target_port[tp_count].ibi_config = I3C_HUB_TP_IBI_CONFIG(tp_count);
	}
}

static int P3H2x4x_Configure_pullup(D_P3H2x4x_Handle *P3H2x4xDriver, I3C_Hub_Configuration *i3c_hub_config)
{
	uint8_t mask, value = 0;

	if (i3c_hub_config->tp0145_pullup != I3C_HUB_TP_PULLUP_NOT_SET) {
		mask |= TP0145_PULLUP_CONF_MASK;
		value |= TP0145_PULLUP_CONF(i3c_hub_config->tp0145_pullup);
	}

	if (i3c_hub_config->tp2367_pullup != I3C_HUB_TP_PULLUP_NOT_SET) {
		mask |= TP2367_PULLUP_CONF_MASK;
		value |= TP2367_PULLUP_CONF(i3c_hub_config->tp2367_pullup);
	}

	return Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_LDO_AND_PULLUP_CONF,
					value, mask, 0, P3H2x4xDriver->in_i3c_mode);
}

static int P3H2x4x_Configure_reset(D_P3H2x4x_Handle *P3H2x4xDriver, I3C_Hub_Configuration *i3c_hub_config)
{
	return Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, CONTROLLER_PORT_CONF_addr,
					(1 << 2 ), 0x04, 0, P3H2x4xDriver->in_i3c_mode);
}

static int P3H2x4x_Configure_io_strength(D_P3H2x4x_Handle *P3H2x4xDriver, I3C_Hub_Configuration *i3c_hub_config)
{
	uint8_t mask_all = 0, val_all = 0;
	uint8_t reg_val, val;
	int ret;

	/* Get IO strength configuration to figure out what needs to be changed */
	ret = Register_I3C_Read(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_IO_STRENGTH ,
			1, &reg_val, P3H2x4xDriver->in_i3c_mode);
	if (ret)
		return ret;

	if (i3c_hub_config->cp0_io_strength != I3C_HUB_IO_STRENGTH_NOT_SET) {
		val = CP0_IO_STRENGTH(i3c_hub_config->cp0_io_strength);
		mask_all |= CP0_IO_STRENGTH_MASK;
		val_all |= val;
	}
	if (i3c_hub_config->cp1_io_strength != I3C_HUB_IO_STRENGTH_NOT_SET) {
		val = CP1_IO_STRENGTH(i3c_hub_config->cp1_io_strength);
		mask_all |= CP1_IO_STRENGTH_MASK;
		val_all |= val;
	}
	if (i3c_hub_config->tp0145_io_strength != I3C_HUB_IO_STRENGTH_NOT_SET) {
		val = TP0145_IO_STRENGTH(i3c_hub_config->tp0145_io_strength);
		mask_all |= TP0145_IO_STRENGTH_MASK;
		val_all |= val;
	}
	if (i3c_hub_config->tp2367_io_strength != I3C_HUB_IO_STRENGTH_NOT_SET) {
		val = TP2367_IO_STRENGTH(i3c_hub_config->tp2367_io_strength);
		mask_all |= TP2367_IO_STRENGTH_MASK;
		val_all |= val;
	}

	/* Set IO strength if required */
	return Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_IO_STRENGTH,
					val_all, mask_all, 0, P3H2x4xDriver->in_i3c_mode);
}

static int P3H2x4x_Configure_ldo(D_P3H2x4x_Handle *P3H2x4xDriver, I3C_Hub_Configuration *i3c_hub_config)
{
	uint8_t ldo_config_mask =0, ldo_config_val = 0;
	uint8_t ldo_disable_mask = 0, ldo_en_val = 0;
	uint8_t reg_val, val;
	int ret;

	/* Enable or Disable LDO's. If there is no DT entry - disable LDO for safety reasons */
	if (i3c_hub_config->cp0_ldo_en == I3C_HUB_LDO_ENABLED)
		ldo_en_val |= CP0_EN_LDO;
	if (i3c_hub_config->cp1_ldo_en == I3C_HUB_LDO_ENABLED)
		ldo_en_val |= CP1_EN_LDO;
	if (i3c_hub_config->tp0145_ldo_en == I3C_HUB_LDO_ENABLED)
		ldo_en_val |= TP0145_EN_LDO;
	if (i3c_hub_config->tp2367_ldo_en == I3C_HUB_LDO_ENABLED)
		ldo_en_val |= TP2367_EN_LDO;

	/* Get current LDOs configuration */
	ret = Register_I3C_Read(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress,I3C_HUB_VCCIO_LDO_CONF ,
			1, &reg_val, P3H2x4xDriver->in_i3c_mode);
	if (ret)
		return ret;

	/* LDOs Voltage level (Skip if not defined in the DT)
	 * Set the mask only if there is a change from current value
	 */
	if (i3c_hub_config->cp0_ldo_volt != I3C_HUB_LDO_VOLT_NOT_SET) {
		val = CP0_VCCIO_LDO_VOLTAGE(i3c_hub_config->cp0_ldo_volt);
		if ((reg_val & CP0_VCCIO_LDO_VOLTAGE_MASK) != val) {
			ldo_config_mask |= CP0_VCCIO_LDO_VOLTAGE_MASK;
			ldo_config_val |= val;

			ldo_disable_mask |= CP0_EN_LDO;
		}
	}

		if (i3c_hub_config->cp1_ldo_volt != I3C_HUB_LDO_VOLT_NOT_SET) {
			val = CP1_VCCIO_LDO_VOLTAGE(i3c_hub_config->cp1_ldo_volt);
			if ((reg_val & CP1_VCCIO_LDO_VOLTAGE_MASK) != val) {
				ldo_config_mask |= CP1_VCCIO_LDO_VOLTAGE_MASK;
				ldo_config_val |= val;

				ldo_disable_mask |= CP1_EN_LDO;
			}
		}

		if (i3c_hub_config->tp0145_ldo_volt != I3C_HUB_LDO_VOLT_NOT_SET) {
			val = TP0145_VCCIO_LDO_VOLTAGE(i3c_hub_config->tp0145_ldo_volt);
			if ((reg_val & TP0145_VCCIO_LDO_VOLTAGE_MASK) != val) {
				ldo_config_mask |= TP0145_VCCIO_LDO_VOLTAGE_MASK;
				ldo_config_val |= val;

				ldo_disable_mask |= TP0145_EN_LDO;
			}
		}
		if (i3c_hub_config->tp2367_ldo_volt != I3C_HUB_LDO_VOLT_NOT_SET) {
			val = TP2367_VCCIO_LDO_VOLTAGE(i3c_hub_config->tp2367_ldo_volt);
			if ((reg_val & TP2367_VCCIO_LDO_VOLTAGE_MASK) != val) {
				ldo_config_mask |= TP2367_VCCIO_LDO_VOLTAGE_MASK;
				ldo_config_val |= val;

				ldo_disable_mask |= TP2367_EN_LDO;
			}
		}
		/*
		 *Update LDO voltage configuration only if value is changed from already existing register
		 * value. It is a good practice to disable the LDO's before making any voltage changes.
		 * Presence of config mask indicates voltage change to be applied.
		 */
		if (ldo_config_mask) {
			/* Disable LDO's before making voltage changes */
			 ret = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_LDO_AND_PULLUP_CONF,
						0x00, ldo_disable_mask, 0, P3H2x4xDriver->in_i3c_mode);
			if (ret)
				return ret;

			/* Update the LDOs configuration */
			ret = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_VCCIO_LDO_CONF,
					ldo_config_val, ldo_config_mask, 0, P3H2x4xDriver->in_i3c_mode);
			if (ret)
				return ret;
		}

		/* Update the LDOs Enable/disable register. This will enable only LDOs enabled in DT */

		 ret = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_LDO_AND_PULLUP_CONF,
				ldo_en_val, LDO_ENABLE_DISABLE_MASK, 0, P3H2x4xDriver->in_i3c_mode);

		 if (ret)
		 	return ret;

	return 0;
}

static int P3H2x4x_Configure_tp(D_P3H2x4x_Handle *P3H2x4xDriver, I3C_Hub_Configuration *i3c_hub_config)
{
	uint8_t pullup_mask, pullup_val = 0;
	uint8_t smbus_mask, smbus_val = 0;
	uint8_t gpio_mask, gpio_val = 0;
	uint8_t i3c_mask, i3c_val = 0;
	uint8_t ibi_mask, ibi_val = 0;
	uint8_t i2c_mask = 0, i2c_val = 0;
	int ret;
	int i;

	/* TBD: Read type of HUB from register I3C_HUB_DEV_INFO_0 to learn target ports count. */
	for (i = 0; i < I3C_HUB_TP_MAX_COUNT; ++i) {
		if (i3c_hub_config->target_port[i].mode != I3C_HUB_TP_MODE_NOT_SET) {
			i3c_mask |= TPn_NET_CON(i);
			smbus_mask |= TPn_SMBUS_MODE_EN(i);
			gpio_mask |= TPn_GPIO_MODE_EN(i);
			i2c_mask |= TPn_I2C_MODE_EN(i);

			if (i3c_hub_config->target_port[i].mode == I3C_HUB_TP_MODE_I3C)
				i3c_val |= TPn_NET_CON(i);
			else if (i3c_hub_config->target_port[i].mode == I3C_HUB_TP_MODE_SMBUS)
				smbus_val |= TPn_SMBUS_MODE_EN(i);
			else if (i3c_hub_config->target_port[i].mode == I3C_HUB_TP_MODE_GPIO)
				gpio_val |= TPn_GPIO_MODE_EN(i);
			else if (i3c_hub_config->target_port[i].mode == I3C_HUB_TP_MODE_I2C)
				i2c_val |= TPn_I2C_MODE_EN(i);

		}
		if (i3c_hub_config->target_port[i].pullup_en != I3C_HUB_TP_PULLUP_DISABLED) {
			pullup_mask |= TPn_PULLUP_EN(i);

			if (i3c_hub_config->target_port[i].pullup_en == I3C_HUB_TP_PULLUP_ENABLED  )
				pullup_val |= TPn_PULLUP_EN(i);
		}

		if (i3c_hub_config->target_port[i].ibi_config != I3C_HUB_IBI_DISABLED) {
			ibi_mask |= TPn_IBI_EN(i);

			if (i3c_hub_config->target_port[i].ibi_config == I3C_HUB_IBI_ENABLED)
				ibi_val |= TPn_IBI_EN(i);
		}
	}

	ret = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_TP_IO_MODE_CONF,
			(smbus_val | i2c_val), (smbus_mask | i2c_mask) , 0, P3H2x4xDriver->in_i3c_mode);
	if (ret)
		return ret;

	ret = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, TARGET_PULL_UP_EN_addr,
			pullup_val, pullup_mask, 0, P3H2x4xDriver->in_i3c_mode);
	if (ret)
		return ret;


	ret = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_TP_SMBUS_AGNT_IBI_CONFIG,
			ibi_val, ibi_mask, 0, P3H2x4xDriver->in_i3c_mode);
	if (ret)
		return ret;

	ret = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_TP_SMBUS_AGNT_EN,
			smbus_val, smbus_mask, 0, P3H2x4xDriver->in_i3c_mode);
	if (ret)
		return ret;


	ret = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_TP_GPIO_MODE_EN,
			gpio_val, gpio_mask, 0, P3H2x4xDriver->in_i3c_mode);
	if (ret)
		return ret;

	/* Request for HUB Network connection in case any TP is configured in I3C mode */
	if (i3c_val) {
		ret = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, CONTROLLER_MUX_REQ_addr,
				CONTROLLER_PORT_MUX_REQ, 0x00, 0, P3H2x4xDriver->in_i3c_mode);
		if (ret)
			return ret;
	}

	/* Enable TP here in case TP was configured */
	ret = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, TARGET_PORT_EN_addr,
			(i3c_val | smbus_val | gpio_val | i2c_val), (i3c_mask | smbus_mask | gpio_mask | i2c_mask), 0, P3H2x4xDriver->in_i3c_mode);
	if (ret)
		return ret;

	ret = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_ONCHIP_TD_AND_SMBUS_AGNT_CONF,
			0x20, TARGET_AGENT_DFT_IBI_CONF_MASK, 0, P3H2x4xDriver->in_i3c_mode);    // 0x20
	if (ret)
		return ret;

	ret = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, TARGET_HUB_NET_CONN_EN_addr,
			(i3c_val | i2c_val), (i3c_mask | i2c_mask), 0, P3H2x4xDriver->in_i3c_mode);

	return ret;
}

int P3H2x4x_Set_Cp_sel_pin(void){

#if defined(P3H2441) || defined(P3H2841)
	pGpioDriver->set_pin(&RST_PIN);

#elif defined(P3H2840) || defined(P3H2440)
	pGpioDriver->clr_pin(&RST_PIN);
#endif

	return 0;
}

int P3H2x41_config(D_P3H2x4x_Handle *P3H2x4xDriver){

	 int32_t status;

	 status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, I3C_HUB_CONTROLLER_BUFFER_PAGE,
			0x69, 0x00, 0, P3H2x4xDriver->in_i3c_mode);

	 status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, CONTROLLER_PORT_CONF_addr,
			0x10, 0x00, 0, P3H2x4xDriver->in_i3c_mode);

	 status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, TARGET_PORT_EN_addr,
			0x00, 0x00, 0, P3H2x4xDriver->in_i3c_mode);

	 status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, IO_DS_CONF_addr,
			0x05, 0x00, 0, P3H2x4xDriver->in_i3c_mode);

	 status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, HUB_NET_OP_MODE_CONF_addr,
			0x00, 0x00, 0, P3H2x4xDriver->in_i3c_mode);

	 status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, VCCIO_VOLTAGE_addr,
			0x00, 0x00, 0, P3H2x4xDriver->in_i3c_mode);

	 status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, LDO_EN_PULL_UP_RES_VALUE_addr,
			0x50, 0x00, 0, P3H2x4xDriver->in_i3c_mode);

	 status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, TARGET_HUB_NET_CONN_EN_addr,
			0x00, 0x00, 0, P3H2x4xDriver->in_i3c_mode);

	 return 0;
}

int P3H2x4x_Configure(D_P3H2x4x_Handle *P3H2x4xDriver, I3C_Hub_Configuration *i3c_hub_config)
{
	int ret;

	ret = P3H2x4x_Configure_ldo(P3H2x4xDriver, i3c_hub_config);
	if (ret)
		return ret;

	ret = P3H2x4x_Configure_io_strength(P3H2x4xDriver, i3c_hub_config);
	if (ret)
		return ret;

	ret = P3H2x4x_Configure_pullup(P3H2x4xDriver, i3c_hub_config);
	if (ret)
		return ret;

	ret = P3H2x4x_Configure_reset(P3H2x4xDriver, i3c_hub_config);
		if (ret)
			return ret;

	ret = P3H2x4x_Configure_tp(P3H2x4xDriver, i3c_hub_config);
	if (ret)
		return ret;

	ret = P3H2x4x_Set_Cp_sel_pin();
	if (ret)
		return ret;
}

int P3H2x4x_UnlockPrtcdReg(D_P3H2x4x_Handle *P3H2x4xDriver){

	uint32_t status;

	status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, 0x10,
											0x69, 0x00, 0, P3H2x4xDriver->in_i3c_mode);

	if (SENSOR_ERROR_NONE != status)
	{
		PRINTF("\r\n  Can not unlock protected regsiters  \r\n");
		return -1;
	}

	return 0;
}

int P3H2x4x_Interface_Reset(D_P3H2x4x_Handle *P3H2x4xDriver){

	uint32_t status;

	if(P3H2x4xDriver->in_i3c_mode){
		status = Register_I3C_Write(P3H2x4xDriver->pCommDrv, &P3H2x4xDriver->deviceInfo, P3H2x4xDriver->slaveAddress, 0x7C,
										0x96, 0x00, 0, true);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n Hub reset failed\r\n");
			return -1;
		}
	}
	P3H2x4xDriver->isInitialized = false;
	return 0;
}

void P3H2x4x_Target_device_reset(void)
{
    LPI3C_DeinitPins();

	pGpioDriver->pin_init(&SCL_PIN, GPIO_DIRECTION_OUT, NULL, NULL, NULL);
	pGpioDriver->pin_init(&SDA_PIN, GPIO_DIRECTION_OUT, NULL, NULL, NULL);

	//scl low
	pGpioDriver->clr_pin(&SCL_PIN);

	//toggling of sda line 14 times
	int dec = 14;
	while(--dec){

		pGpioDriver->toggle_pin(&SDA_PIN);
	}

	//repeated start condition
	pGpioDriver->set_pin(&SCL_PIN);
	pGpioDriver->set_pin(&SDA_PIN);
	pGpioDriver->clr_pin(&SDA_PIN);

	//Stop condition
	pGpioDriver->set_pin(&SDA_PIN);

	LPI3C_InitPins();
}

void P3H2x4x_Reset_Device(D_P3H2x4x_Handle *P3H2x4xDriver){

	P3H2x4xDriver->isInitialized = false;

	pGpioDriver->toggle_pin(&RST_PIN);
	BOARD_DELAY_ms(1);
	pGpioDriver->toggle_pin(&RST_PIN);
	BOARD_DELAY_ms(1);
	pGpioDriver->toggle_pin(&RST_PIN);
	BOARD_DELAY_ms(1);
	pGpioDriver->toggle_pin(&RST_PIN);
}

void P3H2x4x_Cp_sel_pin_init(void)
{
	 gpioConfigKSDK_t gpioConfigDefault = {
			 .pinConfig = {kGPIO_DigitalOutput, 1}
	 };

	pGpioDriver->pin_init(&RST_PIN, GPIO_DIRECTION_OUT, &gpioConfigDefault, NULL, NULL);
	pGpioDriver->clr_pin(&RST_PIN);
}
