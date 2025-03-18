/*
* Copyright 2025 NXP
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef I3C_HUB_I3C_HUB_DRV_H_
#define I3C_HUB_I3C_HUB_DRV_H_

#include "register_io_i3c.h"

#if ((P3H2840) || (P3H2841))
	#define I3C_HUB_TP_MAX_COUNT				0x08
#elif ((P3H2440) || (P3H2441))
	#define I3C_HUB_TP_MAX_COUNT				0x04
#endif

#define I3C_HUB_CP_MAX_COUNT				0x02


/* LDO Disable/Enable DT settings */
#define I3C_HUB_LDO_VOLT_1_0V				0x00
#define I3C_HUB_LDO_VOLT_1_1V				0x01
#define I3C_HUB_LDO_VOLT_1_2V				0x02
#define I3C_HUB_LDO_VOLT_1_8V				0x03
#define I3C_HUB_LDO_VOLT_NOT_SET			0x04

#define I3C_HUB_LDO_DISABLED				0x00
#define I3C_HUB_LDO_ENABLED					0x01
#define I3C_HUB_LDO_NOT_DEFINED				0x02

/* Pullup selection DT settings for HUB */
#define I3C_HUB_TP_PULLUP_250R				0x00
#define I3C_HUB_TP_PULLUP_500R				0x01
#define I3C_HUB_TP_PULLUP_1000R				0x02
#define I3C_HUB_TP_PULLUP_2000R				0x03
#define I3C_HUB_TP_PULLUP_NOT_SET			0x04

#define I3C_HUB_TP_PULLUP_DISABLED			0x01
#define I3C_HUB_TP_PULLUP_ENABLED			0x02
#define I3C_HUB_TP_PULLUP_NOT_DEFINED		0x03

#define I3C_HUB_IO_STRENGTH_20_OHM			0x00
#define I3C_HUB_IO_STRENGTH_30_OHM			0x01
#define I3C_HUB_IO_STRENGTH_40_OHM			0x02
#define I3C_HUB_IO_STRENGTH_50_OHM			0x03
#define I3C_HUB_IO_STRENGTH_NOT_SET			0x04

#define I3C_HUB_IBI_DISABLED				0x00
#define I3C_HUB_IBI_ENABLED					0x01
#define I3C_HUB_IBI_NOT_DEFINED				0x02

#define I3C_HUB_TP_MODE_I3C					0x00
#define I3C_HUB_TP_MODE_SMBUS				0x01
#define I3C_HUB_TP_MODE_GPIO				0x02
#define I3C_HUB_TP_MODE_I2C					0x03
#define I3C_HUB_TP_MODE_NOT_SET				0x04

/* Hub buffer size */
#define I3C_HUB_CONTROLLER_BUFFER_SIZE			88
#define I3C_HUB_TARGET_BUFFER_SIZE				80
#define I3C_HUB_SMBUS_DESCRIPTOR_SIZE			4
#define I3C_HUB_SMBUS_PAYLOAD_SIZE			(I3C_HUB_CONTROLLER_BUFFER_SIZE - I3C_HUB_SMBUS_DESCRIPTOR_SIZE)
#define I3C_HUB_SMBUS_TARGET_PAYLOAD_SIZE	(I3C_HUB_TARGET_BUFFER_SIZE - 2)

/* buff receive flag set */
#define I3C_HUB_TARGET_BUF_CA_TF			0x01
#define I3C_HUB_TARGET_BUF_0_RECEIVE		0x02
#define I3C_HUB_TARGET_BUF_1_RECEIVE		0x04
#define I3C_HUB_TARGET_BUF_0_1_RECEIVE		0x06
#define I3C_HUB_TARGET_BUF_OVRFL			0x0E
#define BUF_RECEIVED_FLAG_MASK				0x0E
#define BUF_RECEIVED_FLAG_TF_MASK			0x0F
#define I3C_HUB_TARGET_BUFF_0_PAGE			0x12
#define I3C_HUB_TARGET_BUFF_1_PAGE			0x13

/*Number of target ports*/
#define I3C_HUB_TP_0						 0x00
#define I3C_HUB_TP_1						 0x01
#define I3C_HUB_TP_2						 0x02
#define I3C_HUB_TP_3						 0x03
#define I3C_HUB_TP_4						 0x04
#define I3C_HUB_TP_5						 0x05
#define I3C_HUB_TP_6						 0x06
#define I3C_HUB_TP_7						 0x07

/*IBI Settings*/
#define I3C_HUB_TP_SMBUS_AGNT_IBI_CONFIG	0x1B
#define TARGET_AGENT_DFT_IBI_CONF			0x20
#define	TARGET_AGENT_DFT_IBI_CONF_MASK		0x21
#define IBI_CMD								0X01

/* Device Status and IBI Registers */
#define I3C_HUB_DEV_AND_IBI_STS				0x20
#define I3C_HUB_TP_SMBUS_AGNT_IBI_STS		0x21
#define SMBUS_AGENT_EVENT_FLAG_STATUS		0x10

#define ONE_BYTE_SIZE						1
#define I3C_HUB_SMBUS_400kHz_TRANSFER_TIMEOUT(x)			(((20 * x) + 80))

/* Hub SMBus timeout time period in nanoseconds */
#define I3C_HUB_SMBUS_400kHz_TIMEOUT	1000

enum i3c_hub_tp {
	TP_0,
	TP_1,
	TP_2,
	TP_3,
	TP_4,
	TP_5,
	TP_6,
	TP_7,
};

enum i3c_hub_rcv_buf {
	RCV_BUF_0,
	RCV_BUF_1,
	RCV_BUF_OF,
};

typedef enum EN_DIS_IBI{
	enable_ibi = 0x01,
	disable_ibi = 0x02,
}en_dis_ibi;

typedef struct IBI_Info
{
	bool is_ibi; 				 /*!< Whether IBI is triggered or not.*/
	uint8_t ibi_buff[2];			 /*!< IBI payload buffer*/
}  ibi_info;
typedef struct
{
	registerDeviceInfo_t deviceInfo; /*!< I3C device context. */
	ARM_DRIVER_I3C *pCommDrv;        /*!< Pointer to the i3c driver. */
	bool isInitialized;              /*!< Whether sensor is intialized or not.*/
	uint16_t slaveAddress;           /*!< slave address.*/
	bool in_i3c_mode; 				 /*!< Whether sensor is in I3C mode or not.*/
	void (*usercallbackibi)(uint8_t *buf, uint8_t len); /*!< Transfer complete callback */
	ibi_info ibi_info;
}  D_P3H2x4x_Handle;

typedef struct
{
	uint8_t  static_target_address;
	uint8_t  target_port;
	uint8_t  bytesToWrite;
	uint8_t  regsiterToWrite;
	uint8_t  dataToWrite[100];
	uint8_t  shiftBy;
	uint8_t  mask;
} p3h2x4x_manual_config;

typedef struct TP_CONFIGURATION {
	uint8_t mode;
	uint8_t pullup_en;
	uint8_t ibi_config;
}Tp_Configuration;

typedef struct I3C_HUB_CONFIGURATION {

	uint8_t cp0_ldo_en;
	uint8_t cp1_ldo_en;
	uint8_t tp0145_ldo_en;
	uint8_t tp2367_ldo_en;
	uint8_t cp0_ldo_volt;
	uint8_t cp1_ldo_volt;
	uint8_t tp0145_ldo_volt;
	uint8_t tp2367_ldo_volt;
	uint8_t tp0145_pullup;
	uint8_t tp2367_pullup;
	uint8_t cp0_io_strength;
	uint8_t cp1_io_strength;
	uint8_t tp0145_io_strength;
	uint8_t tp2367_io_strength;
	Tp_Configuration target_port[I3C_HUB_TP_MAX_COUNT];
}I3C_Hub_Configuration;

/*! @brief       The interface function to set the hub handle for IBI implementation .
 *  @return      ::P3H2x4x_set_handle() returns the status .
 */
void P3H2x4x_set_handle(D_P3H2x4x_Handle *pSensorHandle);

/*! @brief       The interface function to get the hub handle for IBI implementation
 *  @return      ::P3H2x4x_get_handle() returns the status .
 */
D_P3H2x4x_Handle * P3H2x4x_get_handle(void);
/*! @brief       The interface function to read ibi buffer
 *  @details     This function initialize the sensor and sensor handle.
 *  @return      :void type .
 */
void P3H2x4x_read_smbus_agent_rx_buf(D_P3H2x4x_Handle *P3H2x4xDriver, enum i3c_hub_rcv_buf rfbuf, enum i3c_hub_tp tp, bool is_of);

/*! @brief       The interface function to read the IBI .
 *  @details     This function reads the payload data of IBI
 *  @return      ::P3H2x4x_Read_IBI_data() returns the status .
 */
int P3H2x4x_Read_IBI_data(D_P3H2x4x_Handle *P3H2x4xDriver, uint8_t *payload_byte_one, uint8_t *payload_byte_two);

/*! @brief       The interface function to initiate the I3C process.
 *  @details     This function writes the LDO, Unlock write protected registers and VCCIO required for I3C to start.
 *  @return      ::P3H2x4x_Errata_fix() returns the status .
 */
int P3H2x4x_Errata_fix(D_P3H2x4x_Handle *P3H2x4xDriver);

/*! @brief       The interface function to initialize the I3C HUB.
 *  @details     This function initialize the I3C HUB and I3C HUB handlen.
 *  @return      ::P3H2x4x_Initialize() returns the status .
 */
int32_t P3H2x4x_Initialize(D_P3H2x4x_Handle *pSensorHandle, ARM_DRIVER_I3C *pBus, uint8_t index, uint16_t slaveAddress, uint8_t in_i3c_mode);

/*! @brief       The interface function to configure the sensor handle.
 *  @details     This function configures the sensor handle.
 *  @return      ::P3H2x4x_Configure() returns the status .
 */
int P3H2x4x_Configure(D_P3H2x4x_Handle *P3H2x4xDriver, I3C_Hub_Configuration *i3c_hub_config);

/*! @brief       The interface function to read the transaction status.
 *  @details     This function reads the status of any smbus transaction.
 *  @return      ::P3H2x4x_read_transaction_status() returns the status .
 */
int P3H2x4x_read_transaction_status(D_P3H2x4x_Handle *P3H2x4xDriver, uint8_t target_port_status, uint8_t *status, uint8_t data_length);

/*! @brief       The interface function to initialize the default configuration of HUB.
 *  @details     This function configures the default set up configurations.
 *  @return      ::P3H2x4x_default_Configuration() void type .
 */
void P3H2x4x_default_Configuration(I3C_Hub_Configuration *i3c_hub_config);

/*! @brief       The interface function to set the configurations.
 *  @details     This function set the configurations using the default config
 *  @return      ::P3H2x4x_set_Configuration() void type.
 */
void P3H2x4x_set_Configuration(I3C_Hub_Configuration *i3c_hub_config);

/*! @brief       The interface function to read the target port data in smbus mode.
 *  @return      ::P3H2x4x_Target_Smbus_Read() returns the status .
 */
int P3H2x4x_Target_Smbus_Read(D_P3H2x4x_Handle *P3H2x4xDriver, uint8_t target_port, uint8_t Target_address, uint8_t BytesToRead, uint8_t RegisterToRead, uint8_t *recieve_buff);

/*! @brief       The interface function to write the target port data in smbus mode.
 *  @return      ::P3H2x4x_Target_Smbus_Write() returns the status .
 */
int32_t P3H2x4x_Target_Smbus_Write(D_P3H2x4x_Handle *P3H2x4xDriver, uint8_t target_port, uint8_t Target_address, uint8_t BytesToWrite, uint8_t *dataToWrite);

/*! @brief       The interface function to read the I3C target port data.
 *  @return      ::P3H2x4x_Target_I3C_read() returns the status .
 */
int P3H2x4x_Target_I3C_read(D_P3H2x4x_Handle *P3H2x4xDriver, uint8_t Target_address, uint8_t BytesToRead, uint8_t RegisterToRead, uint8_t *recieve_buff);

/*! @brief       The interface function to write the target port data in I3C mode.
 *  @return      ::P3H2x4x_Target_I3C_write() returns the status .
 */
int P3H2x4x_Target_I3C_write(D_P3H2x4x_Handle *P3H2x4xDriver, uint8_t Target_address, uint8_t RegisterToWrite, uint8_t dataToWrite, uint8_t shiftBy, uint8_t mask);

/*! @brief       The interface function to write the multiple bytes of target port data in I3C mode.
 *  @return      ::P3H2x4x_Target_I3C_write() returns the status .
 */
int P3H2x4x_Target_I3C_block_write(D_P3H2x4x_Handle *P3H2x4xDriver, uint8_t Target_address, uint8_t BytesToWrite, uint8_t RegisterToWrite, uint8_t *dataToWrite, uint8_t shiftBy, uint8_t mask);

/*! @brief       The interface function to read the target port in I2C mode.
 *  @return      ::P3H2x4x_Target_I2C_read() returns the status .
 */
int P3H2x4x_Target_I2C_read(D_P3H2x4x_Handle *P3H2x4xDriver, uint8_t Target_address, uint8_t BytesToRead, uint8_t RegisterToRead, uint8_t *recieve_buff);

/*! @brief       The interface function to write the target port in I2C mode.
 *  @return      ::P3H2x4x_Target_I2C_write() returns the status .
 */
int P3H2x4x_Target_I2C_write(D_P3H2x4x_Handle *P3H2x4xDriver, uint8_t Target_address, uint8_t RegisterToWrite, uint8_t dataToWrite, uint8_t shiftBy, uint8_t mask);

/*! @brief       The interface function to write the multiple bytes to target port in I2C mode.
 *  @return      ::P3H2x4x_Target_I2C_block_write() returns the status .
 */
int P3H2x4x_Target_I2C_block_write(D_P3H2x4x_Handle *P3H2x4xDriver, uint8_t Target_address, uint8_t BytesToWrite, uint8_t RegisterToWrite, uint8_t *dataToWrite, uint8_t shiftBy, uint8_t mask);

/*! @brief       The interface function to use to enable/disable the IBI.
 *  @return      ::P3H2x4x_Enable_Disable_IBI() returns the status .
 */
int P3H2x4x_Enable_Disable_IBI(D_P3H2x4x_Handle *P3H2x4xDriver);

/*! @brief       The interface function to use to assign dynamic address without reset.
 *  @return      ::P3H2x4x_Dynamic_addr_assgmt_without_rest() returns the status .
 */
int P3H2x4x_Dynamic_addr_assgmt_without_rest(uint8_t static_addr, uint8_t dynamic_addr);

/*! @brief       The interface function to use to assign the dynamic address with reset.
 *  @return      ::P3H2x4x_Dynamic_addr_assgmt_with_rest() returns the status .
 */
int P3H2x4x_Dynamic_addr_assgmt_with_rest(uint8_t static_addr, uint8_t dynamic_addr);
/*! @brief       The interface function to use to unlock protected write registers.
 *  @return      ::P3H2x4x_UnlockPrtcdReg() returns the status .
 */
int P3H2x4x_UnlockPrtcdReg(D_P3H2x4x_Handle *P3H2x4xDriver);

/*! @brief       The interface function to reset the HUB.
 *  @return      ::P3T1085_I2C_Initialize() returns the status .
 */
int P3H2x4x_Interface_Reset(D_P3H2x4x_Handle *P3H2x4xDriver);

/*! @brief       The interface function to use to configure P3H2x41 series.
 *  @return      ::P3H2x41_config() returns the status .
 */
int P3H2x41_config(D_P3H2x4x_Handle *P3H2x4xDriver);

/*! @brief       This function is to reset the I3C target device.
 *  @return      ::P3H2x4x_P3H2x4x_Target_device_reset() returns the status .
 */
void P3H2x4x_Target_device_reset(void);

/*! @brief       This function is to reset the I3C HUB device .
 *  @return      ::P3H2x4x_Reset_Device() returns the status .
 */
void P3H2x4x_Reset_Device(D_P3H2x4x_Handle *P3H2x4xDriver);

/*! @brief       This function is to initialise the reset pin of the hub .
 *  @return      ::P3H2x4x_Cp_sel_pin_init() returns the status .
 */
void P3H2x4x_Cp_sel_pin_init(void);

#endif /* I3C_HUB_I3C_HUB_DRV_H_ */

