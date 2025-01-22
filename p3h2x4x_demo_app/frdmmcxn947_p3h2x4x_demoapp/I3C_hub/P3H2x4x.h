/*
* Copyright 2025 NXP
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef I3C_HUB_I3C_HUB_H_
#define I3C_HUB_I3C_HUB_H_

#include <stdio.h>
#include <stdint.h>

typedef enum{
	/* Device Information Registers */
	DEV_INFO_0_addr = 0x00,
	DEV_INFO_1_addr = 0x01,
	PID_BYTE_0_addr = 0x02,
	PID_BYTE_1_addr = 0x03,
	PID_BYTE_2_addr = 0x04,
	PID_BYTE_3_addr = 0x05,
	PID_BYTE_4_addr = 0x06,
	PID_BYTE_5_addr = 0x07,
	BCR_addr 		= 0x08,
	DCR_addr 		= 0x09,
	HUB_DEV_CAP_addr = 0x0A,
	HUB_DEV_REV_addr = 0x0B,

	/* Device Configuration Registers */
	DEV_REG_PROT_CODE_addr = 0x10,
	CONTROLLER_PORT_CONF_addr = 0x11,
	TARGET_PORT_EN_addr = 0x12,
	HUB_DEV_CONF_addr = 0x13,
	IO_DS_CONF_addr = 0x14,
	HUB_NET_OP_MODE_CONF_addr = 0x15,
	VCCIO_VOLTAGE_addr = 0x16,
	TARGET_IO_SIGNAL_MODE_addr = 0x17,
	TARGET_SMBUS_EN_addr = 0x18,
	LDO_EN_PULL_UP_RES_VALUE_addr = 0x19,
	CONTROLLER_IBI_GEN_CONF_addr = 0x1A,
	TARGET_SMBUS_IBI_CONF_addr = 0X1B,
	USER_IBI_MDB_VALUE_addr = 0x1C,
	JEDEC_CONTEXT_ID_addr = 0x1D,
	TARGET_GPIO_MODE_EN_addr = 0x1E,

	/* Device Status and IBI Registers */
	DEVICE_IBI_STATUS_addr = 0x20,
	TARGET_SMBUS_IBI_STATUS_addr = 0x21,
	REF_CONTROLLER_IBI_MESSAGE_0_addr = 0x22,
	REF_CONTROLLER_IBI_MESSAGE_1_addr = 0x23,
	REF_CONTROLLER_IBI_MESSAGE_2_addr = 0x24,
	REF_CONTROLLER_IBI_MESSAGE_3_addr = 0x25,
	CURR_CONTROLLER_IBI_MESSAGE_0_addr = 0x28,
	CURR_CONTROLLER_IBI_MESSAGE_1_addr = 0x29,
	CURR_CONTROLLER_IBI_MESSAGE_2_addr = 0x2A,
	CURR_CONTROLLER_IBI_MESSAGE_3_addr = 0x2B,
	CONTROLLER_IBI_MESSAGE_REQ_addr = 0x2C,
	GENERAL_MESSAGE_BUFFER_0_addr = 0x30,
	GENERAL_MESSAGE_BUFFER_1_addr =  0x31,
	GENERAL_MESSAGE_BUFFER_2_addr = 0x32,
	GENERAL_MESSAGE_BUFFER_3_addr =0x33,

	/* Controller Port Control/Status Registers */
	CONTROLLER_MUX_REQ_addr = 0x38,
	CONTROLLER_MUX_SEL_STATUS_addr = 0x39,

	/* Target Ports Control Registers */
	I3C_HUB_TP_SMBUS_AGNT_TRANS_START = 0X50,
	TARGET_HUB_NET_CONN_EN_addr = 0x51,
	TARGET_PULL_UP_EN_addr = 0x53,
	TARGET_SCL_GPIO_OUTPUT_EN_addr = 0x54,
	TARGET_SDA_GPIO_OUTPUT_EN_addr = 0x55,
	TARGET_SCL_GPIO_OUTPUT_LEVEL_addr = 0x56,
	TARGET_SDA_GPIO_OUTPUT_LEVEL_addr = 0x57,
	TARGET_INPUT_DET_CONF_addr = 0x58,
	TARGET_SCL_INPUT_DET_IBI_EN_addr = 0x59,
	TARGET_SDA_INPUT_DET_IBI_EN_addr = 0x5A,

	/* Target Ports Status Registers */
	TARGET_SCL_INPUT_STATUS_addr = 0x60,
	TARGET_SDA_INPUT_STATUS_addr = 0x61,
	TARGET_SCL_INPUT_DET_FLAG_addr = 0x62,
	TARGET_SDA_INPUT_DET_FLAG_addr = 0x63,

	/* SMBus Agent Configuration and Status Registers */
	 I3C_HUB_TP0_SMBUS_AGNT_STS		=	0x64,
	 I3C_HUB_TP1_SMBUS_AGNT_STS		=	0x65,
	 I3C_HUB_TP2_SMBUS_AGNT_STS		=	0x66,
	 I3C_HUB_TP3_SMBUS_AGNT_STS		=	0x67,
	 I3C_HUB_TP4_SMBUS_AGNT_STS		=	0x68,
	 I3C_HUB_TP5_SMBUS_AGNT_STS		=	0x69,
     I3C_HUB_TP6_SMBUS_AGNT_STS		=	0x6A,
	 I3C_HUB_TP7_SMBUS_AGNT_STS		=	0x6B,
	 I3C_HUB_ONCHIP_TD_AND_SMBUS_AGNT_CONF	=	0x6C,

	/* Special Function Registers */
	LDO_CP_STATUS_addr = 0x79,
	BUS_RST_SCL_TIMEOUT_SETTING_addr = 0x7A,
	I3C_TARGET_DEV_ERROR_FLAG_addr = 0x7B,
	HUB_DEV_CMD_addr = 0x7C,
	TARGET_DEV_STATUS_addr = 0x7D,
	TARGET_DEV_ADDR_SETTING_addr = 0x7E,
	PAGE_REG_POINTER_addr = 0x7F,

	/* Paged Transaction Registers */
	I3C_HUB_CONTROLLER_BUFFER_PAGE		=	0x10,
	I3C_HUB_CONTROLLER_AGENT_BUFF		=	0x80,
	I3C_HUB_CONTROLLER_AGENT_BUFF_DATA	=	0x84,
	I3C_HUB_TARGET_BUFF_ADDRESS			=	0x81,
	I3C_HUB_TARGET_BUFF_DATA			=	0x82,

	/* Transaction status checking mask */
	I3C_HUB_XFER_SUCCESS			 =	    0x01,
	I3C_HUB_TP_BUFFER_STATUS_MASK	 =		0x0F,
	I3C_HUB_TP_TRANSACTION_CODE_MASK =		0xF0,

} e_P3H2840_Registers;

#endif /* I3C_HUB_I3C_HUB_H_ */
