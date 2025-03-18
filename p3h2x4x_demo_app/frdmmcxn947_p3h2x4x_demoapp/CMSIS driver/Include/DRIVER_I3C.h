#ifndef CMSIS_DRIVER_INCLUDE_DRIVER_I3C_H_
#define CMSIS_DRIVER_INCLUDE_DRIVER_I3C_H_

/*
 * Copyright (c) 2013-2020 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * $Date:        31. March 2020
 * $Revision:    V2.4
 *
 * Project:      I3C (Inter-Integrated Circuit) Driver definitions
 */

/* History:
 *  Version 2.4
 *    Removed volatile from ARM_I3C_STATUS
 *  Version 2.3
 *    ARM_I3C_STATUS made volatile
 *  Version 2.2
 *    Removed function ARM__MasterTransfer in order to simplify drivers
 *      and added back parameter "xfer_pending" to functions
 *      ARM_I3C_MasterTransmit and ARM_I3C_MasterReceive
 *  Version 2.1
 *    Added function ARM_I3C_MasterTransfer and removed parameter "xfer_pending"
 *      from functions ARM_I3C_MasterTransmit and ARM_I3C_MasterReceive
 *    Added function ARM_I3C_GetDataCount
 *    Removed flag "address_nack" from ARM_I3C_STATUS
 *    Replaced events ARM_I3C_EVENT_MASTER_DONE and ARM_I3C_EVENT_SLAVE_DONE
 *      with event ARM_I3C_EVENT_TRANSFER_DONE
 *    Added event ARM_I3C_EVENT_TRANSFER_INCOMPLETE
 *    Removed parameter "arg" from function ARM_I3C_SignalEvent
 *  Version 2.0
 *    New simplified driver:
 *      complexity moved to upper layer (especially data handling)
 *      more unified API for different communication interfaces
 *    Added:
 *      Slave Mode
 *    Changed prefix ARM_DRV -> ARM_DRIVER
 *  Version 1.10
 *    Namespace prefix ARM_ added
 *  Version 1.00
 *    Initial release
 */

#ifdef  __cplusplus
extern "C"
{
#endif

#include "Driver_Common.h"

#define ARM_I3C_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2,4)  /* API version */


#define _ARM_DRIVER_I3C_(n)      Driver_I3C##n
#define  ARM_DRIVER_I3C_(n) _ARM_DRIVER_I3C_(n)


/****** I3C Control Codes *****/

#define ARM_I3C_OWN_ADDRESS             (0x01UL)    ///< Set Own Slave Address; arg = address
#define ARM_I3C_BUS_SPEED               (0x02UL)    ///< Set Bus Speed; arg = speed
#define ARM_I3C_BUS_CLEAR               (0x03UL)    ///< Execute Bus clear: send nine clock pulses
#define ARM_I3C_ABORT_TRANSFER          (0x04UL)    ///< Abort Master/Slave Transmit/Receive

/*----- I3C Bus Speed -----*/
#define ARM_I3C_BUS_SPEED_STANDARD      (0x01UL)    ///< Standard Speed (100kHz)
#define ARM_I3C_BUS_SPEED_FAST          (0x02UL)    ///< Fast Speed     (400kHz)
#define ARM_I3C_BUS_SPEED_FAST_PLUS     (0x03UL)    ///< Fast+ Speed    (  1MHz)
#define ARM_I3C_BUS_SPEED_HIGH          (0x04UL)    ///< High Speed     (3.4MHz)


/****** I3C Address Flags *****/

#define ARM_I3C_ADDRESS_10BIT           (0x0400UL)  ///< 10-bit address flag
#define ARM_I3C_ADDRESS_GC              (0x8000UL)  ///< General Call flag


/**
\brief I3C Status
*/
typedef struct _ARM_I3C_STATUS {
  uint32_t busy             : 1;        ///< Busy flag
  uint32_t mode             : 1;        ///< Mode: 0=Slave, 1=Master
  uint32_t direction        : 1;        ///< Direction: 0=Transmitter, 1=Receiver
  uint32_t general_call     : 1;        ///< General Call indication (cleared on start of next Slave operation)
  uint32_t arbitration_lost : 1;        ///< Master lost arbitration (cleared on start of next Master operation)
  uint32_t bus_error        : 1;        ///< Bus error detected (cleared on start of next Master/Slave operation)
  uint32_t reserved         : 26;
} ARM_I3C_STATUS;


/****** I3C Event *****/
#define ARM_I3C_EVENT_TRANSFER_DONE       (1UL << 0)  ///< Master/Slave Transmit/Receive finished
#define ARM_I3C_EVENT_TRANSFER_INCOMPLETE (1UL << 1)  ///< Master/Slave Transmit/Receive incomplete transfer
#define ARM_I3C_EVENT_SLAVE_TRANSMIT      (1UL << 2)  ///< Addressed as Slave Transmitter but transmit operation is not set.
#define ARM_I3C_EVENT_SLAVE_RECEIVE       (1UL << 3)  ///< Addressed as Slave Receiver but receive operation is not set.
#define ARM_I3C_EVENT_ADDRESS_NACK        (1UL << 4)  ///< Address not acknowledged from Slave
#define ARM_I3C_EVENT_GENERAL_CALL        (1UL << 5)  ///< Slave addressed with general call address
#define ARM_I3C_EVENT_ARBITRATION_LOST    (1UL << 6)  ///< Master lost arbitration
#define ARM_I3C_EVENT_BUS_ERROR           (1UL << 7)  ///< Bus error detected (START/STOP at illegal position)
#define ARM_I3C_EVENT_BUS_CLEAR           (1UL << 8)  ///< Bus clear finished

typedef void (*ARM_I3C_SignalEvent_t) (uint32_t event);  ///< Pointer to \ref ARM_I3C_SignalEvent : Signal I3C Event.

/**
\brief I3C Driver Capabilities.
*/
typedef struct _ARM_I3C_CAPABILITIES {
  uint32_t address_10_bit : 1;          ///< supports 10-bit addressing
  uint32_t reserved       : 31;         ///< Reserved (must be zero)
} ARM_I3C_CAPABILITIES;

/*ARM I3C DRIVER*/
typedef struct _ARM_DRIVER_I3C{
	  ARM_DRIVER_VERSION   (*GetVersion)     								  (void);                                                                		//< Pointer to \ref ARM_I3C_GetVersion : Get driver version.
	  ARM_I3C_CAPABILITIES (*GetCapabilities)								  (void);                                                                		//< Pointer to \ref ARM_I3C_GetCapabilities : Get driver capabilities.
	  int32_t               (*Initialize)     								  (ARM_I3C_SignalEvent_t cb_event);                                      		//< Pointer to \ref ARM_I3C_Initialize : Initialize I3C Interface.
	  int32_t              (*PowerControl)   								  (ARM_POWER_STATE state);                                               		//< Pointer to \ref ARM_I3C_PowerControl : Control I3C Interface Power.
	  int32_t              (*LPI3C_dynamic_address_assignment)                (uint8_t staticAddess, uint8_t dynamicAddress);                               //< Pointer to \ref ARM_I3C_Initialize : Initialize I3C Interface.
	  int32_t              (*LPI3C_dynamic_address_assignment_without_reset)  (uint8_t staticAddess, uint8_t dynamicAddress);                               //< Pointer to \ref ARM_I3C_Initialize : Initialize I3C Interface.
	  int32_t              (*MasterTransmit) 								  (uint32_t subaddr, const uint8_t *data, uint32_t num, bool xfer_pending, bool is_i3c_mode); //< Pointer to _I3C_MasterTransmit : Start transmitting data as I3C Master.
	  int32_t              (*MasterReceive)  								  (uint32_t subaddr, const uint8_t *data, uint32_t num, bool xfer_pending, bool is_i3c_mode); //< Pointer to _I3C_MasterReceive : Start receiving data as I3C Master.
	  int32_t              (*DirectENEC) 								  	  (uint16_t address, uint16_t cmd); 											//< To enable IBI
	  int32_t              (*DirectDISEC)  								      (uint16_t address, uint16_t cmd); 											//< To disable IBI
	  int32_t              (*I2C_MasterTransmit)  							  (uint32_t subaddr, const uint8_t *data, uint32_t num, bool xfer_pending, bool is_i3c_mode); //< Pointer to \ref ARM_I2C_SlaveTransmit : Start transmitting data as I3C Slave.
} const ARM_DRIVER_I3C;

#ifdef  __cplusplus
}
#endif


#endif /* CMSIS_DRIVER_INCLUDE_DRIVER_I3C_H_ */
