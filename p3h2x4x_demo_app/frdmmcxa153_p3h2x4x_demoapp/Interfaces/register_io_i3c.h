/*
* Copyright 2025 NXP
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef INTERFACES_REGISTER_IO_I3C_H_
#define INTERFACES_REGISTER_IO_I3C_H_

#include "sensor_drv.h"
#include "DRIVER_I3C.h"

#define I3C_INDEX 0

void I3C_SignalEvent_t(uint32_t event);

/*!
 * @brief The interface function to write a sensor register.
 *s
 * @param ARM_DRIVER_I3C *pCommDrv - The I3C driver to use.
 * @param registerDeviceInfo_t *devInfo - The I3C device number and idle function.
 * @param uint16_t slaveAddress - the sensor's I3C slave address.
 * @param uint8_t offset - The register/offset to write to.
 * @param uint8_t *pBuffer - The buffer containing bytes to write.
 * @param uint8_t bytesToWrite - A number of bytes to write.
 *
 * @return ARM_DRIVER_OK if success or ARM_DRIVER_ERROR if error.
 */
int32_t Register_I3C_BlockWrite(ARM_DRIVER_I3C *pCommDrv,
                                registerDeviceInfo_t *devInfo,
                                uint16_t slaveAddress,
                                uint8_t offset,
                                const uint8_t *pBuffer,
                                uint8_t bytesToWrite,
								bool is_i3c_mode);

/*!
 * @brief The interface function to write a sensor register.
 *
 * @param ARM_DRIVER_I3C *pCommDrv - The I3C driver to use.
 * @param registerDeviceInfo_t *devInfo - The I3C device number and idle function.
 * @param uint16_t slaveAddress - the sensor's I3C slave address.
 * @param uint8_t offset - The register/offset to write to
 * @param uint8_t value - The value to write to the register
 * @param uint8_t mask - A mask value to use when writing.
 *                       A non-zero mask indicates that a read-modify-write operation should be used.
 *                       where only the bits set in the mask will be updated according to the value param.
 * @param bool repeatedStart - Indicates whether to send STOP or REPEATED_START bit after the write
 *
 * @return ARM_DRIVER_OK if success or ARM_DRIVER_ERROR if error.
 */
int32_t Register_I3C_Write(ARM_DRIVER_I3C *pCommDrv,
                           registerDeviceInfo_t *devInfo,
                           uint16_t slaveAddress,
                           uint8_t offset,
                           uint8_t value,
                           uint8_t mask,
                           bool repeatedStart,
						   bool is_i3c_mode);

/*!
 * @brief The interface function to read a sensor register.
 *
 * @param ARM_DRIVER_I3C *pCommDrv - The I3C driver to use.
 * @param registerDeviceInfo_t *devInfo - The I3C device number and idle function.
 * @param uint16_t slaveAddress - the sensor's I3C slave address.
 * @param uint8_t offset - The register/offset to read from
 * @param uint8_t length - The number of bytes to read
 * @param uint8_t *pOutBuffer - The pointer to the buffer to store the register value read.
 *
 * @return ARM_DRIVER_OK if success or ARM_DRIVER_ERROR if error.
 */
int32_t Register_I3C_Read(ARM_DRIVER_I3C *pCommDrv,
                          registerDeviceInfo_t *devInfo,
                          uint16_t slaveAddress,
                          uint8_t offset,
                          uint8_t length,
                          uint8_t *pOutBuffer,
						  bool is_i3c_mode);


#endif /* INTERFACES_REGISTER_IO_I3C_H_ */

/*!
 * @brief The interface function to write a sensor register.
 *
 * @param ARM_DRIVER_I3C *pCommDrv - The I2C driver to use.
 * @param registerDeviceInfo_t *devInfo - The I2C device number and idle function.
 * @param uint16_t slaveAddress - the sensor's I2C slave address.
 * @param uint8_t offset - The register/offset to write to
 * @param uint8_t value - The value to write to the register
 * @param uint8_t mask - A mask value to use when writing.
 *                       A non-zero mask indicates that a read-modify-write operation should be used.
 *                       where only the bits set in the mask will be updated according to the value param.
 * @param bool repeatedStart - Indicates whether to send STOP or REPEATED_START bit after the write
 *
 * @return ARM_DRIVER_OK if success or ARM_DRIVER_ERROR if error.
 */
int32_t Register_I2C_Write(ARM_DRIVER_I3C *pCommDrv,
                           registerDeviceInfo_t *devInfo,
                           uint16_t slaveAddress,
                           uint8_t offset,
                           uint8_t value,
                           uint8_t mask,
                           bool repeatedStart,
						   bool is_i3c_mode);
