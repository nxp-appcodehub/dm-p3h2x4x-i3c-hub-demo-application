/*
* Copyright 2025 NXP
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef P3H2X4X_TEMP_SENSOR_DRV_H_
#define P3H2X4X_TEMP_SENSOR_DRV_H_

#include "register_io_i3c.h"

#define P3T1755DP_REG_SIZE_BYTES	2
#define P3T1755DP_CELCIUS_CONV_VAL	0.0625
#define P3T1755DP_TEMP_IGNORE_SHIFT	((uint16_t)4)

#define P3T1755DP_TEMP_NEGPOS_MASK       ((uint16_t)0x8000)
#define P3T1755DP_TEMP_NEGPOS_SHIFT       ((uint16_t)15)

#define P3T1755DP_MAX_THIGH_VALUE_CEL	125
#define P3T1755DP_MIN_TLOW_VALUE_CEL	-40

#define P3T1755DP_GET_TEMP						0x00
#define P3T1755DP_GET_LOW_TEMP					0x02
#define P3T1755DP_GET_HIGH_TEMP					0x03

typedef struct
{
    registerDeviceInfo_t deviceInfo; /*!< I2C device context. */
    ARM_DRIVER_I3C *pCommDrv;        /*!< Pointer to the i2c driver. */
    bool isInitialized;              /*!< whether sensor is intialized or not.*/
    uint16_t slaveAddress;           /*!< slave address.*/
} p3t1755_sensorhandle_t;

/*! @brief       The interface function to initialize the sensor.
 *  @details     This function initialize the sensor and sensor handle.
 *  @param[in]   pSensorHandle  handle to the sensor.
 *  @param[in]   pBus           pointer to the CMSIS API compatible I2C bus object.
 *  @param[in]   index          the I2C device number.
 *  @param[in]   sAddress       slave address of the device on the bus.
 *  @constraints This should be the first API to be called.
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::P3T1755_Initialize() returns the status .
 */
int32_t P3T1755_Initialize(p3t1755_sensorhandle_t *pSensorHandle, ARM_DRIVER_I3C *pBus, uint8_t index, uint16_t sAddress);

/*! @brief       The interface function to get the float value
 *  @details     This function read the temperature with a float value.
 *  @return      ::P3T1755_GetTempFloatValue() returns the status .
 */
int32_t P3T1755_GetTempFloatValue(uint8_t * pBuffer, float * fTemp);

/*! @brief       The interface function to get temperature value
 *  @details     This function converts the float temperature value into integar.
 *  @return      ::P3T1755_I2C_GetTempRegValue() returns the status .
 */
int32_t P3T1755_I2C_GetTempRegValue(float fTemp, uint8_t * pBuffer, uint8_t is_i3c);

/*! @brief       The interface function to get the temperature of the sensor.
 *  @details     This function read the temperature of sensor device and returns raw data in celsius.
 *  @param[in]   pSensorHandle handle to the sensor.
 *  @param[out]  pBuffer       handle to the output buffer
 *  @constraints None
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::P3T1755_GetTemp() returns the status .
 */
int32_t P3T1755_GetTemp(D_P3H2x4x_Handle *P3H2x4xDriver, p3t1755_sensorhandle_t *pSensorHandle, int target_port, uint8_t target_address, bool in_I2C_mode, float *pBuffer);

/*! @brief       The interface function to set the high limit Temperature of the sensor.
 *  @details     This function write the temperature into high limit temperature register of sensor device.
 *  @param[in]   pSensorHandle handle to the sensor.
 *  @param[in]   fTemp         Temperature in celsius
 *  @constraints None
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::P3T1755_GetTHigh() returns the status .
 */
int32_t P3T1755_GetTHigh(D_P3H2x4x_Handle *P3H2x4xDriver, p3t1755_sensorhandle_t *pSensorHandle, int target_port, uint8_t target_address, bool in_I2C_mode, float *pBuffer);

/*! @brief       The interface function to get the low limit Temperature of the sensor.
 *  @details     This function read low limit temperature register of sensor device.
 *  @param[in]   pSensorHandle handle to the sensor.
 *  @param[out]  pBuffer       handle to the output buffer
 *  @constraints None
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::P3T1755_GetTLow() returns the status .
 */
int32_t P3T1755_GetTLow(D_P3H2x4x_Handle *P3H2x4xDriver, p3t1755_sensorhandle_t *pSensorHandle, int target_port, uint8_t target_address, bool in_I2C_mode, float *pBuffer);

/*! @brief       The interface function to set the high limit Temperature of the sensor.
 *  @details     This function write the temperature into high limit temperature register of sensor device.
 *  @param[in]   pSensorHandle handle to the sensor.
 *  @param[in]   fTemp         Temperature in celsius
 *  @constraints None
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::P3T1755_SetTHigh() returns the status .
 */
int32_t P3T1755_SetTHigh(D_P3H2x4x_Handle *P3H2x4xDriver, p3t1755_sensorhandle_t *pSensorHandle, int target_port, uint8_t target_address, bool in_I2C_mode, float fTemp);

/*! @brief       The interface function to set the low limit Temperature of the sensor.
 *  @details     This function write the temperature into low limit temperature register of sensor device.
 *  @param[in]   pSensorHandle handle to the sensor.
 *  @param[in]   fTemp         Temperature in celsius
 *  @constraints None
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::P3T1755_SetTLow() returns the status .
 */
int32_t P3T1755_SetTLow(D_P3H2x4x_Handle *P3H2x4xDriver, p3t1755_sensorhandle_t *pSensorHandle, int target_port, uint8_t target_address, bool in_I2C_mode, float fTemp);

#endif /* P3H2X4X_TEMP_SENSOR_DRV_H_ */
