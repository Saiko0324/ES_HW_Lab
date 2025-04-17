/**
  ******************************************************************************
  * @file    App/gatt_db.c
  * @author  SRA Application Team
  * @brief   Functions to build GATT DB and handle GATT events
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "bluenrg_def.h"
#include "gatt_db.h"
#include "bluenrg_conf.h"
#include "bluenrg_gatt_aci.h"

// USER CODE BEGIN
/** @brief Function that converts a 2-byte little-endian buffer to a 16-bit host value */
#define LE_TO_HOST_16(buf) ((uint16_t)(buf)[0] | ((uint16_t)(buf)[1] << 8))
// USER CODE END

/** @brief Macro that stores Value into a buffer in Little Endian Format (2 bytes)*/
#define HOST_TO_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8) ) )

/** @brief Macro that stores Value into a buffer in Little Endian Format (4 bytes) */
#define HOST_TO_LE_32(buf, val)    ( ((buf)[0] =  (uint8_t) (val)     ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[2] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[3] =  (uint8_t) (val>>24) ) )

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

/* Hardware Characteristics Service */
#define COPY_HW_SENS_W2ST_SERVICE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ACC_GYRO_MAG_W2ST_CHAR_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x00,0xE0,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
/* Software Characteristics Service */
#define COPY_SW_SENS_W2ST_SERVICE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x02,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_QUATERNIONS_W2ST_CHAR_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0x00,0x00,0x01,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

uint16_t HWServW2STHandle, EnvironmentalCharHandle, AccGyroMagCharHandle;
uint16_t SWServW2STHandle, QuaternionsCharHandle;

/* UUIDS */
Service_UUID_t service_uuid;
Char_UUID_t char_uuid;

extern AxesRaw_t x_axes;
extern AxesRaw_t g_axes;
extern AxesRaw_t m_axes;

extern uint16_t connection_handle;
extern uint32_t start_time;

// USER CODE BEGIN
// Define UUIDs for our custom service and characteristics
#define COPY_ACCEL_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xB0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00)
#define COPY_ACCEL_DATA_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0xB0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00)
#define COPY_ACCEL_FREQ_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0xB0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00)

// Service and characteristic handles
uint16_t AccelServiceHandle;
uint16_t AccelDataCharHandle;
uint16_t AccelFreqCharHandle;

// Default sampling frequency
uint16_t accelSamplingFreq = 10;
volatile uint8_t updateAccelData = 0;

/**
 * @brief  Add the Accelerometer Service and its characteristics
 * @retval Status
 */
tBleStatus Add_Accel_Service(void)
{
  tBleStatus ret;
  uint8_t uuid[16];

  // Add Accelerometer Service
  COPY_ACCEL_SERVICE_UUID(uuid);
  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid.Service_UUID_128, PRIMARY_SERVICE,
                          3*2, &AccelServiceHandle);
  if (ret != BLE_STATUS_SUCCESS)
    return BLE_STATUS_ERROR;

  // Add Accelerometer Data Characteristic (notify)
  COPY_ACCEL_DATA_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret = aci_gatt_add_char(AccelServiceHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                          3*2,  // xyz(3*2)
                          CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          16, 0, &AccelDataCharHandle);
  if (ret != BLE_STATUS_SUCCESS)
    return BLE_STATUS_ERROR;

  // Add Accelerometer Frequency Characteristic (read/write)
  COPY_ACCEL_FREQ_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret = aci_gatt_add_char(AccelServiceHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                          2,  // 2 bytes for frequency value (uint16_t)
                          CHAR_PROP_READ|CHAR_PROP_WRITE,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE|GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          16, 0, &AccelFreqCharHandle);
  if (ret != BLE_STATUS_SUCCESS)
    return BLE_STATUS_ERROR;

  // Initialize the frequency value
  uint8_t freqData[2];
  HOST_TO_LE_16(freqData, accelSamplingFreq);
  ret = aci_gatt_update_char_value(AccelServiceHandle, AccelFreqCharHandle,
                                  0, 2, freqData);
  if (ret != BLE_STATUS_SUCCESS)
    return BLE_STATUS_ERROR;

  PRINTF("Accelerometer Service added successfully.\n");
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update accelerometer characteristic value and notify client
 * @param  accelData Array of XYZ acceleration values
 * @retval Status
 */
tBleStatus Accel_Update(int16_t *accelData)
{
  uint8_t buff[6]; // 3*2 bytes XYZ data
  tBleStatus ret;

  // Add XYZ acceleration data
  HOST_TO_LE_16(buff, accelData[0]); // X
  HOST_TO_LE_16(buff+2, accelData[1]); // Y
  HOST_TO_LE_16(buff+4, accelData[2]); // Z

  ret = aci_gatt_update_char_value(AccelServiceHandle, AccelDataCharHandle,
                                   0, 6, buff);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error while updating Accelerometer data characteristic: 0x%02X\n", ret);
    return BLE_STATUS_ERROR;
  }

  return BLE_STATUS_SUCCESS;
}


// USER CODE END
/**
 * @brief  Add the 'HW' service (and the Environmental and AccGyr characteristics).
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_HWServW2ST_Service(void)
{
  tBleStatus ret;
  uint8_t uuid[16];

  /* Add_HWServW2ST_Service */
  COPY_HW_SENS_W2ST_SERVICE_UUID(uuid);
  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid.Service_UUID_128, PRIMARY_SERVICE,
                          1+3*5, &HWServW2STHandle);
  if (ret != BLE_STATUS_SUCCESS)
    return BLE_STATUS_ERROR;

  /* Fill the Environmental BLE Characteristc */
  COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid);
  uuid[14] |= 0x04; /* One Temperature value*/
  uuid[14] |= 0x10; /* Pressure value*/
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                           2+2+4,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &EnvironmentalCharHandle);
  if (ret != BLE_STATUS_SUCCESS)
    return BLE_STATUS_ERROR;

  /* Fill the AccGyroMag BLE Characteristc */
  COPY_ACC_GYRO_MAG_W2ST_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                           2+3*3*2,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &AccGyroMagCharHandle);
  if (ret != BLE_STATUS_SUCCESS)
    return BLE_STATUS_ERROR;

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Add the SW Feature service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_SWServW2ST_Service(void)
{
  tBleStatus ret;
  int32_t NumberOfRecords=1;
  uint8_t uuid[16];

  COPY_SW_SENS_W2ST_SERVICE_UUID(uuid);
  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid.Service_UUID_128, PRIMARY_SERVICE,
                          1+3*NumberOfRecords, &SWServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_QUATERNIONS_W2ST_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(SWServW2STHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                           2+6*SEND_N_QUATERNIONS,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &QuaternionsCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  return BLE_STATUS_SUCCESS;

fail:
  return BLE_STATUS_ERROR;
}

/**
 * @brief  Update acceleration characteristic value
 * @param  AxesRaw_t structure containing acceleration value in mg.
 * @retval tBleStatus Status
 */
tBleStatus Acc_Update(AxesRaw_t *x_axes, AxesRaw_t *g_axes, AxesRaw_t *m_axes)
{
  uint8_t buff[2+2*3*3];
  tBleStatus ret;

  HOST_TO_LE_16(buff,(HAL_GetTick()>>3));

  HOST_TO_LE_16(buff+2,-x_axes->AXIS_Y);
  HOST_TO_LE_16(buff+4, x_axes->AXIS_X);
  HOST_TO_LE_16(buff+6,-x_axes->AXIS_Z);

  HOST_TO_LE_16(buff+8,g_axes->AXIS_Y);
  HOST_TO_LE_16(buff+10,g_axes->AXIS_X);
  HOST_TO_LE_16(buff+12,g_axes->AXIS_Z);

  HOST_TO_LE_16(buff+14,m_axes->AXIS_Y);
  HOST_TO_LE_16(buff+16,m_axes->AXIS_X);
  HOST_TO_LE_16(buff+18,m_axes->AXIS_Z);

  ret = aci_gatt_update_char_value(HWServW2STHandle, AccGyroMagCharHandle,
				   0, 2+2*3*3, buff);
  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating Acceleration characteristic: 0x%02X\n",ret) ;
    return BLE_STATUS_ERROR ;
  }

  return BLE_STATUS_SUCCESS;
}

/*******************************************************************************
* Function Name  : Read_Request_CB.
* Description    : Update the sensor values.
* Input          : Handle of the characteristic to update.
* Return         : None.
*******************************************************************************/
void Read_Request_CB(uint16_t handle)
{
  tBleStatus ret;

  if(handle == AccGyroMagCharHandle + 1)
  {
    Acc_Update(&x_axes, &g_axes, &m_axes);
  }
  else if (handle == EnvironmentalCharHandle + 1)
  {
    float data_t, data_p;
    data_t = 27.0 + ((uint64_t)rand()*5)/RAND_MAX; //T sensor emulation
    data_p = 1000.0 + ((uint64_t)rand()*100)/RAND_MAX; //P sensor emulation
    BlueMS_Environmental_Update((int32_t)(data_p *100), (int16_t)(data_t * 10));
  }

  if(connection_handle !=0)
  {
    ret = aci_gatt_allow_read(connection_handle);
    if (ret != BLE_STATUS_SUCCESS)
    {
      PRINTF("aci_gatt_allow_read() failed: 0x%02x\r\n", ret);
    }
  }
}

/**
 * @brief  Handle GATT write requests
 * @param  attr_handle Handle of the attribute
 * @param  data_length Length of the data
 * @param  data Data to be written
 * @retval None
 */
void Write_Request_CB(uint16_t attr_handle, uint8_t data_length, uint8_t *data)
{
  if(attr_handle == AccelFreqCharHandle + 1) {
    // Extract frequency value from data (2 bytes, little-endian)
    accelSamplingFreq = LE_TO_HOST_16(data);

    // Validate the frequency (set bounds)
    if(accelSamplingFreq < 1) accelSamplingFreq = 1;     // Minimum 1 Hz
    if(accelSamplingFreq > 1000) accelSamplingFreq = 100; // Maximum 100 Hz

    PRINTF("New accelerometer sampling period: %.1f s\n", (float)accelSamplingFreq * 0.1);

    // Update the characteristic value with validated frequency
    uint8_t freqData[2];
    HOST_TO_LE_16(freqData, accelSamplingFreq);
    aci_gatt_update_char_value(AccelServiceHandle, AccelFreqCharHandle,
                              0, 2, freqData);
  }
}

tBleStatus BlueMS_Environmental_Update(int32_t press, int16_t temp)
{
  tBleStatus ret;
  uint8_t buff[8];
  HOST_TO_LE_16(buff, HAL_GetTick()>>3);

  HOST_TO_LE_32(buff+2,press);
  HOST_TO_LE_16(buff+6,temp);

  ret = aci_gatt_update_char_value(HWServW2STHandle, EnvironmentalCharHandle,
                                   0, 8, buff);

  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating TEMP characteristic: 0x%04X\n",ret) ;
    return BLE_STATUS_ERROR ;
  }

  return BLE_STATUS_SUCCESS;
}
