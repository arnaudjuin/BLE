/**
 ******************************************************************************
 * @file    Source/App/Ble/ble_service.c
 * @author  Arthur Burnichon
 * @date    20-Oct-2017
 * @brief   Ble services component implementation
 ******************************************************************************
 * @attention
 *
 * &copy; COPYRIGHT(c) 2017 Arthur Burnichon - arthur@meetluseed.com
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ble_service.h"
#include "ble_common.h"

#include "bluenrg_gatt_aci.h"
#include "stm32_debug.h"
#include "Button/button.h"
#include "Led/led.h"

/* Private macro -------------------------------------------------------------*/
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
    do {\
        uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
        uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
        uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
    }while(0)

/* Private define ------------------------------------------------------------*/

#define COPY_LED_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct, 0xaf, 0xce, 0xb8, 0xe0, 0x03, 0x77, 0x11, 0xe0, 0x5a, 0xc2, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b)
#define COPY_LED_READWRITE(uuid_struct)     COPY_UUID_128(uuid_struct, 0xa4, 0xc3, 0xff, 0xc1, 0xaa, 0x17, 0x11, 0xe0, 0x5a, 0xba, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b)
#define COPY_LED_BLINK(uuid_struct)         COPY_UUID_128(uuid_struct, 0xa3, 0xc2, 0xfe, 0xc0, 0xa9, 0x16, 0x11, 0xe0, 0x5a, 0xb2, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b)
#define COPY_BUTTON_SERVICEUUID(uuid_struct)  COPY_UUID_128(uuid_struct, 0xC6, 0x09, 0xB3, 0xC3, 0x09, 0xf8, 0x11, 0xe7, 0x53, 0x9A, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b)
#define COPY_BUTTON_NBPRESSED(uuid_struct)  COPY_UUID_128(uuid_struct, 0x9A, 0xB6, 0x73, 0x9C, 0x00, 0x65, 0x11, 0xe7, 0x11, 0xd7, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b)
#define COPY_ACCELERATOR_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct, 0x02, 0x36, 0x6e, 0x80, 0xcf, 0x3a, 0x11, 0xe1, 0x9A, 0xB4, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b)
#define COPY_ACCELETATOR_SENSOR (uuid_struct)       COPY_UUID_128(uuid_struct, 0x34, 0x0A, 0x1B, 0x80, 0xcf, 0x4B, 0x11, 0xe1, 0xAC, 0x36, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b)
#define COPY_ENVISENSOR_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct, 0x42, 0x82, 0x1a, 0x40, 0xe4, 0x77, 0x11, 0xe2, 0x82, 0xd0, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b)
#define COPY_TEMPERATURE_SENSOR (uuid_struct)       COPY_UUID_128(uuid_struct, 0xA3, 0x2E, 0x55, 0x20, 0xE4, 0x77, 0x11, 0xe2, 0xA9, 0xe3, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b)
#define COPY_PRESSURE_SENSOR (uuid_struct)       COPY_UUID_128(uuid_struct, 0xCD, 0x20, 0xC4, 0x80, 0xE4, 0x8B, 0x11, 0xe2, 0x84, 0x0B, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b)
#define COPY_HUMIDITY_SENSOR (uuid_struct)  COPY_UUID_128(uuid_struct, 0x01, 0xC5, 0x0B, 0x60, 0xE4, 0x8C, 0x11, 0xe2, 0xA0, 0x73, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b)
#define COPY_MOTION_SERVICE_UUID(uuid_struct)	 COPY_UUID_128(uuid_struct, 0xAC, 0x59, 0xAF, 0x24, 0x78, 0x19, 0x11, 0xe3, 0x9C, 0x62, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b)
#define COPY_INCLINATION_SERVICE(uuid_struct)	 COPY_UUID_128(uuid_struct, 0x53, 0xFC, 0xB3, 0x92, 0x10, 0x29, 0x11, 0xe3, 0x63, 0x90, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b)




/* Private variables ---------------------------------------------------------*/
uint16_t led_serv_handle, led_readwrite_char_handle, led_blink_char_handle, button_serv_handle, button_nbpressed,accelero_serv_handle, accelero_read,envi_sensor_serv_handle,humidity_read,temperature_read,pressure_read,motion_handle,inclination_read;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
tBleStatus Ble_Service_Led_Init(void)
{
    tBleStatus result;
    uint8_t uuid[16];

    // copy "LED service UUID" defined above to 'uuid' local variable
    COPY_LED_SERVICE_UUID(uuid);

    result = aci_gatt_add_serv(UUID_TYPE_128, uuid, PRIMARY_SERVICE, 7, &led_serv_handle);

    if (result == BLE_STATUS_SUCCESS) {
        // copy "LED button characteristic UUID" defined above to 'uuid' local variable
        COPY_LED_READWRITE(uuid);
        result =  aci_gatt_add_char(led_serv_handle, UUID_TYPE_128, uuid, 1, CHAR_PROP_NOTIFY | CHAR_PROP_READ | CHAR_PROP_WRITE, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE, 16, 0, &led_readwrite_char_handle);
        if (result != BLE_STATUS_SUCCESS){
            DEBUG_PRINTF("Error while adding LED service.\n");
        }

        COPY_LED_BLINK(uuid);
        result =  aci_gatt_add_char(led_serv_handle, UUID_TYPE_128, uuid, 4, CHAR_PROP_WRITE, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP, 16, 0, &led_blink_char_handle);
    }

    if (result != BLE_STATUS_SUCCESS){
        DEBUG_PRINTF("Error while adding LED service.\n");
    }

    return result;
}


tBleStatus Ble_Service_Envi_sensors_Init(void)
{
    tBleStatus result;
    uint8_t uuid[16];

    // copy "LED service UUID" defined above to 'uuid' local variable
    COPY_ENVISENSOR_SERVICE_UUID(uuid);

    result = aci_gatt_add_serv(UUID_TYPE_128, uuid, PRIMARY_SERVICE, 10, &envi_sensor_serv_handle);
    if (result == BLE_STATUS_SUCCESS) {
    	COPY_HUMIDITY_SENSOR (uuid);
        result =  aci_gatt_add_char(envi_sensor_serv_handle, UUID_TYPE_128, uuid, 2, CHAR_PROP_NOTIFY | CHAR_PROP_READ, ATTR_PERMISSION_NONE,  GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP, 16, 0, &humidity_read);
        if (result != BLE_STATUS_SUCCESS){
            DEBUG_PRINTF("Error while adding env sensor (humidity) service.\n");
        }

        COPY_TEMPERATURE_SENSOR(uuid);
        result =  aci_gatt_add_char(envi_sensor_serv_handle, UUID_TYPE_128, uuid, 2, CHAR_PROP_NOTIFY | CHAR_PROP_READ, ATTR_PERMISSION_NONE,  GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP, 16, 0, &temperature_read);
        if (result != BLE_STATUS_SUCCESS){
                DEBUG_PRINTF("Error while adding env sensor (temp) service.\n");
            }
            COPY_PRESSURE_SENSOR(uuid);
                  result =  aci_gatt_add_char(envi_sensor_serv_handle, UUID_TYPE_128, uuid, 3, CHAR_PROP_NOTIFY | CHAR_PROP_READ, ATTR_PERMISSION_NONE,  GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP, 16, 0, &pressure_read);

                  if (result != BLE_STATUS_SUCCESS){
                                  DEBUG_PRINTF("Error while adding env sensor (pressure) service.\n");
                              }
    }








    return result;
}

tBleStatus Ble_Service_Inclination_Init(void)			///On initialise de la même manière que la LED et le button
{
    tBleStatus result;
    uint8_t uuid[16];

    COPY_INCLINATION_SERVICE(uuid);

    result = aci_gatt_add_serv(UUID_TYPE_128, uuid, PRIMARY_SERVICE, 7, &motion_handle);

    if (result == BLE_STATUS_SUCCESS) {
        // copy "LED button characteristic UUID" defined above to 'uuid' local variable
    	COPY_BUTTON_NBPRESSED(uuid);
        result =  aci_gatt_add_char(motion_handle, UUID_TYPE_128, uuid, 4, CHAR_PROP_NOTIFY | CHAR_PROP_READ, ATTR_PERMISSION_NONE,  GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP, 16, 0, &inclination_read);
        if (result != BLE_STATUS_SUCCESS){
            DEBUG_PRINTF("Error while adding Accelero service.\n");
        }
    	DEBUG_PRINTF("Bien lancé");

}
    return result;
}


tBleStatus Ble_Service_Accelero_Init(void)			///On initialise de la même manière que la LED et le button
{
    tBleStatus result;
    uint8_t uuid[16];

    COPY_ACCELERATOR_SERVICE_UUID (uuid);

    result = aci_gatt_add_serv(UUID_TYPE_128, uuid, PRIMARY_SERVICE, 7, &accelero_serv_handle);

    if (result == BLE_STATUS_SUCCESS) {
        // copy "LED button characteristic UUID" defined above to 'uuid' local variable
    	COPY_BUTTON_NBPRESSED(uuid);
        result =  aci_gatt_add_char(accelero_serv_handle, UUID_TYPE_128, uuid, 6, CHAR_PROP_NOTIFY | CHAR_PROP_READ, ATTR_PERMISSION_NONE,  GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP, 16, 0, &accelero_read);
        if (result != BLE_STATUS_SUCCESS){
            DEBUG_PRINTF("Error while adding Accelero service.\n");
        }
    	DEBUG_PRINTF("Bien lancé");

}
    return result;
}
void Ble_Service_Inclination_Read(uint32_t data)
{
  uint32_t val =data ;
  tBleStatus result;

  result = aci_gatt_update_char_value(motion_handle,inclination_read, 0, 1, &val);

  if (result != BLE_STATUS_SUCCESS){
      DEBUG_PRINTF("Error sending read LED ReadWrite Char.\n");
  }
}


void Ble_Service_Temp_Read(void)
{
  uint8_t val = Led_Get_Status();
  tBleStatus result;

  result = aci_gatt_update_char_value(envi_sensor_serv_handle,temperature_read, 0, 1, &val);

  if (result != BLE_STATUS_SUCCESS){
      DEBUG_PRINTF("Error sending read LED ReadWrite Char.\n");
  }
}

void Ble_Service_Humidity_Read(void)
{
  uint8_t val = Led_Get_Status();
  tBleStatus result;

  result = aci_gatt_update_char_value(envi_sensor_serv_handle, humidity_read, 0, 1, &val);

  if (result != BLE_STATUS_SUCCESS){
      DEBUG_PRINTF("Error sending read LED ReadWrite Char.\n");
  }
}

void Ble_Service_Pressure_Read(void)
{
  uint8_t val = Led_Get_Status();
  tBleStatus result;

  result = aci_gatt_update_char_value(envi_sensor_serv_handle, pressure_read, 0, 1, &val);

  if (result != BLE_STATUS_SUCCESS){
      DEBUG_PRINTF("Error sending read LED ReadWrite Char.\n");
  }
}


void Ble_Service_Led_Read(void)
{
  uint8_t val = Led_Get_Status();
  tBleStatus result;

  result = aci_gatt_update_char_value(led_serv_handle, led_readwrite_char_handle, 0, 1, &val);

  if (result != BLE_STATUS_SUCCESS){
      DEBUG_PRINTF("Error sending read LED ReadWrite Char.\n");
  }
}





void Ble_Service_Led_Write(uint8_t value)
{
  if (value == 0) {
      Led_Off();
  }
  else if (value == 1){
      Led_On();
  }
}

void Ble_Service_Led_Blink(uint16_t interval_ms, uint16_t duration_ms)
{
  Led_Blink(interval_ms, duration_ms);
}

/**
 * @brief  This function allows to add Ble services
 * @param  None
 * @retval None
 */


tBleStatus Ble_Service_Button_Init(void)			///On initialise de la même manière que la LED
{
    tBleStatus result;
    uint8_t uuid[16];

    COPY_BUTTON_SERVICEUUID(uuid);

    result = aci_gatt_add_serv(UUID_TYPE_128, uuid, PRIMARY_SERVICE, 7, &button_serv_handle);

    if (result == BLE_STATUS_SUCCESS) {
        // copy "LED button characteristic UUID" defined above to 'uuid' local variable
    	COPY_BUTTON_NBPRESSED(uuid);
        result =  aci_gatt_add_char(button_serv_handle, UUID_TYPE_128, uuid, 2, CHAR_PROP_NOTIFY | CHAR_PROP_READ, ATTR_PERMISSION_NONE, GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP, 16, 0, &button_nbpressed);
        if (result != BLE_STATUS_SUCCESS){
            DEBUG_PRINTF("Error while adding LED service.\n");
        }
    	DEBUG_PRINTF("Bien lancé");

}
    return result;
}


tBleStatus Ble_Init_Service(void)
{
  tBleStatus result;

  result = Ble_Service_Led_Init();
  result = Ble_Service_Button_Init();
  result = Ble_Service_Accelero_Init();
  result =  Ble_Service_Envi_sensors_Init();
  result = Ble_Service_Inclination_Init();
  return result;
}


void Ble_Service_Button_Read(void)
{
  uint16_t val = Button_Get_nbpressed();																///On récupère la valeur
  tBleStatus result;
  result = aci_gatt_update_char_value(button_serv_handle, button_nbpressed, 0, 1, &val);   ///On l'envoie

  if (result != BLE_STATUS_SUCCESS){
      DEBUG_PRINTF("Error sending read button ReadWrite Char.\n");
  }
}



void Ble_Service_Accelero_Read(void)
{
  uint64_t val = Mems_Get_Accelero();																///On récupère la valeur
  tBleStatus result;
  result = aci_gatt_update_char_value(accelero_serv_handle, accelero_read ,0, 1, &val);   ///On l'envoie

  if (result != BLE_STATUS_SUCCESS){
      DEBUG_PRINTF("Error sending read accelero ReadWrite Char.\n");
  }
}
/**
 * @brief  This function is called when a GATT Client wants to read an attribute.
 * @param  handle : the attribute handler
 * @retval None
 */
void Ble_Read_Request_Callback(uint16_t handle)
{
  if (handle == led_readwrite_char_handle + 1) {
      Ble_Service_Led_Read();
  }
  if (handle == button_nbpressed + 1) {
       Ble_Service_Button_Read();
   }
  if (handle == accelero_read + 1) {
	  Ble_Service_Accelero_Read();
     }
  if (handle == temperature_read+ 1) {
	  Ble_Service_Temp_Read();
       }
  if (handle == humidity_read+ 1) {
 	  Ble_Service_Humidity_Read();
        }
  if (handle == pressure_read+ 1) {
 	  Ble_Service_Pressure_Read();
        }
}


/**
 * @brief  This function is called when a GATT Client wants to write an attribute.
 * @param  handle : the attribute handler
 * @param  data : ptr to the data to write
 * @param  length : the length of the data
 * @retval None
 */
void Ble_Attribute_Modified_CallBack(uint16_t handle, uint8_t *data, uint16_t length)
{
  if (handle == led_readwrite_char_handle + 1 && length == 1) {
      Ble_Service_Led_Write(*(uint8_t*)data);
  }
  else if (handle == led_blink_char_handle + 1 && length == 4) {
      uint16_t interval = ((uint8_t)data[3] << 8) + (uint8_t)data[2];
      uint16_t duration = ((uint8_t)data[1] << 8) + (uint8_t)data[0];

      Ble_Service_Led_Blink(interval, duration);
  }
}
