/**
 ******************************************************************************
 * @file    Source/App/Ble/ble_common.c
 * @author  Arthur Burnichon
 * @date    20-Oct-2017
 * @brief   Ble common component implementation
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
#include "ble_common.h"

#include <stdlib.h>
#include <string.h>

#include "stm32_bluenrg_ble.h"
#include "bluenrg_utils.h"
#include "bluenrg_gap.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_aci_const.h"
#include "hci_const.h"
#include "hci_le.h"
#include "hci.h"
#include "osal.h"
#include "sm.h"
#include "gp_timer.h"
#include "stm32_debug.h"

#include "Led/led.h"

/* Private define ------------------------------------------------------------*/
#define MACADDR_SIZE 6 // MAC Address size

#define IDB04A1 0
#define IDB05A1 1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t bnrg_expansion_board = IDB04A1; // at startup, suppose the X-NUCLEO-IDB04A1 is used

struct timer device_discoverable_timer;
volatile uint8_t device_discoverable = 0;

volatile uint16_t central_handle = 0;
volatile uint8_t central_addr[MACADDR_SIZE] = {0};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @brief  This function Initialize the Ble component.
 * @param  mac_addr : The 6 bytes mac address of the Ble component
 * @param  device_name : The name of the Ble device characteristic. The max
 *         lenght of this name is 8 bytes.
 * @retval None
 */
tBleStatus Ble_Init(uint8_t *mac_addr, const char *device_name)
{
  uint8_t bdaddr[MACADDR_SIZE];
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;

  uint8_t  hwVersion;
  uint16_t fwVersion;

  tBleStatus result;

  /* Initialize the BlueNRG SPI driver */
  BNRG_SPI_Init();

  /* Initialize the BlueNRG HCI */
  HCI_Init();

  /* Reset BlueNRG hardware */
  BlueNRG_RST();

  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);

  /*
   * Reset BlueNRG again otherwise we won't
   * be able to change its MAC address.
   * aci_hal_write_config_data() must be the first
   * command after reset otherwise it will fail.
   */
  BlueNRG_RST();

  DEBUG_PRINTF("HWver %d, FWver %d\n", hwVersion, fwVersion);

  if (hwVersion > 0x30) { /* X-NUCLEO-IDB05A1 expansion board is used */
      bnrg_expansion_board = IDB05A1;
      /*
       * Change the MAC address to avoid issues with Android cache:
       * if different boards have the same MAC address, Android
       * applications unless you restart Bluetooth on tablet/phone
       */
      mac_addr[5] += 1;
  }

  Osal_MemCpy(bdaddr, mac_addr, MACADDR_SIZE);

  result = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
  if(result){
      DEBUG_PRINTF("Setting BD_ADDR failed.\n");
  }

  result = aci_gatt_init();
  if(result){
      DEBUG_PRINTF("GATT_Init failed.\n");
  }

  if (bnrg_expansion_board == IDB05A1) {
      result = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }
  else {
      result = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }

  if(result != BLE_STATUS_SUCCESS){
      DEBUG_PRINTF("GAP_Init failed.\n");
  }

  result = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, strlen(device_name), (uint8_t *)device_name);

  if(result){
      DEBUG_PRINTF("Program stop : aci_gatt_update_char_value failed.\n");
      while(1);
  }

  result = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED, OOB_AUTH_DATA_ABSENT, NULL, 7, 16, USE_FIXED_PIN_FOR_PAIRING, 123456, BONDING);
  if (result == BLE_STATUS_SUCCESS) {
      DEBUG_PRINTF("BLE Stack Initialized.\n");
  }

  result = Ble_Init_Service();

  // Set output power level
  result = aci_hal_set_tx_power_level(1, 4);

  return result;
}

/**
 * @brief  This function allows to add Ble services
 * @param  None
 * @retval None
 */
__weak tBleStatus Ble_Init_Service(void)
{
  //FIXME Shall be re-implemented in ble_service.c to meet users needs
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Set the BLE device in discoverable mode.
 * @param  broadcast_name : the name who shall be broadcast by the BLE
 * @param  timeout_second : the duration of the discoverable mode in second.
 *         if set to 0 the device stay always discoverable
 * @retval None
 */
void Ble_Set_Discoverable(const char *broadcast_name, uint32_t timeout_ms)
{
  tBleStatus result;

  uint32_t broadcast_name_len = strlen(broadcast_name);
  const char name_ad_type[] = {AD_TYPE_COMPLETE_LOCAL_NAME, 0x00};
  char *local_name = (char *)malloc((broadcast_name_len + 1) * sizeof(char));

  // To prevent error from calling set discoverable when the device is already discoverable
  if (device_discoverable == 0) {
      // Creation of the local name by prepending the advertising type of Local name
      // (cf. aci_gap_set_discoverable)
      strcpy(local_name, name_ad_type);
      strcat(local_name, broadcast_name);

      // Disable scan response
      hci_le_set_scan_resp_data(0,NULL);

      result = aci_gap_set_discoverable(ADV_IND, 80, 120, PUBLIC_ADDR, NO_WHITE_LIST_USE, broadcast_name_len + 1, local_name, 0, NULL, 80, 240);
      if (result != BLE_STATUS_SUCCESS) {
          DEBUG_PRINTF("Error while setting discoverable mode (%d)\n", result);
      }
      else {
          device_discoverable = 1;
          Timer_Set(&device_discoverable_timer, timeout_ms);
      }
  }

  free(local_name);
}

/**
 * @brief  Get the BLE device discoverable (undirect connection) mode status.
 * @param  None
 * @retval Return 0 if device is not discoverable (undirect connection enable).
 *         Return 1 if device is discoverable (undirect connection disable).
 */
uint8_t Ble_Is_Discoverable(void)
{
  if (device_discoverable == 0) {
      return 0;
  }
  else {
      return 1;
  }
}

/**
 * @brief  Set the BLE device in non discoverable (undirect connection) mode.
 * @param  broadcast_name : the name who shall be broadcast by the BLE
 * @param  timeout_second : the duration of the discoverable (undirect connection) mode in second.
 *         if set to 0 the device stay always in the discoverable (undirect connection) mode
 * @retval None
 */
void Ble_Set_Non_Discoverable(void)
{
  tBleStatus result;

  result = aci_gap_set_non_discoverable();

  if (result != BLE_STATUS_SUCCESS) {
      DEBUG_PRINTF("Error while setting Non discoverable mode (%d)\n", result);
  }
  else {
      device_discoverable = 0;
  }
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  uint8_t Address of peer device
 * @param  uint16_t Connection handle
 * @retval None
 */
__weak void Ble_Connection_Callback()
{
  //FIXME Can be re-implemented to meet users needs
}

/**
 * @brief  This function is called when a GATT Client wants to read an attribute.
 * @param  handle : the attribute handler
 * @retval None
 */
__weak void Ble_Read_Request_Callback(uint16_t handle)
{
  //FIXME Shall be re-implemented in ble_service.c to meet users needs
}

/**
 * @brief  This function is called when a GATT Client wants to write an attribute.
 * @param  handle : the attribute handler
 * @param  data : ptr to the data to write
 * @param  length : the length of the data
 * @retval None
 */
__weak void Ble_Attribute_Modified_CallBack(uint16_t handle, uint8_t *data, uint16_t length)
{
  //FIXME Shall be re-implemented in ble_service.c to meet users needs
}

/**
 * @brief  This function is called when the peer device gets disconnected.
 * @param  None
 * @retval None
 */
__weak void Ble_Disconnection_Callback(void)
{
  //FIXME Can be re-implemented to meet users needs
}

/**
 * @brief  This function process the Ble component.
 * @param  None
 * @retval None
 */
void Ble_Process(void)
{
  HCI_Process();

  if (Ble_Is_Discoverable() && Timer_Expired(&device_discoverable_timer) && central_handle == 0) {
     Ble_Set_Non_Discoverable();
  }
}

/**
 * @brief  Callback processing the ACI events.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  void* Pointer to the ACI packet
 * @retval None
 */
void HCI_Event_CB(void *pckt)
{
  hci_uart_pckt *hci_pckt = pckt;
  hci_event_pckt *event_pckt = (hci_event_pckt*) hci_pckt->data;

  if (hci_pckt->type != HCI_EVENT_PKT)
    return;

  switch (event_pckt->evt)
  {

    case EVT_DISCONN_COMPLETE:
      {
        central_handle = 0;
        Ble_Disconnection_Callback();
      }
      break;

    case EVT_LE_META_EVENT:
      {
        evt_le_meta_event *evt = (void *) event_pckt->data;

        switch (evt->subevent)
        {
          case EVT_LE_CONN_COMPLETE:
            {
              evt_le_connection_complete *cc = (void *) evt->data;
              central_handle = cc->handle;
              memcpy(central_addr, cc->peer_bdaddr, sizeof(cc->peer_bdaddr));
              device_discoverable = 0;
              Ble_Connection_Callback();
            }
            break;
        }
      }
      break;

    case EVT_VENDOR:
      {
        evt_blue_aci *blue_evt = (void*) event_pckt->data;
        switch (blue_evt->ecode)
        {
          case EVT_BLUE_GATT_READ_PERMIT_REQ:
            {
              evt_gatt_read_permit_req *pr = (void*) blue_evt->data;
              Ble_Read_Request_Callback(pr->attr_handle);
              if (central_handle != 0) {
                  aci_gatt_allow_read(central_handle);
              }
            }
            break;
          case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
            {
              /* this callback is invoked when a GATT attribute is modified
                 extract callback data and pass to suitable handler function */
              if (bnrg_expansion_board == IDB05A1) {
                  evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
                  Ble_Attribute_Modified_CallBack(evt->attr_handle, evt->att_data, evt->data_length);
              }
              else {
                  evt_gatt_attr_modified_IDB04A1 *evt = (evt_gatt_attr_modified_IDB04A1*)blue_evt->data;
                  Ble_Attribute_Modified_CallBack(evt->attr_handle, evt->att_data, evt->data_length);
              }
            }
            break;
        }
      }
      break;
  }
}

