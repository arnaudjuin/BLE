/**
 ******************************************************************************
 * @file    Source/App/Ble/ble_common.h
 * @author  Arthur Burnichon
 * @date    20-Oct-2017
 * @brief   Ble common component definition file
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
#ifndef BLE_COMMON_H_
#define BLE_COMMON_H_

/* Includes ------------------------------------------------------------------*/
#include "hal_conf.h"

#include "ble_status.h"

/* Exported define -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

tBleStatus Ble_Init(uint8_t *mac_addr, const char *device_name);
tBleStatus Ble_Init_Service(void);
void Ble_Set_Discoverable(const char *broadcast_name, uint32_t timeout_ms);
uint8_t Ble_Is_Discoverable(void);
void Ble_Set_Non_Discoverable(void);
void Ble_Connection_Callback();
void Ble_Read_Request_Callback(uint16_t handle);
void Ble_Attribute_Modified_CallBack(uint16_t handle, uint8_t *data, uint16_t length);
void Ble_Disconnection_Callback();
void Ble_Process(void);

#endif // BLE_COMMON_H_
