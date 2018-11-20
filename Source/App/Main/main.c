/**
 ******************************************************************************
 * @file    Source/App/Main/main.c
 * @author  Arthur Burnichon
 * @date    20-Oct-2017
 * @brief   Main program body
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
#include "stm32_debug.h"

#include "Ble/ble_common.h"
#include "Ble/ble_service.h"
#include "Button/button.h"
#include "Led/led.h"
#include "Mems/mems.h"
#include "Motion/motion.h"
#include "hal_conf.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t mac_addr[] = { 0xCC, 0x45, 0x4C, 0x45, 0x00, 0x04 };
char *device_name = "team04";
char *broadcast_name = "PP_team04";

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main function who sequence the initialization of Hardware peripheries
 *         and start the processing loop of the application.
 *
 * @param  None
 * @retval Integer
 */
int main(void) {
	// Configure the HAL Library
	HAL_Init();

	// Initialize the debug output
	DEBUG_INIT();

	// Configure the system clock
	SystemClock_Config();

	// Initialize Button component
	Button_Init();

	// Initialize Led component
	Led_Init();

	// Initialize Mems component
	Mems_Init();
	Mems_Accelero_Enable();

	// Initialize Motion component
	Motion_Init();

	// Initialize the Ble component
	Ble_Init(mac_addr, device_name);
	Ble_Set_Discoverable(broadcast_name, 120000);


  while(1)
    {
      Button_Process();
      Led_Process();
      Mems_Process();
      Motion_Process();
      Ble_Process();
    }
}

/**
 * @note Allows to notify user if notification was enable
 *       on the Led ReadWrite characteristic
 */
void Led_Status_Change_Callback() {
	Ble_Service_Led_Read();
}
void Button_Status_Change_Callback() {
	Ble_Service_Button_Read();					///lE CALLBACK ACTUALISE LA VALEUR DU BLE READ
}
void Accelero_Status_Change_Callback() {
	Ble_Service_Accelero_Read();					///lE CALLBACK ACTUALISE LA VALEUR DU BLE READ
}
/**
 * @note Allows to re-enable discoverable (undirect connection) mode
 *
 */
void Button_Long_Pressed_Callback() {
	Led_Blink(200, 1800);
	Ble_Set_Discoverable(broadcast_name, 120000);						///SI appui long on rend le periphérique découvrable
}
void Button_Pressed_Released_Callback(uint8_t press_release_count)
{
	Button_Status_Change_Callback();											///Le service BLE releve la nouvelle valeur a chaque appui

}







