/**
 ******************************************************************************
 * @file    Source/App/Button/button.c
 * @author  Arthur Burnichon
 * @date    20-Oct-2017
 * @brief   Button component implementation file
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
#include "button.h"

#include "stm32_debug.h"

#include "gp_timer.h"
/* Private define ------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct timer button_timer;
button_event_t button_last_state;
uint8_t button_press_count;
uint16_t NB_COUNT;							///SUR 2 OCTETS

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
/**
  * @brief  Button initialization function
  * @param  None
  * @retval None
  */
void Button_Init()
{
      BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
      Timer_Set(&button_timer, LONG_PRESS_PERIOD);
      button_last_state = buttonReleased;
      button_press_count = 0;
      NB_COUNT=0;
}

/**
  * @brief  Get button last event detected
  * @param  None
  * @retval Last event of the button
  */
button_event_t Button_Get_Last_Event(void)
{
  return button_last_state;
}

/**
  * @brief  Callback for button pressed detection
  *         should be re-implemented by user for application needs
  * @param  None
  * @retval None
  */
__weak void Button_Pressed_Callback(void)
{
  //FIXME Can be re-implemented to meet users needs
	 ;

}

/**
  * @brief  Callback for button released detection
  *         should be re-implemented by user for application needs
  * @param  None
  * @retval None
  */
__weak void Button_Released_Callback(void)
{
  //FIXME Can be re-implemented to meet users needs

}

/**
  * @brief  Callback for button pressed released detection
  *         should be re-implemented by user for application needs
  * @param  press_release_count allows to detect simple, double or
  *         multiple click from button
  * @retval None
  */
__weak void Button_Pressed_Released_Callback(uint8_t press_release_count)
{
  //FIXME Can be re-implemented to meet users needs
  DEBUG_PRINTF("Button Pressed & Released %d times\n", press_release_count);
}



/**
  * @brief  Callback for button long pressed detection
  *         should be re-implemented by user for application needs
  * @param  None
  * @retval None
  */
__weak void Button_Long_Pressed_Callback(void)
{
  //FIXME Can be re-implemented to meet users needs
}

/**
  * @brief  Processing function to detect button event
  * @param  None
  * @retval None
  */

uint16_t Button_Get_nbpressed(void)
{
	NB_COUNT++;
	return  NB_COUNT;
}

__weak void Button_Status_Change_Callback(void) {
	//
}


void Button_Process(void)
{
  uint32_t button_state = BSP_PB_GetState(BUTTON_KEY);

  switch (button_last_state) {
    case buttonPressed:
      if (button_state == SET) {
          if (Timer_Time(&button_timer) <= DEBOUNCE_PERIOD) {
              button_last_state = buttonReleased;
              button_press_count = 0;
          }
          else if (Timer_Time(&button_timer) > DEBOUNCE_PERIOD) {
              button_last_state = buttonPressedReleased;
          }
      }
      else {
          if (Timer_Expired(&button_timer)) {
              button_last_state = buttonLongPressed;
              Button_Long_Pressed_Callback();
          }
          else {
              Button_Pressed_Callback();
          }
      }
      break;
    case buttonPressedReleased:
      if (button_state == SET && Timer_Time(&button_timer) > PRESS_PERIOD) {
          Button_Pressed_Released_Callback(button_press_count);
          button_last_state = buttonReleased;
          button_press_count = 0;
      }
      else if (button_state == RESET && Timer_Time(&button_timer) <= PRESS_PERIOD) {
          button_last_state = buttonPressed;
          if(button_press_count < MAX_PRESS_NUMBER) {
              ++button_press_count;
          }
      }
      break;
    case buttonLongPressed:
      if (button_state == SET) {
          Button_Released_Callback();
          button_last_state = buttonReleased;
          button_press_count = 0;
      }
      break;
    case buttonReleased:
    default:
      if (button_state == RESET) {
          button_last_state = buttonPressed;
          button_press_count = 1;
          Timer_Restart(&button_timer);
      }
      break;
  }
}
