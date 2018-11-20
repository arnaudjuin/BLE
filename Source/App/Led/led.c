/**
 ******************************************************************************
 * @file    Source/App/Led/led.c
 * @author  Arthur Burnichon
 * @date    20-Oct-2017
 * @brief   Led component implementation
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
#include "led.h"

#include "gp_timer.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct timer led_blink_interval_timer;
struct timer led_blink_duration_timer;
uint16_t led_blink_interval_ms;
uint16_t led_blink_duration_ms;

uint8_t led_status;

/* Private function prototypes -----------------------------------------------*/
void Led_Set_Status(void);
void Led_Reset_Status(void);
void Led_Toggle(void);

/* Private functions ---------------------------------------------------------*/
void Led_Set_Status(void)
{
  led_status = 1;
  Led_Status_Change_Callback();
}

void Led_Reset_Status(void)
{
  led_status = 0;
  Led_Status_Change_Callback();
}

void Led_Toggle(void)
{
  if (led_status == 0) {
      Led_Set_Status();
  }
  else {
      Led_Reset_Status();
  }
	BSP_LED_Toggle(LED2);
}

/* Public functions ----------------------------------------------------------*/
void Led_Init(void)
{
  led_status = 0;
	led_blink_interval_ms = 0;
	BSP_LED_Init(LED2);
}

/**
 * @breif  Turn on the led
 * @retval None
 */
void Led_On(void)
{
  Led_Reset_Status();
	BSP_LED_On(LED2);
}

/**
 * @breif  Get the led status
 * @retval Return 0 if led is off
 *         Return 1 if led is on
 */
uint8_t Led_Get_Status(void)
{
  return led_status;
}

/**
 * @breif  Allows to notify user of led status change
 * @retval None
 */
__weak void Led_Status_Change_Callback()
{
  //FIXME Can be re-implemented to meet users needs
}

/**
 * @breif  Turn off the led
 * @retval None
 */
void Led_Off(void)
{
	led_blink_interval_ms = 0;
  Led_Reset_Status();
	BSP_LED_Off(LED2);
}

/**
 * @brief  Led blink initialization using a blinking duration
 *         and a led state duration. If the blinking duration
 *         is set to 0 the led blink never stop until call of
 *         Led_Off or Led_Blink function
 *
 * @param  interval_ms the led on/off duration
 * @param  duration_ms the duration of led blinking
 * @retval None
 */
void Led_Blink(uint16_t interval_ms, uint16_t duration_ms)
{
	if (interval_ms)
	{
		led_blink_interval_ms = interval_ms;
		led_blink_duration_ms = duration_ms;
		Timer_Set(&led_blink_interval_timer, interval_ms);
		Timer_Set(&led_blink_duration_timer, duration_ms);
	}
	else
	{
		Led_On();
	}
}

/**
 * @brief  Led processing function who should be call
 *         periodically by the main processing loop
 * @param  None
 * @retval None
 */
void Led_Process(void)
{
	if (Timer_Expired(&led_blink_interval_timer) && led_blink_interval_ms != 0)
	{
		if (!Timer_Expired(&led_blink_duration_timer) || led_blink_duration_ms == 0)
		{
			Led_Toggle();
			Timer_Reset(&led_blink_interval_timer);
		}
		// Check if the blink interval is a multiple of the blink duration to ensure
		// timer duration doesn't expire before the last led toggle
		else if (Timer_Expired(&led_blink_duration_timer) && (led_blink_duration_ms%led_blink_interval_ms == 0))
		{
			Led_Toggle();
			led_blink_interval_ms = 0;
		}
	}
}
