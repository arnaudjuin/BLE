/**
 ******************************************************************************
 * @file    Source/App/Button/button.h
 * @author  Arthur Burnichon
 * @date    20-Oct-2017
 * @brief   Button component definition file
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
#ifndef BUTTON_BUTTON_H_
#define BUTTON_BUTTON_H_

/* Includes ------------------------------------------------------------------*/
#include "hal_conf.h"

/* Exported define -----------------------------------------------------------*/
#define MAX_PRESS_NUMBER 3
#define DEBOUNCE_PERIOD 10
#define PRESS_PERIOD 500
#define LONG_PRESS_PERIOD 3000

/* Exported types ------------------------------------------------------------*/
typedef enum  {
    buttonReleased = 0,
    buttonPressed,
    buttonPressedReleased,
    buttonLongPressed,
} button_event_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Button_Init();
button_event_t Button_Get_Last_Event(void);
void Button_Pressed_Callback(void);
void Button_Long_Pressed_Released_Callback(void);
void Button_Pressed_Released_Callback(uint8_t press_release_count);
void Button_Long_Pressed_Callback(void);
void Button_Process(void);
uint16_t Button_Get_nbpressed(void);
#endif // BUTTON_BUTTON_H_
