/**
 ******************************************************************************
 * @file    Source/App/Mems/mems.h
 * @author  Arthur Burnichon
 * @date    20-Oct-2017
 * @brief   Mems component definition file
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
#ifndef MEMS_MEMS_H_
#define MEMS_MEMS_H_

/* Includes ------------------------------------------------------------------*/
#include "hal_conf.h"
/* Exported define -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Mems_Init(void);
void Mems_DeInit(void);
void Mems_Accelero_Enable(void);
void Mems_Gyro_Enable(void);
void Mems_Magneto_Enable(void);
void Mems_Humidity_Enable(void);
void Mems_Temperature_Enable(void);
void Mems_Pressure_Enable(void);
void Mems_Accelero_Callback(int32_t axis_x, int32_t axis_y, int32_t axis_z);
void Mems_Gyro_Callback(int32_t axis_x, int32_t axis_y, int32_t axis_z);
void Mems_Magneto_Callback(int32_t axis_x, int32_t axis_y, int32_t axis_z);
void Mems_Humidity_Callback(float value);
void Mems_Temperature_Callback(float value);
void Mems_Pressure_Callback(float value);
void Mems_Process(void);

#endif // MEMS_MEMS_H_
