/**
 ******************************************************************************
 * @file    Source/App/Mems/mems.c
 * @author  Arthur Burnichon
 * @date    20-Oct-2017
 * @brief   Mems component implementation
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
#include "mems.h"

#include <math.h>   /* trunc */

#include "gp_timer.h"
#include "stm32_debug.h"

#ifdef USE_IKS01A1_NUCLEO
#include "x_nucleo_iks01a1.h"
#include "x_nucleo_iks01a1_accelero.h"
#include "x_nucleo_iks01a1_gyro.h"
#include "x_nucleo_iks01a1_magneto.h"
#include "x_nucleo_iks01a1_pressure.h"
#include "x_nucleo_iks01a1_humidity.h"
#include "x_nucleo_iks01a1_temperature.h"
#elif defined(USE_IKS01A2_NUCLEO)
#include "x_nucleo_iks01a2.h"
#include "x_nucleo_iks01a2_accelero.h"
#include "x_nucleo_iks01a2_gyro.h"
#include "x_nucleo_iks01a2_magneto.h"
#include "x_nucleo_iks01a2_pressure.h"
#include "x_nucleo_iks01a2_humidity.h"
#include "x_nucleo_iks01a2_temperature.h"
#else
#error Please select the Mems expansion board used
#endif

/* Private typedef -----------------------------------------------------------*/
typedef struct displayFloatToInt_s {
  int8_t sign; /* 0 means positive, 1 means negative*/
  uint32_t  out_int;
  uint32_t  out_dec;
} displayFloatToInt_t;

/* Private define ------------------------------------------------------------*/
#define MAX_BUF_SIZE 256

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct timer mems_read_timer;
static void *ACCELERO_handle = NULL;
static void *GYRO_handle = NULL;
static void *MAGNETO_handle = NULL;
static void *HUMIDITY_handle = NULL;
static void *TEMP_handle = NULL;
static void *PRESSURE_handle = NULL;
static void *PRESSURE_TEMP_handle = NULL;

/* Private function prototypes -----------------------------------------------*/
static void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec);

static void Mems_Accelero_Handler(void *handle);
static void Mems_Gyro_Handler(void *handle);
static void Mems_Magneto_Handler(void *handle);
static void Mems_Humidity_Handler(void *handle);
static void Mems_Temperature_Handler(void *handle);
static void Mems_Pressure_Handler(void *handle);

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Splits a float into two integer values.
 * @param  in the float value as input
 * @param  out_value the pointer to the output integer structure
 * @param  dec_prec the decimal precision to be used
 * @retval None
 */
static void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec)
{
  if(in >= 0.0f)
    {
      out_value->sign = 0;
    }else
      {
        out_value->sign = 1;
        in = -in;
      }

  out_value->out_int = (int32_t)in;
  in = in - (float)(out_value->out_int);
  out_value->out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}

/**
 * @brief  Handles the accelerometer axes data getting/sending
 * @param  handle : Accelerometer handle
 * @retval None
 */
static void Mems_Accelero_Handler(void *handle)
{
  SensorAxes_t acceleration;
  uint8_t status;

  BSP_ACCELERO_IsInitialized(handle, &status);

  if (status == 1)
    {
      if (BSP_ACCELERO_Get_Axes(handle, &acceleration) == COMPONENT_ERROR)
        {
          acceleration.AXIS_X = 0;
          acceleration.AXIS_Y = 0;
          acceleration.AXIS_Z = 0;
        }

      Mems_Accelero_Callback(acceleration.AXIS_X, acceleration.AXIS_Y, acceleration.AXIS_Z);
    }
}

/**
 * @brief  Handles the gyroscope axes data getting/sending
 * @param  handle : Gyroscope handle
 * @retval None
 */
static void Mems_Gyro_Handler(void *handle)
{
  SensorAxes_t angular_velocity;
  uint8_t status;

  BSP_GYRO_IsInitialized(handle, &status);

  if (status == 1)
    {
      if (BSP_GYRO_Get_Axes(handle, &angular_velocity) == COMPONENT_ERROR)
        {
          angular_velocity.AXIS_X = 0;
          angular_velocity.AXIS_Y = 0;
          angular_velocity.AXIS_Z = 0;
        }

      Mems_Gyro_Callback(angular_velocity.AXIS_X, angular_velocity.AXIS_Y, angular_velocity.AXIS_Z);
    }
}

/**
 * @brief  Handles the magnetometer axes data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static void Mems_Magneto_Handler(void *handle)
{
  SensorAxes_t magnetic_field;
  uint8_t status;

  BSP_MAGNETO_IsInitialized(handle, &status);

  if (status == 1)
    {
      if (BSP_MAGNETO_Get_Axes(handle, &magnetic_field) == COMPONENT_ERROR)
        {
          magnetic_field.AXIS_X = 0;
          magnetic_field.AXIS_Y = 0;
          magnetic_field.AXIS_Z = 0;
        }

      Mems_Gyro_Callback(magnetic_field.AXIS_X, magnetic_field.AXIS_Y, magnetic_field.AXIS_Z);
    }
}

/**
 * @brief  Handles the humidity sensor data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static void Mems_Humidity_Handler(void *handle)
{
  float humidity;
  uint8_t status;

  BSP_HUMIDITY_IsInitialized(handle, &status);

  if (status == 1)
    {
      if (BSP_HUMIDITY_Get_Hum(handle, &humidity) == COMPONENT_ERROR)
        {
          humidity = 0.0f;
        }

      Mems_Humidity_Callback(humidity);
    }
}

/**
 * @brief  Handles the temperature data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static void Mems_Temperature_Handler(void *handle)
{
  float temperature;
  uint8_t status;

  BSP_TEMPERATURE_IsInitialized(handle, &status);

  if (status == 1)
    {
      if (BSP_TEMPERATURE_Get_Temp(handle, &temperature) == COMPONENT_ERROR)
        {
          temperature = 0.0f;
        }

      Mems_Temperature_Callback(temperature);
    }
}

/**
 * @brief  Handles the pressure sensor data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static void Mems_Pressure_Handler(void *handle)
{
  float pressure;
  uint8_t status;

  BSP_PRESSURE_IsInitialized(handle, &status);

  if (status == 1)
    {
      if (BSP_PRESSURE_Get_Press(handle, &pressure) == COMPONENT_ERROR)
        {
          pressure = 0.0f;
        }

      Mems_Pressure_Callback(pressure);
    }
}

/* Public functions ----------------------------------------------------------*/
/**
 * @brief  Initialize MEMS sensors
 * @param  None
 * @retval None
 */
void Mems_Init(void)
{
  BSP_ACCELERO_Init(ACCELERO_SENSORS_AUTO, &ACCELERO_handle);
  BSP_GYRO_Init(GYRO_SENSORS_AUTO, &GYRO_handle);
  BSP_MAGNETO_Init(MAGNETO_SENSORS_AUTO, &MAGNETO_handle );
  BSP_HUMIDITY_Init(HUMIDITY_SENSORS_AUTO, &HUMIDITY_handle);
  // Init HTS221 Temperature sensor unit
  BSP_TEMPERATURE_Init(TEMPERATURE_SENSORS_AUTO, &TEMP_handle );
  // Init LPS25H/B Temperature sensor unit
  BSP_TEMPERATURE_Init(TEMPERATURE_SENSORS_AUTO, &PRESSURE_TEMP_handle );
  BSP_PRESSURE_Init(PRESSURE_SENSORS_AUTO, &PRESSURE_handle );
  Timer_Set(&mems_read_timer,500);			///Modification du timer de 100ms à 500ms
}

/**
 * @brief  Enable accelerometer sensor
 * @param  None
 * @retval None
 */
void Mems_Accelero_Enable(void)
{
  BSP_ACCELERO_Sensor_Enable(ACCELERO_handle);
}

/**
 * @brief  Enable gyroscope sensor
 * @param  None
 * @retval None
 */
void Mems_Gyro_Enable(void)
{
  BSP_GYRO_Sensor_Enable(GYRO_handle);
}

/**
 * @brief  Enable magnetometer sensor
 * @param  None
 * @retval None
 */
void Mems_Magneto_Enable(void)
{
  BSP_MAGNETO_Sensor_Enable(MAGNETO_handle);
}

/**
 * @brief  Enable humidity sensor
 * @param  None
 * @retval None
 */
void Mems_Humidity_Enable(void)
{
  BSP_HUMIDITY_Sensor_Enable(HUMIDITY_handle);

}

/**
 * @brief  Enable temperature sensor
 * @param  None
 * @retval None
 */
void Mems_Temperature_Enable(void)
{
  BSP_TEMPERATURE_Sensor_Enable(TEMP_handle);
}

/**
 * @brief  Enable pressure sensor
 * @param  None
 * @retval None
 */
void Mems_Pressure_Enable(void)
{
  // Pressure sensor include a Temperature sensor for correction
  BSP_TEMPERATURE_Sensor_Enable(PRESSURE_TEMP_handle);
  BSP_PRESSURE_Sensor_Enable(PRESSURE_handle);
}

/**
 * @brief  De-initialize MEMS sensors
 * @param  None
 * @retval None
 */
void Mems_DeInit(void)
{
  BSP_ACCELERO_DeInit(&ACCELERO_handle);
  BSP_GYRO_DeInit(&GYRO_handle);
  BSP_MAGNETO_DeInit(&MAGNETO_handle );
  BSP_HUMIDITY_DeInit(&HUMIDITY_handle);
  // DeInit HTS221 Temperature sensor unit
  BSP_TEMPERATURE_DeInit(&TEMP_handle );
  // DeInit LPS25H/B Temperature sensor unit
  BSP_TEMPERATURE_DeInit(&PRESSURE_TEMP_handle );
  BSP_PRESSURE_DeInit(&PRESSURE_handle );
}

/**
 * @brief  Callback for accelerometer data
 *         should be re-implemented by user for application needs
 * @param  axis_x for Axis X data, axis_y for Axis Y data, axis_z for Axis Z data
 * @retval None
 */
__weak void Mems_Accelero_Callback(int32_t axis_x, int32_t axis_y, int32_t axis_z)
{

}

/**
 * @brief  Callback for gyroscope data
 *         should be re-implemented by user for application needs
 * @param  axis_x for Axis X data, axis_y for Axis Y data, axis_z for Axis Z data
 * @retval None
 */
__weak void Mems_Gyro_Callback(int32_t axis_x, int32_t axis_y, int32_t axis_z)
{
  //FIXME Can be re-implemented to meet users needs
}

/**
 * @brief  Callback for magnetometer data
 *         should be re-implemented by user for application needs
 * @param  axis_x for Axis X data, axis_y for Axis Y data, axis_z for Axis Z data
 * @retval None
 */
__weak void Mems_Magneto_Callback(int32_t axis_x, int32_t axis_y, int32_t axis_z)
{
  //FIXME Can be re-implemented to meet users needs
}

/**
 * @brief  Callback for humidity sensor data
 *         should be re-implemented by user for application needs
 * @param  value : Humidity value
 * @retval None
 */
__weak void Mems_Humidity_Callback(float value)
{
  //FIXME Can be re-implemented to meet users needs
  displayFloatToInt_t out_value;
  floatToInt(value, &out_value, 2 );
  DEBUG_PRINTF("HUM: %d.%02d\r\n", (int)out_value.out_int, (int)out_value.out_dec);
}

/**
 * @brief  Callback for temperature sensor data
 *         should be re-implemented by user for application needs
 * @param  value : Temperature value
 * @retval None
 */
__weak void Mems_Temperature_Callback(float value)
{
  //FIXME Can be re-implemented to meet users needs
  displayFloatToInt_t out_value;
  floatToInt(value, &out_value, 2 );
  DEBUG_PRINTF("TEMP: %c%d.%02d\r\n", ((out_value.sign) ? '-' : '+'), (int)out_value.out_int, (int)out_value.out_dec);
}

/**
 * @brief  Callback for pressure sensor data
 *         should be re-implemented by user for application needs
 * @param  value : Temperature value
 * @retval None
 */
__weak void Mems_Pressure_Callback(float value)
{
  //FIXME Can be re-implemented to meet users needs
  displayFloatToInt_t out_value;
  floatToInt(value, &out_value, 2 );
  DEBUG_PRINTF("PRES: %d.%02d\r\n", (int)out_value.out_int, (int)out_value.out_dec);
}

/**
 * @brief  Process all sensors data
 * @param  None
 * @retval None
 */
void Mems_Process(void) {
  uint8_t status;

  if (Timer_Expired(&mems_read_timer))
    {
      if (BSP_ACCELERO_IsEnabled(ACCELERO_handle, &status) == COMPONENT_OK && status == 1)
        {
          Mems_Accelero_Handler(ACCELERO_handle);
        }
      if (BSP_GYRO_IsEnabled(GYRO_handle, &status) == COMPONENT_OK && status == 1)
        {
          Mems_Gyro_Handler(GYRO_handle);
        }
      if (BSP_MAGNETO_IsEnabled(MAGNETO_handle, &status) == COMPONENT_OK && status == 1)
        {
          Mems_Magneto_Handler(MAGNETO_handle);
        }
      if (BSP_HUMIDITY_IsEnabled(HUMIDITY_handle, &status) == COMPONENT_OK && status == 1)
        {
          Mems_Humidity_Handler(HUMIDITY_handle);
        }
      if (BSP_TEMPERATURE_IsEnabled(TEMP_handle, &status) == COMPONENT_OK && status == 1)
        {
          Mems_Temperature_Handler(TEMP_handle);
        }
      if (BSP_PRESSURE_IsEnabled(PRESSURE_handle, &status) == COMPONENT_OK && status == 1)
        {
          Mems_Temperature_Handler(PRESSURE_TEMP_handle);
          Mems_Pressure_Handler(PRESSURE_handle);
        }
      Timer_Restart(&mems_read_timer);
    }
}
