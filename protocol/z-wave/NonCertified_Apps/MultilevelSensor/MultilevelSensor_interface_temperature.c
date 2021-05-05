/***************************************************************************//**
 * @file MultilevelSensor_interface_temperature.c
 * @brief MultilevelSensor_interface_temperature.c
 * @copyright 2020 Silicon Laboratories Inc.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

// -----------------------------------------------------------------------------
//                   Includes
// -----------------------------------------------------------------------------
#include <string.h>
#include <stddef.h>
#include <stdbool.h>
#include "MultilevelSensor_interface.h"
#include "MultilevelSensor_si7021.h"
// -----------------------------------------------------------------------------
//                Macros and Typedefs
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//              Static Function Declarations
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                Global Variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                Static Variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//              Public Function Definitions
// -----------------------------------------------------------------------------

bool MultilevelSensor_interface_temperature_init(void)
{
  return Sensor_si7021_init();
}

bool MultilevelSensor_interface_temperature_deinit(void)
{
  return true;
}

bool MultilevelSensor_interface_temperature_read(sl_sensor_read_result_t* o_result, uint8_t i_scale)
{
  bool retval = false;
  int32_t  temperature_celsius = 0;

  if(o_result != NULL)
  {
    memset(o_result, 0, sizeof(sl_sensor_read_result_t));

    if(true == Sensor_si7021_read_temperature(&temperature_celsius))
    {
      retval = true;
      o_result->precision  = SL_SENSOR_READ_RESULT_PRECISION_3;
      o_result->size_bytes = SL_SENSOR_READ_RESULT_SIZE_4;
      
      if(i_scale == SL_SENSOR_SCALE_FAHRENHEIT)
      {
        float temperature_celsius_divided = (float)temperature_celsius/(float)1000;
        int32_t temperature_fahrenheit = (int32_t)((temperature_celsius_divided * ((float)9/(float)5) + (float)32)*1000);

        o_result->raw_result[3] = (temperature_fahrenheit&0xFF);
        o_result->raw_result[2] = (temperature_fahrenheit>>8 )&0xFF;
        o_result->raw_result[1] = (temperature_fahrenheit>>16)&0xFF;
        o_result->raw_result[0] = (temperature_fahrenheit>>24)&0xFF;
      }
      else
      {
        o_result->raw_result[3] = (temperature_celsius&0xFF);
        o_result->raw_result[2] = (temperature_celsius>>8 )&0xFF;
        o_result->raw_result[1] = (temperature_celsius>>16)&0xFF;
        o_result->raw_result[0] = (temperature_celsius>>24)&0xFF;
      }
    }
  }
  return retval;
}

// -----------------------------------------------------------------------------
//              Static Function Definitions
// -----------------------------------------------------------------------------
