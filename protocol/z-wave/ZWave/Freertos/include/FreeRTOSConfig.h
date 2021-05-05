#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "FreeRTOSConfigTasks.h"  // Parameters from this task configuration file is used to configure the FreeRTOS!

void enterPowerDown(uint32_t millis);
void exitPowerDown(uint32_t millis);

#define configPRE_SLEEP_PROCESSING enterPowerDown
#define configPOST_SLEEP_PROCESSING exitPowerDown

#ifdef EFR32ZG23
#include "FreeRTOSConfig_series2.h"
#else
#include "FreeRTOSConfigTarget.h" // Include target specific configuration
#include "FreeRTOSConfig_series1.h"
#endif

/*
 * Converts a time in milliseconds to a time in ticks.
 * This macro overrides a macro of the same name defined in projdefs.h that gets
 * uint32_t overflow in case xTimeInMs > 4.2950e6  ~1.2 hours.
 */
#define pdMS_TO_TICKS( xTimeInMs ) \
      ( ( TickType_t ) ( ( ( uint64_t ) ( xTimeInMs ) * ( TickType_t ) configTICK_RATE_HZ ) / ( TickType_t ) 1000 ) )

#endif /* FREERTOS_CONFIG_H */
