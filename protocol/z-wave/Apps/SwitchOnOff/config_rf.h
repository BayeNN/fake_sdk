/**
 * @file
 * @brief RF Configuration file for Switch On/Off sample application.
 * @details This file contains RF definitions for the Z-Wave sample app.
 * @copyright 2019 Silicon Laboratories Inc.
 */
#ifndef _CONFIG_RF_H_
#define _CONFIG_RF_H_

// The maximum allowed Tx power in deci dBm
#define APP_MAX_TX_POWER      0

// The deci dBm output measured at a PA setting of 0dBm (raw value 24)
#define APP_MEASURED_0DBM_TX_POWER 33

// The maximum allowed transmit power for LR given in deci dBm.
#ifndef APP_MAX_TX_POWER_LR
#define APP_MAX_TX_POWER_LR 140
#endif

#define ENABLE_PTI 0

#endif /* _CONFIG_RF_H_ */
