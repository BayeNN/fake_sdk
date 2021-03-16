/***************************************************************************//**
 * @file
 * @brief Core Wi-Fi Commissioning application logic.
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
#include <stdio.h>
#include "os.h"
#include "io.h"
#include "bsp_os.h"
#include "common.h"
#include "em_common.h"
#include "sl_wfx_task.h"
#include "sl_wfx_host.h"
#include "sl_wfx_host_pinout.h"
#include "sl_simple_led_instances.h"
#include "sl_simple_button_instances.h"
#include "sl_simple_button_config.h"
#include "gpiointerrupt.h"
#include "app_webpage.h"
#include "app_wifi_events.h"
#include "app_wifi_commissioning.h"

#ifdef SL_CATALOG_POWER_MANAGER_PRESENT
#include "sl_power_manager.h"
#endif

#define START_TASK_PRIO              30u
#define START_TASK_STK_SIZE         600u

#ifdef SL_CATALOG_WFX_SECURE_LINK_PRESENT
extern void wfx_securelink_task_start(void);
#endif

extern uint8_t wirq_irq_nb;

/// Start task stack.
static CPU_STK start_task_stk[START_TASK_STK_SIZE];
/// Start task TCB.
static OS_TCB  start_task_tcb;
static void    start_task(void *p_arg);

static void wfx_interrupt(uint8_t intNo)
{
  RTOS_ERR err;

  (void)intNo;

  OSSemPost(&wfx_wakeup_sem, OS_OPT_POST_ALL, &err);
#ifdef SL_CATALOG_WFX_BUS_SPI_PRESENT
  OSFlagPost(&wfx_bus_evts, SL_WFX_BUS_EVENT_FLAG_RX, OS_OPT_POST_FLAG_SET, &err);
#endif
#ifdef SL_CATALOG_WFX_BUS_SDIO_PRESENT
#ifdef SLEEP_ENABLED
  OSFlagPost(&wfx_bus_evts, SL_WFX_BUS_EVENT_FLAG_RX, OS_OPT_POST_FLAG_SET, &err);
#endif
#endif
}

void sl_button_on_change(const sl_button_t *handle)
{
  if ((handle == &sl_button_btn0)
      && (sl_button_get_state(&sl_button_btn0) == SL_SIMPLE_BUTTON_POLARITY)) {
    sl_led_toggle(&sl_led_led0);
  } else if ((handle == &sl_button_btn1)
             && (sl_button_get_state(&sl_button_btn1) == SL_SIMPLE_BUTTON_POLARITY)) {
    sl_led_toggle(&sl_led_led1);
  }
}

/**************************************************************************//**
 * Configure the GPIO pins.
 *****************************************************************************/
static void gpio_setup(void)
{
  // Configure WF200 reset pin.
  GPIO_PinModeSet(SL_WFX_HOST_PINOUT_RESET_PORT, SL_WFX_HOST_PINOUT_RESET_PIN, gpioModePushPull, 0);
  // Configure WF200 WUP pin.
  GPIO_PinModeSet(SL_WFX_HOST_PINOUT_WUP_PORT, SL_WFX_HOST_PINOUT_WUP_PIN, gpioModePushPull, 0);
#ifdef  SL_CATALOG_WFX_BUS_SPI_PRESENT
  // GPIO used as IRQ.
  GPIO_PinModeSet(SL_WFX_HOST_PINOUT_SPI_WIRQ_PORT, SL_WFX_HOST_PINOUT_SPI_WIRQ_PIN, gpioModeInputPull, 0);

  if (GPIO->IEN & (1 << SL_WFX_HOST_PINOUT_SPI_WIRQ_PIN)) {
    uint8_t index_start = SL_WFX_HOST_PINOUT_SPI_WIRQ_PIN - SL_WFX_HOST_PINOUT_SPI_WIRQ_PIN % 4;
    uint8_t index_stop = index_start + 4;
    for (uint8_t i = index_start; i < index_stop; i++) {
      if ((GPIO->IEN & (1 << i)) == 0) {
        // Interrupt not used, use this EXTI line to reroute the interrupt
        wirq_irq_nb = i;
        break;
      }
    }

    if (wirq_irq_nb == SL_WFX_HOST_PINOUT_SPI_WIRQ_PIN) {
      // No unused interrupt line found
      printf("WFx interrupt line shared, it can cause unknown behaviors\r\n");
    }
  }

  GPIOINT_CallbackRegister(wirq_irq_nb, (GPIOINT_IrqCallbackPtr_t)wfx_interrupt);
#endif
}

static void start_task(void *p_arg)
{
  RTOS_ERR  err;
  PP_UNUSED_PARAM(p_arg); // Prevent compiler warning.

  // Clear the console and buffer
  printf("\033\143");
  printf("\033[3J");
  printf("Wi-Fi Commissioning Micrium OS Example\r\n");

  // Initialize the IO module.
  IO_Init(&err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

  // Call common module initialization.
  Common_Init(&err);
  APP_RTOS_ASSERT_CRITICAL(err.Code == RTOS_ERR_NONE,; );

  // Initialize the BSP.
  BSP_OS_Init();

  gpio_setup();

  //start wfx bus communication task.
  wfx_bus_start();
#ifdef SL_CATALOG_WFX_SECURE_LINK_PRESENT
  wfx_securelink_task_start(); // start securelink key renegotiation task
#endif //SL_CATALOG_WFX_SECURE_LINK_PRESENT

#ifdef SL_CATALOG_POWER_MANAGER_PRESENT
  // Prevent the program to go into EM2 sleep before the Wi-Fi
  // chip is initialized and as long the SoftAP is up.
  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif

  wifi_start();
  webpage_start();

  // Delete the init thread.
  OSTaskDel(0, &err);
}

/**************************************************************************//**
 * Wi-Fi Commissioning application init.
 *****************************************************************************/
void app_wifi_commissioning_init(void)
{
  RTOS_ERR err;

  OSTaskCreate(&start_task_tcb, // Create the Start Task.
               "Start Task",
               start_task,
               DEF_NULL,
               START_TASK_PRIO,
               &start_task_stk[0],
               (START_TASK_STK_SIZE / 10u),
               START_TASK_STK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}
