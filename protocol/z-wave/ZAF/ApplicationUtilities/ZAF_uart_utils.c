/**
 * UART common utilities source file
 * @copyright 2019 Silicon Laboratories Inc.
 */


#include <ZAF_uart_utils.h>
#ifdef EFR32ZG23
#include <em_eusart.h>
#include <stddef.h>
#endif
#include <em_usart.h>
#include <em_cmu.h>
#include <ZW_typedefs.h>

static void UartEnable(USART_TypeDef * uart, uint32_t baudRate, bool bEnableTx, bool bEnableRx);
static void UartSetTxOutput(USART_TypeDef * usart, GPIO_Port_TypeDef port, unsigned int pin);
static void UartSetRxInput(USART_TypeDef * usart, GPIO_Port_TypeDef port, unsigned int pin);

// Ocelot has only one USART and two EUSART (enhanced USART) - so all functions refering to USART1, has been 
// mapped to EUSART1
#ifdef EFR32ZG23 //OCELOT_HARDCODED
static void EusartEnable(EUSART_TypeDef * eusart, uint32_t baudRate, bool bEnableTx, bool bEnableRx);
static void EusartSetTxOutput(EUSART_TypeDef * eusart, GPIO_Port_TypeDef port, unsigned int pin);
static void EusartSetRxInput(EUSART_TypeDef * eusart, GPIO_Port_TypeDef port, unsigned int pin);
#else
static uint32_t FindUartTxLocation(GPIO_Port_TypeDef port, unsigned int pin);
static uint32_t FindUartRxLocation(GPIO_Port_TypeDef port, unsigned int pin);
#endif

void
ZAF_UART0_init(GPIO_Port_TypeDef txPort,
               unsigned int txPin,
               GPIO_Port_TypeDef rxPort,
               unsigned int rxPin)
{
  USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;
  uartInit.enable = usartDisable;

  //UARTs clock must be enabled to configure UART
  CMU_ClockEnable(cmuClock_USART0, true);

  USART_InitAsync(USART0, &uartInit);
  UartSetTxOutput(USART0, txPort, txPin);
  UartSetRxInput(USART0, rxPort, rxPin);
  //disable UARTs clock
  CMU_ClockEnable(cmuClock_USART0, false);
}


#ifdef EFR32ZG23
void
ZAF_EUSART1_init(GPIO_Port_TypeDef txPort,
               unsigned int txPin,
               GPIO_Port_TypeDef rxPort,
               unsigned int rxPin)
{
  //UARTs clock must be enabled to configure UART
  EUSART_UartInit_TypeDef eusartInit = EUSART_UART_INIT_DEFAULT_HF;
  eusartInit.enable = usartDisable;

  //CMU_ClockEnable(cmuClock_GPIO, true); // Do I need this? OCELOT_HARDCODED
  //CMU_ClockEnable(cmuClock_PRS, true); // Do I need this? OCELOT_HARDCODED
  CMU_ClockEnable(cmuClock_EUSART1, true);

  EUSART_UartInitHf(EUSART1, &eusartInit);
  /* Enable pins at correct UART/USART location. */
  //GPIO->EUSARTROUTE[1].ROUTEEN = // OCELOT_HARDCODED - FIXME - do I need this?
  //  GPIO_EUSART_ROUTEEN_TXPEN | GPIO_EUSART_ROUTEEN_RXPEN;
  EusartSetTxOutput(EUSART1, txPort, txPin);
  EusartSetRxInput(EUSART1, rxPort, rxPin);

  //disable UARTs clock
  CMU_ClockEnable(cmuClock_EUSART1, false);
}
#endif

void
ZAF_UART1_init(GPIO_Port_TypeDef txPort,
               unsigned int txPin,
               GPIO_Port_TypeDef rxPort,
               unsigned int rxPin)
{
#ifdef EFR32ZG23
  ZAF_EUSART1_init(txPort, txPin, rxPort, rxPin);
#else
  USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;
  uartInit.enable = usartDisable;

  //UARTs clock must be enabled to configure UART
  CMU_ClockEnable(cmuClock_USART1, true);

  USART_InitAsync(USART1, &uartInit);
  UartSetTxOutput(USART1, txPort, txPin);
  UartSetRxInput(USART1, rxPort, rxPin);
  //disable UARTs clock
  CMU_ClockEnable(cmuClock_USART1, false);
#endif
}


void
ZAF_UART0_enable(
  uint32_t baudRate,   /* IN  Baud rate (e.g. 9600=>9600baud/s, 115200=>115200baud/s) */
  bool     bEnableTx,  /* IN  true: Tx enabled, false: Tx disabled */
  bool     bEnableRx)  /* IN  true: Rx enabled, false: Rx disabled */
{
  /* enable cmu clock for USART0 */
  CMU_ClockEnable(cmuClock_USART0, true);
  UartEnable(USART0, baudRate, bEnableTx, bEnableRx);
}


#ifdef EFR32ZG23
void
EUSART1_enable(
  uint32_t baudRate,  /* IN  Baud rate (e.g. 9600=>9600baud/s, 115200=>115200baud/s) */
  bool     bEnableTx,  /* IN  true: Tx enabled, false: Tx disabled */
  bool     bEnableRx)  /* IN  true: Rx enabled, false: Rx disabled */
{
  /* enable cmu clock for EUSART1 */
  //CMU_ClockEnable(cmuClock_PRS, true); // Do I need this? OCELOT_HARDCODED
  CMU_ClockEnable(cmuClock_EUSART1, true);
  EusartEnable(EUSART1, baudRate, bEnableTx, bEnableRx);
}
#endif

void
UART1_enable(
  uint32_t baudRate,  /* IN  Baud rate (e.g. 9600=>9600baud/s, 115200=>115200baud/s) */
  bool     bEnableTx,  /* IN  true: Tx enabled, false: Tx disabled */
  bool     bEnableRx)  /* IN  true: Rx enabled, false: Rx disabled */
{
#ifdef EFR32ZG23
  ZAF_UART1_enable(baudRate, bEnableTx, bEnableRx);
#else
  /* enable cmu clock for USART1 */
  CMU_ClockEnable(cmuClock_USART1, true);
  UartEnable(USART1, baudRate, bEnableTx, bEnableRx);
#endif
}

void
ZAF_UART1_enable(
  uint32_t baudRate,  /* IN  Baud rate (e.g. 9600=>9600baud/s, 115200=>115200baud/s) */
  bool     bEnableTx,  /* IN  true: Tx enabled, false: Tx disabled */
  bool     bEnableRx)  /* IN  true: Rx enabled, false: Rx disabled */
{
#ifdef EFR32ZG23
  EUSART1_enable(baudRate, bEnableTx, bEnableRx);
#else
  UART1_enable(baudRate, bEnableTx, bEnableRx);
#endif
}

void
ZAF_UART0_tx_send(
  const uint8_t * pData, uint32_t iLength)
{
  while (iLength > 0)
  {
    USART_Tx(USART0, *pData++);
    iLength--;
  }
}


#ifdef EFR32ZG23
void
EUSART1_tx_send(
  const uint8_t * pData, uint32_t iLength)
{
  while (iLength > 0)
  {
    EUSART_Tx(EUSART1, *pData++);
    iLength--;
  }
}
#endif

void
UART1_tx_send(
  const uint8_t * pData, uint32_t iLength)
{
#ifdef EFR32ZG23
  EUSART1_tx_send(pData, iLength);
#else
  while (iLength > 0)
  {
    USART_Tx(USART1, *pData++);
    iLength--;
  }
#endif
}

void
ZAF_UART1_tx_send(const uint8_t * pData,
                  uint32_t iLength)
{
#ifdef EFR32ZG23
  EUSART1_tx_send(pData, iLength);
#else
  UART1_tx_send(pData, iLength);
#endif
}

static void UartEnable(
  USART_TypeDef * uart,
  uint32_t baudRate,  /* IN  Baud rate (e.g. 9600=>9600baud/s, 115200=>115200baud/s) */
  bool     bEnableTx,  /* IN  true: Tx enabled, false: Tx disabled */
  bool     bEnableRx)  /* IN  true: Rx enabled, false: Rx disabled */
{
  USART_BaudrateAsyncSet(uart, 0, baudRate, usartOVS16);

  //disable rx and tx
#ifndef EFR32ZG23  
  uart->ROUTEPEN &= ~_USART_ROUTEPEN_MASK;
#else
  GPIO->USARTROUTE[0].ROUTEEN &=
    ~(GPIO_USART_ROUTEEN_TXPEN | GPIO_USART_ROUTEEN_RXPEN);
#endif

  if (bEnableTx)
  {
  //enable tx
#ifndef EFR32ZG23 // OCELOT_HARDCODED - FIXME - maybe this is not enough, not sure about ROUTEPEN
    uart->ROUTEPEN |= USART_ROUTEPEN_TXPEN;
#else
    GPIO->USARTROUTE[0].ROUTEEN |= GPIO_USART_ROUTEEN_TXPEN;
#endif
    USART_Enable(uart, usartEnableTx);
  }

  if (bEnableRx)
  {
  //enable rx
#ifndef EFR32ZG23 // OCELOT_HARDCODED - FIXME - maybe this is not enough, not sure about ROUTEPEN
    uart->ROUTEPEN |= USART_ROUTEPEN_RXPEN;
#else
    GPIO->USARTROUTE[0].ROUTEEN |= GPIO_USART_ROUTEEN_RXPEN;
#endif
    USART_Enable(uart, usartEnableRx);
  }

  if(bEnableTx && bEnableRx)
  {
    USART_Enable(uart, usartEnable);
  }
}

#ifdef EFR32ZG23
static void EusartEnable(
  EUSART_TypeDef * eusart,
  uint32_t baudRate,  /* IN  Baud rate (e.g. 9600=>9600baud/s, 115200=>115200baud/s) */
  bool     bEnableTx,  /* IN  true: Tx enabled, false: Tx disabled */
  bool     bEnableRx)  /* IN  true: Rx enabled, false: Rx disabled */
{

  EUSART_BaudrateSet(eusart, 0, baudRate);
  EUSART_Enable(EUSART1, eusartDisable);

  uint32_t enable = eusartDisable;  //disable rx and tx
  if (bEnableTx && bEnableRx) //enable rx and tx
    enable = eusartEnable;
  else if (bEnableTx)  //enable tx
    enable = eusartEnableTx;
  else if (bEnableRx)  //enable rx
    enable = eusartEnableRx;

  EUSART_Enable(EUSART1, enable);
}
#endif

static void UartSetTxOutput(USART_TypeDef * usart,
                            GPIO_Port_TypeDef port,
                            unsigned int pin)
{
  GPIO_PinModeSet(port, pin, gpioModePushPull, 1);
#ifdef EFR32ZG23 // there is only one USART in Ocelot - USART0
  UNUSED(usart);
  GPIO->USARTROUTE[0].TXROUTE =
    (port << _GPIO_USART_TXROUTE_PORT_SHIFT)
    | (pin << _GPIO_USART_TXROUTE_PIN_SHIFT);
#else
  usart->ROUTELOC0 &= ~_USART_ROUTELOC0_TXLOC_MASK;
  usart->ROUTELOC0 |= FindUartTxLocation(port, pin);
#endif
}

#ifdef EFR32ZG23
static void EusartSetTxOutput(EUSART_TypeDef * eusart,
                            GPIO_Port_TypeDef port,
                            unsigned int pin)
{
  GPIO_PinModeSet(port, pin, gpioModePushPull, 1);

  GPIO->EUSARTROUTE[EUSART_NUM(eusart)].ROUTEEN = GPIO_EUSART_ROUTEEN_TXPEN;
  GPIO->EUSARTROUTE[EUSART_NUM(eusart)].TXROUTE = (port << _GPIO_EUSART_TXROUTE_PORT_SHIFT)
                                                           | (pin << _GPIO_EUSART_TXROUTE_PIN_SHIFT);

}
#endif

static void UartSetRxInput(USART_TypeDef * usart,
                           GPIO_Port_TypeDef port,
                           unsigned int pin)
{
  GPIO_PinModeSet(port, pin, gpioModeInput, 0);
#ifdef EFR32ZG23 // there is only one USART in Ocelot - USART0
  UNUSED(usart);
  GPIO->USARTROUTE[0].RXROUTE =
    (port << _GPIO_USART_RXROUTE_PORT_SHIFT)
    | (pin << _GPIO_USART_RXROUTE_PIN_SHIFT);
#else
  usart->ROUTELOC0 &= ~_USART_ROUTELOC0_RXLOC_MASK;
  usart->ROUTELOC0 |= FindUartRxLocation(port, pin);
#endif
}

#ifdef EFR32ZG23
static void EusartSetRxInput(EUSART_TypeDef * eusart,
                           GPIO_Port_TypeDef port,
                           unsigned int pin)
{
  GPIO_PinModeSet(port, pin, gpioModeInput, 0);
  GPIO->EUSARTROUTE[EUSART_NUM(eusart)].RXROUTE =
    (port << _GPIO_EUSART_RXROUTE_PORT_SHIFT)
    | (pin << _GPIO_EUSART_RXROUTE_PIN_SHIFT);
}
#endif

#ifndef EFR32ZG23 // Ocelot has a different mapping
static uint32_t FindUartTxLocation(GPIO_Port_TypeDef port,
                                   unsigned int pin)
{
  switch(port)
  {
    case gpioPortA:
    {
      if(0 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC0;
      }
      else if(1 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC1;
      }
      else if(2 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC2;
      }
      else if(3 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC3;
      }
      else if(4 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC4;
      }
      else if(5 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC5;
      }
      break;
    }
    case gpioPortB:
    {
      if(11 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC6;
      }
      else if(12 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC7;
      }
      else if(13 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC8;
      }
      else if(14 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC9;
      }
      else if(15 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC10;
      }
      break;
    }
    case gpioPortC:
    {
      if(6 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC11;
      }
      else if(7 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC12;
      }
      else if(8 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC13;
      }
      else if(9 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC14;
      }
      else if(10 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC15;
      }
      else if(11 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC16;
      }
      break;
    }
    case gpioPortD:
    {
      if(13 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC21;
      }
      else if(14 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC22;
      }
      else if(15 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC23;
      }
      break;
    }
    case gpioPortF:
    {
      if(0 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC24;
      }
      else if(1 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC25;
      }
      else if(2 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC26;
      }
      else if(3 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC27;
      }
      else if(4 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC28;
      }
      else if(5 == pin)
      {
        return USART_ROUTELOC0_TXLOC_LOC29;
      }
      break;
    }
  }

  return USART_ROUTELOC0_TXLOC_LOC0;
}

static uint32_t FindUartRxLocation(GPIO_Port_TypeDef port,
                                   unsigned int pin)
{
  switch(port)
  {
    case gpioPortA:
    {
      if(0 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC31;
      }
      else if(1 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC0;
      }
      else if(2 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC1;
      }
      else if(3 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC2;
      }
      else if(4 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC3;
      }
      else if(5 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC4;
      }
      break;
    }
    case gpioPortB:
    {
      if(11 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC5;
      }
      else if(12 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC6;
      }
      else if(13 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC7;
      }
      else if(14 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC8;
      }
      else if(15 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC9;
      }
      break;
    }
    case gpioPortC:
    {
      if(6 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC10;
      }
      else if(7 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC11;
      }
      else if(8 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC12;
      }
      else if(9 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC13;
      }
      else if(10 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC14;
      }
      else if(11 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC15;
      }
      break;
    }
    case gpioPortD:
    {
      if(13 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC20;
      }
      else if(14 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC21;
      }
      else if(15 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC22;
      }
      break;
    }
    case gpioPortF:
    {
      if(0 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC23;
      }
      else if(1 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC24;
      }
      else if(2 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC25;
      }
      else if(3 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC26;
      }
      else if(4 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC27;
      }
      else if(5 == pin)
      {
        return USART_ROUTELOC0_RXLOC_LOC28;
      }
      break;
    }
  }

  return USART_ROUTELOC0_RXLOC_LOC0;
}
#endif
