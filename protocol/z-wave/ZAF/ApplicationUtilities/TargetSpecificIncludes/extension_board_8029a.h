/**
 * Provides support for BRD8029A (Buttons and LEDs EXP Board)
 *
 * @copyright 2018 Silicon Laboratories Inc.
 */

#ifndef EXTENSION_BOARD_8029A_H
#define EXTENSION_BOARD_8029A_H

#ifdef EFR32ZG23  // Ocelot
#include <extension_board_8029a_efr32zg23.h>
#else             // 700s (Nixi)
#include <extension_board_8029a_efr32xg13.h>
#endif

#endif /* EXTENSION_BOARD_8029A_H */
