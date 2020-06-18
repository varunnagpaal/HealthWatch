/*
 **
 ** Source file generated on July 3, 2019 at 03:14:30.	
 **
 ** Copyright (C) 2019 Analog Devices Inc., All Rights Reserved.
 **
 ** This file is generated automatically based upon the options selected in 
 ** the Pin Multiplexing configuration editor. Changes to the Pin Multiplexing
 ** configuration should be made by changing the appropriate options rather
 ** than editing this file.
 **
 ** Selected Peripherals
 ** --------------------
 ** CAPTOUCH (Left)
 ** SPI0 (SCLK, MISO, MOSI, CS)
 **
 ** GPIO (unavailable)
 ** ------------------
 ** P0_01, P3_00, P3_01, P3_02, P3_03
 */
 /** 
 * \file pinmux.c
 * \brief GPIO pin muxing functions for AD module
 *
 * \author Varun Nagpal
 *
 * \date Oct, 14 2019
 *
 * \version 0.9
 *
 */

#include "device.h"

#define CAPTOUCH_LEFT_PORTP0_MUX  ((uint16_t) ((uint16_t) 1<<2))
#define SPI0_SCLK_PORTP3_MUX  ((uint16_t) ((uint16_t) 1<<0))
#define SPI0_MISO_PORTP3_MUX  ((uint16_t) ((uint16_t) 1<<2))
#define SPI0_MOSI_PORTP3_MUX  ((uint16_t) ((uint16_t) 1<<4))
#define SPI0_CS_PORTP3_MUX  ((uint16_t) ((uint16_t) 1<<6))

int32_t adi_initpinmux(void);

/*
 * Initialize the Port Control MUX Registers
 */
int32_t adi_initpinmux(void) {
    /* Port Control MUX registers */
    *((volatile uint32_t *)REG_GPIO0_GPCON) = CAPTOUCH_LEFT_PORTP0_MUX;
    *((volatile uint32_t *)REG_GPIO3_GPCON) = SPI0_SCLK_PORTP3_MUX | SPI0_MISO_PORTP3_MUX
     | SPI0_MOSI_PORTP3_MUX | SPI0_CS_PORTP3_MUX;

    return 0;
}

