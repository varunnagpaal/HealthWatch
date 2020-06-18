/*
 **
 ** Source file generated on July 2, 2019 at 17:32:02.
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
 ** SPI0 (SCLK, MISO, MOSI, CS)
 **
 ** GPIO (unavailable)
 ** ------------------
 ** P3_00, P3_01, P3_02, P3_03
 */
/** 
 * \file spi0_pinmux.c
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
#include "pin_mux.h"

/* Set SPI0 pins */
#define SPI0_SCLK_PORTP3_MUX  ((uint16_t) ((uint16_t) 1<<0))
#define SPI0_MISO_PORTP3_MUX  ((uint16_t) ((uint16_t) 1<<2))
#define SPI0_MOSI_PORTP3_MUX  ((uint16_t) ((uint16_t) 1<<4))
#define SPI0_CS_PORTP3_MUX  ((uint16_t) ((uint16_t) 1<<6))

/* Clear SPI0 pins */
#define PORTP3_PIN0_MUX_GPIO ((uint16_t) ((uint16_t) 0<<0))
#define PORTP3_PIN1_MUX_GPIO ((uint16_t) ((uint16_t) 0<<2))
#define PORTP3_PIN2_MUX_GPIO ((uint16_t) ((uint16_t) 0<<4))
#define PORTP3_PIN3_MUX_GPIO ((uint16_t) ((uint16_t) 0<<6))

/* Clear UART0 pins */
#define GPIO6_PORTP0_MUX  ((uint16_t) ((uint16_t) 1<<12))
#define GPIO7_PORTP0_MUX  ((uint16_t) ((uint16_t) 1<<14))

/*
 * Initialize the Port Control MUX Registers
 */
int32_t adi_initpinmux_setSPI0(void) {
    /* Port Control MUX registers */
    *((volatile uint32_t *)REG_GPIO3_GPCON) = SPI0_SCLK_PORTP3_MUX | SPI0_MISO_PORTP3_MUX
     | SPI0_MOSI_PORTP3_MUX | SPI0_CS_PORTP3_MUX;

    return 0;
}

int32_t adi_initpinmux_clearSPI0(void) {
    /* Port Control MUX registers */
    *((volatile uint32_t *)REG_GPIO3_GPCON) = PORTP3_PIN0_MUX_GPIO | PORTP3_PIN1_MUX_GPIO
     | PORTP3_PIN2_MUX_GPIO | PORTP3_PIN3_MUX_GPIO;

    return 0;
}

int32_t adi_initpinmux_clearUART0(void)
{
    /* Port Control MUX registers */
    *((volatile uint32_t *)REG_GPIO0_GPCON) = GPIO6_PORTP0_MUX | GPIO7_PORTP0_MUX;

    return 0;  
}
