/** 
 * \file pin_mux.h
 * \brief GPIO pin muxing functions for AD module
 *
 * \author Varun Nagpal
 *
 * \date Oct, 14 2019
 *
 * \version 0.9
 *
 */
#ifndef HWATCH_AD_PIN_MUX_H
#define HWATCH_AD_PIN_MUX_H

#include <stdint.h>

extern int32_t adi_initpinmux_setSPI0(void);
extern int32_t adi_initpinmux_clearSPI0(void);
extern int32_t adi_initpinmux_clearUART0(void);

#endif /* HWATCH_AD_PIN_MUX_H */
