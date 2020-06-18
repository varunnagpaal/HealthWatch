/** 
 * \file init.h
 * \brief Hardware initialization functions for AD module
 *
 * \author Varun Nagpal
 *
 * \date Oct, 14 2019
 *
 * \version 0.9
 *
 */
#ifndef HWATCH_INIT_H
#define HWATCH_INIT_H

#include "spi.h"
#include "common.h"
#include "common_ad.h"

extern HWATCH_AD_RESULT_TYPE InitClks(void);
extern HWATCH_AD_RESULT_TYPE InitGpio(void);
extern HWATCH_AD_RESULT_TYPE ConfigureGpio(void);
extern HWATCH_AD_RESULT_TYPE InitWatchDogTimer(void);
extern HWATCH_AD_RESULT_TYPE InitIntervalTimer(void);
extern HWATCH_AD_RESULT_TYPE InitSpi( ADI_SPI_DEV_ID_TYPE const devId,
                                    ADI_SPI_DEV_HANDLE * const phSpiDev,
                                    const HWATCH_SPI_MODE_TYPE spiMode,
                                    const bool_t bBlockingMode,
                                    const bool_t bDmaMode,
                                    const bool_t bContinuousMode );

extern HWATCH_AD_RESULT_TYPE UnInitClks(void);
extern HWATCH_AD_RESULT_TYPE UnInitGpio(void);
extern HWATCH_AD_RESULT_TYPE UnInitWatchDogTimer(void);
extern HWATCH_AD_RESULT_TYPE UnInitIntervalTimer(void);
extern HWATCH_AD_RESULT_TYPE UnInitSpi(ADI_SPI_DEV_HANDLE  * const hSpiDev);

#endif  /* HWATCH_INIT_H */
