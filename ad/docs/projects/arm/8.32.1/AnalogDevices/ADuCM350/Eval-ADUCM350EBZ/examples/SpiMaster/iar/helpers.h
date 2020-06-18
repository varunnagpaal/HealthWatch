/** 
 * \file helpers.h
 * \brief Helper functions for AD module
 *
 * \author Varun Nagpal
 *
 * \date Oct, 14 2019
 *
 * \version 0.9
 *
 */
 
#ifndef HWATCH_AD_HELPERS_H
#define HWATCH_AD_HELPERS_H

#include "device.h"
#include "common_ad.h"

extern HWATCH_AD_RESULT_TYPE Sleep( const ADI_SYS_POWER_MODE sleepMode,
                                  bool_t volatile * const bLowPowerExitFlag );

extern HWATCH_AD_RESULT_TYPE StartTimer( ADI_GPT_HANDLE * const pTimer,
                                       const IRQn_Type timerIRQn,
                                       float timeout_sec,
                                       bool_t sleepAfter,
                                       const ADI_SYS_POWER_MODE sleepMode,
                                       bool_t volatile * pbLowPowerExitFlag );

extern HWATCH_AD_RESULT_TYPE EnableTimer( ADI_GPT_HANDLE * const pTimer,
                                        const IRQn_Type timerIRQn );

extern HWATCH_AD_RESULT_TYPE StopTimer( ADI_GPT_HANDLE * const pTimer,
                                      const IRQn_Type timerIRQn );

extern HWATCH_AD_RESULT_TYPE ResetTimer( ADI_GPT_HANDLE * const pTimer,
                                       const IRQn_Type timerIRQn,
                                       bool_t volatile * const pbTimerExpiredFlag );

extern HWATCH_AD_RESULT_TYPE EnableBtnIrq(void);
extern HWATCH_AD_RESULT_TYPE DisableBtnIrq(void);

extern void EnableBleAckIrq(void);
extern void DisableBleAckIrq(void);
extern void ClearBleAckPendingIrq(void);
extern void SetBleAckIrqPriority( uint32_t const priority );

extern void AssertWakeUpToBle(void);
extern void DeAssertWakeUpToBle(void);
#endif /* HWATCH_AD_HELPERS_H */
