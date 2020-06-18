/** 
 * \file fsm.h
 * \brief Finite State Machine functions for AD module
 *
 * \author Varun Nagpal
 *
 * \date Oct, 14 2019
 *
 * \version 0.9
 *
 */
 
#ifndef HWATCH_AD_FSM_H
#define HWATCH_AD_FSM_H

#include "gpt.h"
#include "common_ad.h"

/* State Logic Functions */
extern HWATCH_AD_STATE_TYPE AD_State_HardwareInit( void );
extern HWATCH_AD_STATE_TYPE AD_State_Sleep( void );
extern HWATCH_AD_STATE_TYPE AD_State_Wait( void );
extern HWATCH_AD_STATE_TYPE AD_State_WaitForBleToWakeup( void );
extern HWATCH_AD_STATE_TYPE AD_State_WaitForBleToPair( void );
extern HWATCH_AD_STATE_TYPE AD_State_WaitForUserDetails( void );
extern HWATCH_AD_STATE_TYPE AD_State_WaitForUserToWear( void );
extern HWATCH_AD_STATE_TYPE AD_State_StoreSensorData( void );
extern HWATCH_AD_STATE_TYPE AD_State_StreamSensorData( void );
extern HWATCH_AD_STATE_TYPE AD_State_Shutdown( void );

/* Helper Functions */
static HWATCH_AD_RESULT_TYPE InitSystemHardware(void);
static HWATCH_AD_RESULT_TYPE UnInitSystemHardware(void);

#endif  /* HWATCH_ADI_FSM_H */
