/** 
 * \file fsm.h
 * \brief nRF state machine functions
 *
 *
 * \author Varun Nagpal
 *
 * \date Oct, 14 2019
 *
 * \version 0.9
 *
 */
 
#ifndef HWATCH_NRF_FSM_H
#define HWATCH_NRF_FSM_H

#include "common_nrf.h"

/* State Logic Functions */
extern HWATCH_NRF_STATE_TYPE NRF_State_PowerOnReset( void );
extern HWATCH_NRF_STATE_TYPE NRF_State_DeepSleep( void );
extern HWATCH_NRF_STATE_TYPE NRF_State_HardwareInit( void );
extern HWATCH_NRF_STATE_TYPE NRF_State_Nap( void );
extern HWATCH_NRF_STATE_TYPE NRF_State_Wait( void );
extern HWATCH_NRF_STATE_TYPE NRF_State_ConfirmWakeupToAdi( void );
extern HWATCH_NRF_STATE_TYPE NRF_State_Pair( void );
extern HWATCH_NRF_STATE_TYPE NRF_State_WaitForUserDetails( void );
extern HWATCH_NRF_STATE_TYPE NRF_State_WaitForUserToWear( void );
extern HWATCH_NRF_STATE_TYPE NRF_State_StoringSensorData( void );
extern HWATCH_NRF_STATE_TYPE NRF_State_StreamSensorData( void );
extern HWATCH_NRF_STATE_TYPE NRF_State_Shutdown( void );

#endif  /* HWATCH_NRF_FSM_H */
