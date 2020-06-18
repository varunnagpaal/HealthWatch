/*!
 *****************************************************************************
 * @file:   main.c
 * @brief:
 * @version: $Revision: 0.1 $
 * @date:    $Date: 29-06-2019 $
 *-----------------------------------------------------------------------------
 *

#define SUCCESS 0
#define FAIL    1
#define
typedef enum AD_STATE
{
  AD_STATE_SLEEPING;
  AD_STATE_AWAKENED;
  AD_STATE_ATTEMPT_NRF_WAKEUP;
  AD_STATE_CONFIRMED_NRF_WAKEUP;
  AD_STATE_RX_SENSOR_X_DATA;
}

uint8_t Sleep()
{
  uint8_t result = SUCCESS;

  return result;
}

uint8_t IsrOnBtnPress()
{
  uint8_t result = SUCCESS;

  return result;
}
uint8_t InitTimer()
{

}

uint8_t InitGpio()
{

}

uint8_t InitSpi()
{

}

uint8_t InitClks()
{

}

uint8_t Init()
{
  uint8_t result = SUCCESS;

  result = InitGpio();
  CHECK( result );

  result = InitClks();
  CHECK( result );

  result = InitTimer();
  CHECK( result );

  result = InitSpi();
  CHECK( result );

  return result;
}

int main()
{
  uint8_t result = SUCCESS;
  result = Init();
  CHECK (result);

  result = SetupIsr(IsrOnBtnPress);
  CHECK( result );

  result = Sleep();
  CHECK( result );

  return result;
}
